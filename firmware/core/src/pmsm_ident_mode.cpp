#include "pmsm_ident_mode.h"

#include <cmath>
#include <limits>

pmsm_ident_mode::pmsm_ident_mode(motor_channel* mtr_ch_, Messaging* msg_, time_interface* time_)
    : mode(mtr_ch_, msg_, time_),
      current_pid(pid_kp, pid_ki, pid_kd, 1.0f / 1000.0f, -max_ident_test_voltage, max_ident_test_voltage),
      current_filter(current_filter_cutoff_hz, 1000.0f)
{
    mode_msgs = mtr_ch_->get_motor_msgs().mode.pmsm_ident;
}

void pmsm_ident_mode::configure_control_objects() {
    // Real PWM frequency isn't known until the motor channel's timer has been configured, which
    // happens well after this object (static-storage-duration, held in device::channel_modes) is
    // constructed -- so the pid_controller/low_pass_filter built in the constructor use safe
    // placeholder dt/sample-rate values, and get rebuilt here with the real numbers once this
    // mode actually starts running (called once from user_reset()).
    pwm_frequency_hz = static_cast<float>(mtr_ch->calculate_pwm_cycles_from_us(1000000));
    pwm_dt_seconds = 1.0f / pwm_frequency_hz;

    current_pid = pid_controller(pid_kp, pid_ki, pid_kd, pwm_dt_seconds, -max_ident_test_voltage, max_ident_test_voltage);
    current_filter = low_pass_filter(current_filter_cutoff_hz, pwm_frequency_hz);

    step_settle_min_dwell_cycles          = mtr_ch->calculate_pwm_cycles_from_us(step_settle_min_dwell_us);
    step_settle_timeout_cycles            = mtr_ch->calculate_pwm_cycles_from_us(step_settle_timeout_us);
    current_ramp_settle_cycles            = mtr_ch->calculate_pwm_cycles_from_us(current_ramp_settle_us);
    sample_average_cycles                 = mtr_ch->calculate_pwm_cycles_from_us(sample_average_us);
    step_timeout_cycles                   = mtr_ch->calculate_pwm_cycles_from_us(step_timeout_us);
    inductance_ramp_down_cycles           = mtr_ch->calculate_pwm_cycles_from_us(inductance_ramp_down_us);
    inductance_settle_zero_timeout_cycles = mtr_ch->calculate_pwm_cycles_from_us(inductance_settle_zero_timeout_us);
    inductance_ripple_accum_cycles        = mtr_ch->calculate_pwm_cycles_from_us(inductance_ripple_accum_us);

    // Cycles per half-period of the ripple test's alternation, from a fixed frequency rather than
    // a fixed cycle count, so it stays consistent if pwm_frequency_hz ever changes.
    uint32_t half_period_cycles = static_cast<uint32_t>(pwm_frequency_hz / (2.0f * inductance_ripple_test_frequency_hz) + 0.5f);
    inductance_ripple_alternation_cycles = half_period_cycles > 0 ? half_period_cycles : 1;
}

void pmsm_ident_mode::user_reset() {
    stage = enable_resistance_measurement ? ident_stage::RESISTANCE
          : enable_inductance_measurement ? ident_stage::INDUCTANCE
          : ident_stage::DONE;

    step_number = 0;
    hex_cursor = 0;
    r_state = resistance_state::RAMP_TO_LEVEL;
    levels_remaining = step_is_recording() ? num_test_levels : 1;
    step_is_new_position = true;

    l_state = inductance_state::SETTLE_ZERO;
    current_pair = phase_pair::UV;

    saturation_cycle_counter = 0;
    stable_at_target_count = 0;
    state_cycle_counter = 0;
    sample_i_accum = 0.0f;
    sample_v_accum = 0.0f;
    sample_quiet_i_accum = 0.0f;
    sample_accum_count = 0;

    ripple_unfolded_accum = 0.0f;
    ripple_voltage_accum = 0.0f;
    prev_raw_loop_i = 0.0f;
    prev_command_sign = 1;
    have_prev_raw_i = false;
    ramp_down_start_v = 0.0f;
    current_test_voltage = inductance_test_step_voltage;
    inductance_voltage_retry_count = 0;

    for(uint8_t i = 0; i < 3; i++) {
        pair_results[i] = pair_result{};
    }
    for(uint8_t i = 0; i < 6; i++) {
        combo_results[i] = combo_result{};
    }

    regression.reset();
    current_pid.reset();
    current_filter.reset(0.0f);

    configure_control_objects();

    // Reduces residual IGBT-Vsat/Rds_on-curvature bias the resistance regression's slope can't
    // fully separate from true R -- resistance-only, since the ripple current in the inductance
    // stage oscillates through zero rapidly and this correction is triggered by measured current
    // sign (with a small deadband): a fast-dithering near-zero ripple would trip it inconsistently,
    // injecting a correction that doesn't track the clean alternating voltage command and doesn't
    // cancel through the ripple method's differencing the way a genuinely constant bias would.
    mtr_ch->set_basic_deadtime_compensation(stage == ident_stage::RESISTANCE);
}

mode::function_results pmsm_ident_mode::user_run_loop() {
    return function_results::COMPLETE;
}

mode::function_results pmsm_ident_mode::user_run_flagged_timer_update() {
    return function_results::COMPLETE;
}

mode::function_results pmsm_ident_mode::user_run_flagged_systick() {
    return function_results::COMPLETE;
}

mode::function_results pmsm_ident_mode::user_run_flagged_all_adc_eoc() {
    switch(stage) {
        case ident_stage::RESISTANCE: return run_resistance_stage();
        case ident_stage::INDUCTANCE: return run_inductance_stage();
        case ident_stage::DONE:       return function_results::COMPLETE;
    }
    return function_results::ERROR; // unreachable, silences -Wreturn-type
}

float pmsm_ident_mode::command_pair_voltage(phase_pair pair, float v_diff) {
    const pair_phases& pf = pair_phase_table[static_cast<uint8_t>(pair)];
    float v[3] = {0.0f, 0.0f, 0.0f};
    v[pf.plus_idx]  =  0.5f * v_diff;
    v[pf.minus_idx] = -0.5f * v_diff;
    v[pf.quiet_idx] = 0.0f;
    float achieved[3] = {0.0f, 0.0f, 0.0f};
    mtr_ch->set_phase_voltage(v[0], v[1], v[2], &achieved[0], &achieved[1], &achieved[2]);
    return achieved[pf.plus_idx] - achieved[pf.minus_idx];
}

void pmsm_ident_mode::read_pair_currents(phase_pair pair, float& loop_current_out, float& quiet_current_out) {
    const pair_phases& pf = pair_phase_table[static_cast<uint8_t>(pair)];
    float iu, iv, iw;
    mtr_ch->get_phase_currents(iu, iv, iw);
    float i[3] = {iu, iv, iw};
    loop_current_out = 0.5f * (i[pf.plus_idx] - i[pf.minus_idx]);
    quiet_current_out = i[pf.quiet_idx];
}

float pmsm_ident_mode::combine_slopes(float a, float b, bool valid_a, bool valid_b, bool& out_valid, bool& out_mismatch) {
    out_mismatch = false;
    out_valid = valid_a && valid_b;
    if(!out_valid) return 0.0f;

    float avg = 0.5f * (a + b);
    if(std::fabs(avg) > 1e-6f && std::fabs(a - b) > slope_mismatch_fraction_threshold * std::fabs(avg)) {
        out_mismatch = true;
    }
    return avg;
}

void pmsm_ident_mode::finish_level() {
    levels_remaining--;
    step_is_new_position = false; // any further levels tested in this step are same-position magnitude changes
    state_cycle_counter = 0;
    r_state = (levels_remaining > 0) ? resistance_state::RAMP_TO_LEVEL : resistance_state::ADVANCE;
}

mode::function_results pmsm_ident_mode::run_resistance_stage() {
    // ================================================================
    // Six-step "hexagon" resistance identification.
    //
    // Each step commands one of the six (pair, sign) combinations in hex_sequence, 60 degrees
    // apart from the step before and after it -- unlike reversing a single pair's polarity in
    // place (a 180 degree flip, landing exactly on the unstable equilibrium of the new target and
    // producing slow/unpredictable rotor motion), every transition here is a real, predictable,
    // single-direction torque step, so a plain ramp-and-confirm is enough with no dither needed.
    //
    // Within a step, the num_test_levels current levels are tested from max down to min --
    // levels_remaining counts down num_test_levels..1 and test_level_fractions[levels_remaining-1]
    // is always the fraction under test. Starting at max gives the strongest, most decisive pull
    // into the new 60-degree position right when that's needed; only the first (max) level of a
    // step needs step_is_new_position's confirmed settle, since the smaller levels after it are
    // just a magnitude change at the same position.
    // ================================================================

    const hex_combo& combo = current_combo();
    float target_current = static_cast<float>(combo.sign) * test_level_fractions[levels_remaining - 1] * motor_calibration_current;

    float loop_i = 0.0f, quiet_i = 0.0f;
    read_pair_currents(combo.pair, loop_i, quiet_i);
    float filtered_i = current_filter.update(loop_i);
    float v_cmd = current_pid.update(target_current, filtered_i);
    float v_achieved = command_pair_voltage(combo.pair, v_cmd);

    switch(r_state) {
        case resistance_state::RAMP_TO_LEVEL:
        {
            if(std::fabs(v_cmd) >= (max_ident_test_voltage - 0.01f)) {
                saturation_cycle_counter++;
            } else {
                saturation_cycle_counter = 0;
            }

            if(std::fabs(filtered_i - target_current) < reach_target_tolerance_amps) {
                saturation_cycle_counter = 0;
                state_cycle_counter = 0;
                stable_at_target_count = 0;
                r_state = resistance_state::SETTLE;
            } else if(saturation_cycle_counter >= step_timeout_cycles) {
                // Can't reach this level -- likely an open/high-resistance phase, or (for a
                // same-position lower level) a fluke; either way skip it rather than sample bad
                // data. If every level in this step fails this way, regression stays invalid
                // (fewer than 2 points) and this step's combo is correctly left unmeasured.
                msg->add(mode_msgs.resistance_too_high, 0);
                saturation_cycle_counter = 0;
                finish_level();
            }
            break;
        }
        case resistance_state::SETTLE:
        {
            state_cycle_counter++;

            if(step_is_new_position) {
                // Just moved 60 degrees -- confirm the current has genuinely stabilized at the
                // new target (not just "time has passed"). If the rotor were still settling into
                // its new alignment, residual back-EMF would keep perturbing the filtered current
                // away from target instead of letting it hold steady for this many cycles in a row.
                if(state_cycle_counter < step_settle_min_dwell_cycles) break;

                bool at_target = std::fabs(filtered_i - target_current) < reach_target_tolerance_amps;
                stable_at_target_count = at_target ? (stable_at_target_count + 1) : 0;

                if(stable_at_target_count < step_settle_consecutive_samples && state_cycle_counter < step_settle_timeout_cycles) {
                    break;
                }
                if(stable_at_target_count < step_settle_consecutive_samples) {
                    // Never confirmed stable -- report and proceed anyway rather than hang; the
                    // phase-imbalance check below still catches a lingering effect on the sample.
                    msg->add(mode_msgs.resistance_step_settle_timeout, 0);
                }
            } else if(state_cycle_counter < current_ramp_settle_cycles) {
                // Same hexagon position, just a smaller magnitude -- a fixed dwell is enough.
                break;
            }

            state_cycle_counter = 0;
            sample_i_accum = 0.0f;
            sample_v_accum = 0.0f;
            sample_quiet_i_accum = 0.0f;
            sample_accum_count = 0;
            r_state = resistance_state::SAMPLE;
            break;
        }
        case resistance_state::SAMPLE:
        {
            sample_i_accum += filtered_i;
            sample_v_accum += v_achieved; // actual achieved voltage, not the PID's nominal v_cmd -- see command_pair_voltage()'s comment
            sample_quiet_i_accum += quiet_i;
            sample_accum_count++;
            state_cycle_counter++;
            if(state_cycle_counter < sample_average_cycles) break;

            float avg_i = sample_i_accum / static_cast<float>(sample_accum_count);
            float avg_v = sample_v_accum / static_cast<float>(sample_accum_count);
            float avg_quiet_i = sample_quiet_i_accum / static_cast<float>(sample_accum_count);

            if(step_is_recording()) {
                regression.add_sample(avg_i, avg_v); // V = R*I + V_offset -- fit slope = R
            }

            // Quiet-phase imbalance, evaluated once per level using this settled/averaged window
            // -- a healthy motor carries ~0 current on the undriven phase at steady state.
            if(std::fabs(avg_i) > imbalance_min_loop_current &&
               std::fabs(avg_quiet_i) > imbalance_fraction_threshold * std::fabs(avg_i)) {
                msg->add(mode_msgs.resistance_phase_sense_imbalance, 0);
            }

            finish_level();
            break;
        }
        case resistance_state::ADVANCE:
        {
            // This step's sweep is done -- store its slope (only if this step's data is meant to
            // be recorded, i.e. it's in the later full cycle of its half) and move to the next
            // hexagon step, 60 degrees further in whichever direction this half is going.
            if(step_is_recording()) {
                bool have_slope = regression.valid();
                float slope = have_slope ? regression.slope() : 0.0f;
                combo_result& cr = combo_results[hex_cursor];
                if(step_is_forward()) {
                    cr.slope_fwd = slope;
                    cr.fwd_valid = have_slope;
                } else {
                    cr.slope_bwd = slope;
                    cr.bwd_valid = have_slope;
                }
            }

            current_pid.reset();
            current_filter.reset(0.0f);
            regression.reset();
            saturation_cycle_counter = 0;
            stable_at_target_count = 0;
            state_cycle_counter = 0;

            step_number++;
            if(step_number >= total_steps) {
                resolve_pair_results_from_combos();
                solve_resistance_from_pairs();
                step_number = 0;
                hex_cursor = 0;
                stage = enable_inductance_measurement ? ident_stage::INDUCTANCE : ident_stage::DONE;
                if(stage == ident_stage::INDUCTANCE) {
                    // See user_reset()'s comment -- deadtime compensation is resistance-only.
                    mtr_ch->set_basic_deadtime_compensation(false);
                }
            } else {
                advance_hex_cursor();
            }
            levels_remaining = step_is_recording() ? num_test_levels : 1; // lead-in steps just need to firmly seat the rotor, not a full regression sweep
            step_is_new_position = true;
            r_state = resistance_state::RAMP_TO_LEVEL;
            break;
        }
    }

    return function_results::CONTINUE;
}

mode::function_results pmsm_ident_mode::run_inductance_stage() {

    float loop_i = 0.0f, quiet_i = 0.0f;
    read_pair_currents(current_pair, loop_i, quiet_i);
    (void)quiet_i; // not used by the ripple method -- the zero-average-torque property doesn't depend on it

    switch(l_state) {
        case inductance_state::SETTLE_ZERO:
        {
            command_pair_voltage(current_pair, 0.0f);
            state_cycle_counter++;
            float filtered_i = current_filter.update(loop_i);
            bool settled = std::fabs(filtered_i) < zero_current_tolerance_amps;
            bool timed_out = state_cycle_counter >= inductance_settle_zero_timeout_cycles;
            if((settled && state_cycle_counter >= current_ramp_settle_cycles) || timed_out) {
                if(timed_out && !settled) {
                    // Current didn't fully decay before starting the ripple test -- reduces
                    // confidence in the first sample, not fatal (still bounded by the alternation
                    // period, which quickly re-zeroes the average). Not fatal on its own; proceed
                    // anyway rather than hang.
                    msg->add(mode_msgs.current_decay_timeout, 0);
                }
                // A clean (near-zero) starting current avoids biasing the first ripple sample --
                // no mechanical settle-confirmation needed here (unlike resistance's hexagon
                // steps), since the ripple test itself has ~zero average torque by construction.
                state_cycle_counter = 0;
                ripple_unfolded_accum = 0.0f;
                ripple_voltage_accum = 0.0f;
                have_prev_raw_i = false;
                regression.reset();
                l_state = inductance_state::RIPPLE_ACCUM;
            }
            break;
        }
        case inductance_state::RIPPLE_ACCUM:
        {
            // Symmetric bipolar square-wave voltage injection (ODrive-style): alternate sign
            // every inductance_ripple_alternation_cycles cycles. For a PMSM the magnet-alignment
            // torque term is linear in the injected current vector, so it cancels exactly over
            // one full alternation period regardless of which pair is driven -- this is why no
            // mechanical settle/hexagon-walk is needed for this stage. See
            // inductance_ripple_alternation_cycles' comment for why this isn't just alternated
            // every single cycle: that maximizes sensitivity to getting the pipeline delay exactly
            // right below, at the cost of zero margin for error if it isn't.
            uint32_t alternation_cycles = inductance_ripple_alternation_cycles > 0 ? inductance_ripple_alternation_cycles : 1;
            int8_t sign = ((state_cycle_counter / alternation_cycles) % 2 == 0) ? 1 : -1;
            float v_achieved = command_pair_voltage(current_pair, static_cast<float>(sign) * current_test_voltage);
            // Tracks what was actually delivered, not just what was commanded -- current_test_
            // voltage can be silently clamped down (VBUS rail, enforce_current_sense_duty_limit(),
            // enforce_min_pulse_width()), and using the nominal command in L=V/dI_dt when less was
            // actually delivered reads out as a falsely high L.
            ripple_voltage_accum += std::fabs(v_achieved);

            // Raw (unfiltered) current: current_filter's ~200Hz cutoff would heavily attenuate and
            // phase-lag a signal alternating this fast, silently corrupting dI/dt. Also note the
            // one-PWM-cycle pipeline delay (read_pair_currents() returns the previous cycle's
            // conversion) -- the delta between this cycle's raw reading and last cycle's is the
            // response to the sign that was commanded on the PREVIOUS cycle, so prev_command_sign
            // (stashed from last cycle) is paired with THIS delta, not the sign just commanded above.
            if(have_prev_raw_i) {
                float delta = loop_i - prev_raw_loop_i;
                ripple_unfolded_accum += static_cast<float>(prev_command_sign) * delta;
                regression.add_sample(static_cast<float>(state_cycle_counter), ripple_unfolded_accum);
            }
            prev_raw_loop_i = loop_i;
            prev_command_sign = sign;
            have_prev_raw_i = true;

            state_cycle_counter++;
            if(state_cycle_counter >= inductance_ripple_accum_cycles) {
                ramp_down_start_v = static_cast<float>(sign) * current_test_voltage;
                state_cycle_counter = 0;
                l_state = inductance_state::RAMP_DOWN;
            }
            break;
        }
        case inductance_state::RAMP_DOWN:
        {
            state_cycle_counter++;
            uint32_t denom = inductance_ramp_down_cycles > 0 ? inductance_ramp_down_cycles : 1;
            float frac = static_cast<float>(state_cycle_counter) / static_cast<float>(denom);
            if(frac > 1.0f) frac = 1.0f;
            float v_cmd = lerp(ramp_down_start_v, 0.0f, frac);
            command_pair_voltage(current_pair, v_cmd);

            // The lerp above only ramps the commanded VOLTAGE to zero on a fixed schedule -- it
            // doesn't confirm the winding CURRENT has actually decayed before PAIR_DONE switches
            // which phase is "quiet" for the next pair. An abrupt topology switch with residual
            // current still flowing is an uncontrolled transient, not a clean handoff -- the same
            // concern resistance's hexagon walk explicitly guards against with a confirmed-zero
            // exit rather than a fixed wait (see its RAMP_TO_ZERO). The zero-average-torque
            // argument covers the alternation WITHIN a pair's test; it says nothing about this
            // transition. Once the lerp reaches 0 (frac clamps to 1.0, so v_cmd stays 0), keep
            // holding there until the filtered current is confirmed near zero, with a timeout
            // safety net so a stuck reading doesn't hang here.
            float filtered_i = current_filter.update(loop_i);
            bool ramp_done = state_cycle_counter >= inductance_ramp_down_cycles;
            bool current_settled = std::fabs(filtered_i) < zero_current_tolerance_amps;
            bool timed_out = state_cycle_counter >= inductance_settle_zero_timeout_cycles;

            if(ramp_done && (current_settled || timed_out)) {
                if(timed_out && !current_settled) {
                    msg->add(mode_msgs.current_decay_timeout, 0);
                }
                state_cycle_counter = 0;

                float dI_dt = regression.valid() ? regression.slope() * pwm_frequency_hz : 0.0f;
                bool have_signal = regression.valid() && std::fabs(dI_dt) > inductance_ripple_min_dIdt_amps_per_sec;

                // Average ACTUAL achieved voltage over this attempt's accumulation window, not
                // current_test_voltage's nominal command -- if it got clamped down (VBUS rail,
                // enforce_current_sense_duty_limit(), enforce_min_pulse_width()), using the
                // nominal value here would silently read out as a too-high L. Used both for the
                // L=V/dI_dt formula below and as the scaling basis for the next voltage estimate.
                uint32_t accum_cycle_count = inductance_ripple_accum_cycles > 0 ? inductance_ripple_accum_cycles : 1;
                float avg_achieved_voltage = ripple_voltage_accum / static_cast<float>(accum_cycle_count);

                // half_period_seconds converts dI/dt into this attempt's actual ripple peak
                // amplitude, so it can be compared against a target expressed in amps (tied to
                // motor_calibration_current) rather than an abstract amps/sec threshold.
                float half_period_seconds = static_cast<float>(inductance_ripple_alternation_cycles) / pwm_frequency_hz;
                float achieved_ripple_amps = have_signal ? std::fabs(dI_dt) * half_period_seconds : 0.0f;
                float target_ripple_amps = inductance_ripple_target_current_fraction * motor_calibration_current;
                bool close_enough = have_signal && achieved_ripple_amps >= inductance_ripple_accept_fraction_of_target * target_ripple_amps;

                if(!close_enough && current_test_voltage < max_ident_test_voltage && inductance_voltage_retry_count < inductance_test_voltage_max_retries) {
                    // Ripple too small relative to target -- likely too little headroom above the
                    // IGBT/MOSFET's Vsat/Rds_on drop at this voltage for much of the commanded
                    // swing to actually reach the winding. Scale voltage using THIS attempt's own
                    // measured response as the model (ripple amplitude is linear in voltage for a
                    // fixed L), which converges in far fewer retries than a fixed step ever could.
                    // Falls back to a fixed bootstrap step only when there's no measurable signal
                    // at all yet to estimate a ratio from (e.g. the very first attempt).
                    float next_voltage = have_signal
                        ? avg_achieved_voltage * (target_ripple_amps / achieved_ripple_amps)
                        : avg_achieved_voltage * inductance_test_voltage_step_multiplier;
                    current_test_voltage = clamp(next_voltage, inductance_test_step_voltage, max_ident_test_voltage);
                    inductance_voltage_retry_count++;
                    ripple_unfolded_accum = 0.0f;
                    ripple_voltage_accum = 0.0f;
                    have_prev_raw_i = false;
                    regression.reset();
                    current_filter.reset(0.0f);
                    l_state = inductance_state::SETTLE_ZERO;
                    break;
                }

                if(have_signal) {
                    float l_henries = std::fabs(avg_achieved_voltage) / std::fabs(dI_dt);
                    bool plausible = l_henries >= inductance_absolute_min_henries && l_henries <= inductance_absolute_max_henries;
                    if(plausible) {
                        pair_results[static_cast<uint8_t>(current_pair)].l_henries = l_henries;
                        pair_results[static_cast<uint8_t>(current_pair)].inductance_valid = true;
                    } else {
                        msg->add(mode_msgs.inductance_out_of_range, 0);
                        pair_results[static_cast<uint8_t>(current_pair)].inductance_valid = false;
                    }
                } else {
                    // Signal never became trustworthy even at max_ident_test_voltage / after
                    // exhausting retries.
                    msg->add(mode_msgs.inductance_fit_failure, 0);
                    pair_results[static_cast<uint8_t>(current_pair)].inductance_valid = false;
                }
                l_state = inductance_state::PAIR_DONE;
            }
            break;
        }
        case inductance_state::PAIR_DONE:
        {
            current_filter.reset(0.0f);
            regression.reset();
            ripple_unfolded_accum = 0.0f;
            ripple_voltage_accum = 0.0f;
            have_prev_raw_i = false;
            state_cycle_counter = 0;
            // current_test_voltage deliberately carries over to the next pair, not reset to
            // inductance_test_step_voltage -- once a working voltage is found for one pair, the
            // others need very close to the same value, so there's no need to re-search from
            // scratch each time. Only the retry budget resets, in case a pair genuinely differs.
            inductance_voltage_retry_count = 0;

            phase_pair completed_pair = current_pair;
            current_pair = static_cast<phase_pair>((static_cast<uint8_t>(current_pair) + 1) % 3);
            l_state = inductance_state::SETTLE_ZERO;

            if(completed_pair == phase_pair::WU) {
                solve_inductance_from_pairs();
                current_pair = phase_pair::UV;
                stage = ident_stage::DONE;
            }
            break;
        }
    }

    return function_results::CONTINUE;
}

void pmsm_ident_mode::resolve_pair_results_from_combos() {
    static constexpr uint8_t pair_combo_plus[3]  = {0, 2, 4}; // UV+, VW+, WU+ combo indices
    static constexpr uint8_t pair_combo_minus[3] = {3, 5, 1}; // UV-, VW-, WU- combo indices

    for(uint8_t p = 0; p < 3; p++) {
        combo_result& plus = combo_results[pair_combo_plus[p]];
        combo_result& minus = combo_results[pair_combo_minus[p]];

        bool direction_mismatch = false;
        bool plus_valid = false, minus_valid = false;
        float plus_slope  = combine_slopes(plus.slope_fwd,  plus.slope_bwd,  plus.fwd_valid,  plus.bwd_valid,  plus_valid,  direction_mismatch);
        bool minus_direction_mismatch = false;
        float minus_slope = combine_slopes(minus.slope_fwd, minus.slope_bwd, minus.fwd_valid, minus.bwd_valid, minus_valid, minus_direction_mismatch);
        if(direction_mismatch || minus_direction_mismatch) {
            // The same combo's forward-sweep and backward-sweep slope estimates disagree more
            // than expected -- possible mechanical hysteresis/asymmetry a single-direction sweep
            // wouldn't have revealed.
            msg->add(mode_msgs.resistance_direction_mismatch, 0);
        }

        bool polarity_mismatch = false, r_valid = false;
        float r_avg = combine_slopes(plus_slope, minus_slope, plus_valid, minus_valid, r_valid, polarity_mismatch);
        if(polarity_mismatch) {
            msg->add(mode_msgs.resistance_slope_mismatch, 0);
        }

        bool plausible = r_valid && r_avg >= resistance_absolute_min_ohms && r_avg <= resistance_absolute_max_ohms;
        if(r_valid && !plausible) {
            msg->add(mode_msgs.resistance_out_of_range, 0);
        }
        pair_results[p].r_ohms = r_avg;
        pair_results[p].resistance_valid = plausible;
    }
}

void pmsm_ident_mode::solve_resistance_from_pairs() {
    const pair_result& uv = pair_results[static_cast<uint8_t>(phase_pair::UV)];
    const pair_result& vw = pair_results[static_cast<uint8_t>(phase_pair::VW)];
    const pair_result& wu = pair_results[static_cast<uint8_t>(phase_pair::WU)];

    if(uv.resistance_valid && vw.resistance_valid && wu.resistance_valid) {
        float r[3];
        r[0] = 0.5f * (uv.r_ohms + wu.r_ohms - vw.r_ohms); // U
        r[1] = 0.5f * (uv.r_ohms + vw.r_ohms - wu.r_ohms); // V
        r[2] = 0.5f * (vw.r_ohms + wu.r_ohms - uv.r_ohms); // W

        bool all_plausible = true;
        for(uint8_t i = 0; i < 3; i++) {
            if(r[i] < resistance_absolute_min_ohms || r[i] > resistance_absolute_max_ohms) {
                all_plausible = false;
            }
        }

        if(all_plausible) {
            measured_resistance[0] = r[0];
            measured_resistance[1] = r[1];
            measured_resistance[2] = r[2];
        } else {
            // Each pair looked plausible individually, but the solved per-phase values are
            // mutually inconsistent (can happen from noise/bias even with 3 "reasonable" pair
            // sums) -- report and leave as NaN rather than fabricate a number.
            msg->add(mode_msgs.resistance_out_of_range, 0);
            float nan_val = std::numeric_limits<float>::quiet_NaN();
            measured_resistance[0] = nan_val;
            measured_resistance[1] = nan_val;
            measured_resistance[2] = nan_val;
        }
    } else {
        // Each phase's formula needs all 3 pair sums -- if any pair failed (e.g. an open/high-
        // resistance phase), no individual phase resistance can be solved exactly. Leave a NaN
        // sentinel rather than fabricate a number; a real open phase will typically also show up
        // as a failure in the OTHER pair sharing that phase, narrowing down which physical phase
        // is at fault.
        msg->add(mode_msgs.resistance_incomplete, 0);
        float nan_val = std::numeric_limits<float>::quiet_NaN();
        measured_resistance[0] = nan_val;
        measured_resistance[1] = nan_val;
        measured_resistance[2] = nan_val;
    }
}

void pmsm_ident_mode::solve_inductance_from_pairs() {
    // IMPORTANT: unlike resistance, the per-pair L measured by run_inductance_stage() is the
    // *apparent loop inductance* Lx + Ly - 2*Mxy (mutual coupling between windings does not
    // cancel in a two-phase-energized test the way resistance does). The decomposition below
    // assumes the cross-coupling terms are small/roughly equal and folds them into the result for
    // API-shape consistency with measured_resistance[3] -- it is NOT a rigorously validated
    // per-phase self-inductance split. Treat measured_inductance[] as lower-confidence than
    // measured_resistance[] until/unless a mutual-inductance-aware identification method (which
    // needs more than 3 line-to-line tests) is implemented.
    const pair_result& uv = pair_results[static_cast<uint8_t>(phase_pair::UV)];
    const pair_result& vw = pair_results[static_cast<uint8_t>(phase_pair::VW)];
    const pair_result& wu = pair_results[static_cast<uint8_t>(phase_pair::WU)];

    if(uv.inductance_valid && vw.inductance_valid && wu.inductance_valid) {
        float l[3];
        l[0] = 0.5f * (uv.l_henries + wu.l_henries - vw.l_henries); // U
        l[1] = 0.5f * (uv.l_henries + vw.l_henries - wu.l_henries); // V
        l[2] = 0.5f * (vw.l_henries + wu.l_henries - uv.l_henries); // W

        bool all_plausible = true;
        for(uint8_t i = 0; i < 3; i++) {
            if(l[i] < inductance_absolute_min_henries || l[i] > inductance_absolute_max_henries) {
                all_plausible = false;
            }
        }

        if(all_plausible) {
            measured_inductance[0] = l[0];
            measured_inductance[1] = l[1];
            measured_inductance[2] = l[2];
        } else {
            msg->add(mode_msgs.inductance_out_of_range, 0);
            float nan_val = std::numeric_limits<float>::quiet_NaN();
            measured_inductance[0] = nan_val;
            measured_inductance[1] = nan_val;
            measured_inductance[2] = nan_val;
        }
    } else {
        msg->add(mode_msgs.inductance_incomplete, 0);
        float nan_val = std::numeric_limits<float>::quiet_NaN();
        measured_inductance[0] = nan_val;
        measured_inductance[1] = nan_val;
        measured_inductance[2] = nan_val;
    }
}
