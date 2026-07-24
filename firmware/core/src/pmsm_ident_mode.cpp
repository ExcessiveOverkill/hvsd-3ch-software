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

    align_ramp_hold_cycles           = mtr_ch->calculate_pwm_cycles_from_us(align_ramp_hold_us);
    align_mech_settle_cycles         = mtr_ch->calculate_pwm_cycles_from_us(align_mech_settle_us);
    current_ramp_settle_cycles       = mtr_ch->calculate_pwm_cycles_from_us(current_ramp_settle_us);
    sample_average_cycles            = mtr_ch->calculate_pwm_cycles_from_us(sample_average_us);
    pair_timeout_cycles              = mtr_ch->calculate_pwm_cycles_from_us(pair_timeout_us);
    ramp_to_zero_timeout_cycles      = mtr_ch->calculate_pwm_cycles_from_us(ramp_to_zero_timeout_us);
    inductance_settle_timeout_cycles = mtr_ch->calculate_pwm_cycles_from_us(inductance_settle_timeout_us);
    inductance_ramp_down_cycles      = mtr_ch->calculate_pwm_cycles_from_us(inductance_ramp_down_us);
}

void pmsm_ident_mode::user_reset() {
    stage = ident_stage::ALIGN;
    a_state = align_state::RAMP_POS;
    r_state = resistance_state::RAMP_TO_LEVEL;
    l_state = inductance_state::SETTLE_ZERO;

    current_pair = phase_pair::UV;
    level_index = 0;
    polarity = 1;
    saturation_cycle_counter = 0;
    pair_timed_out = false;
    pair_measurement_complete = false;
    r_pos_slope = 0.0f;
    sample_i_accum = 0.0f;
    sample_v_accum = 0.0f;
    sample_accum_count = 0;

    inductance_measuring_iss = true;
    inductance_pair_failed = false;
    v_step = 0.0f;
    i_ss = 0.0f;
    rise_cycle_index = 0;
    settled_count = 0;
    last_filtered_i_for_settle = 0.0f;
    iss_accum = 0.0f;
    iss_accum_count = 0;
    ramp_down_start_v = 0.0f;

    state_cycle_counter = 0;

    for(uint8_t i = 0; i < 3; i++) {
        pair_results[i] = pair_result{};
    }

    regression.reset();
    current_pid.reset();
    current_filter.reset(0.0f);

    configure_control_objects();
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
        case ident_stage::ALIGN:      return run_align_stage();
        case ident_stage::RESISTANCE: return run_resistance_stage();
        case ident_stage::INDUCTANCE: return run_inductance_stage();
        case ident_stage::DONE:       return function_results::COMPLETE;
    }
    return function_results::ERROR; // unreachable, silences -Wreturn-type
}

pmsm_ident_mode::phase_pair pmsm_ident_mode::next_pair(phase_pair p) {
    switch(p) {
        case phase_pair::UV: return phase_pair::VW;
        case phase_pair::VW: return phase_pair::WU;
        case phase_pair::WU: return phase_pair::UV;
    }
    return phase_pair::UV; // unreachable, silences -Wreturn-type
}

bool pmsm_ident_mode::command_pair_voltage(phase_pair pair, float v_diff) {
    const pair_phases& pf = pair_phase_table[static_cast<uint8_t>(pair)];
    float v[3] = {0.0f, 0.0f, 0.0f};
    v[pf.plus_idx]  =  0.5f * v_diff;
    v[pf.minus_idx] = -0.5f * v_diff;
    v[pf.quiet_idx] = 0.0f;
    return mtr_ch->set_phase_voltage(v[0], v[1], v[2]);
}

void pmsm_ident_mode::read_pair_currents(phase_pair pair, float& loop_current_out, float& quiet_current_out) {
    const pair_phases& pf = pair_phase_table[static_cast<uint8_t>(pair)];
    float iu, iv, iw;
    mtr_ch->get_phase_currents(iu, iv, iw);
    float i[3] = {iu, iv, iw};
    loop_current_out = 0.5f * (i[pf.plus_idx] - i[pf.minus_idx]);
    quiet_current_out = i[pf.quiet_idx];
}

mode::function_results pmsm_ident_mode::run_align_stage() {

    // This stage always runs briefly (if either measurement is enabled) to settle a possibly
    // free-spinning rotor before any real test, regardless of which measurement(s) are enabled --
    // harmless if the rotor was already stationary/locked.
    if(!enable_resistance_measurement && !enable_inductance_measurement) {
        stage = ident_stage::DONE;
        return function_results::CONTINUE;
    }

    float align_target = 0.5f * motor_calibration_current;
    float loop_i = 0.0f, quiet_i = 0.0f;
    read_pair_currents(phase_pair::UV, loop_i, quiet_i);
    float filtered_i = current_filter.update(loop_i);

    switch(a_state) {
        case align_state::RAMP_POS:
        {
            float v_cmd = current_pid.update(align_target, filtered_i);
            command_pair_voltage(phase_pair::UV, v_cmd);

            if(std::fabs(v_cmd) >= (max_ident_test_voltage - 0.01f)) {
                saturation_cycle_counter++;
            } else {
                saturation_cycle_counter = 0;
            }

            if(std::fabs(filtered_i - align_target) < reach_target_tolerance_amps) {
                saturation_cycle_counter = 0;
                state_cycle_counter = 0;
                a_state = align_state::HOLD_POS;
            } else if(saturation_cycle_counter >= pair_timeout_cycles) {
                // Can't reach alignment current -- likely a disconnected motor or open circuit.
                // Report and skip straight to DONE (-> resistance) rather than hanging here;
                // the resistance stage's own per-pair timeout will independently detect (and more
                // precisely localize) the same underlying problem.
                msg->add(mode_msgs.align_current_not_reached, 0);
                saturation_cycle_counter = 0;
                state_cycle_counter = 0;
                a_state = align_state::DONE;
            }
            break;
        }
        case align_state::HOLD_POS:
        {
            float v_cmd = current_pid.update(align_target, filtered_i);
            command_pair_voltage(phase_pair::UV, v_cmd);
            state_cycle_counter++;
            if(state_cycle_counter >= align_ramp_hold_cycles) {
                state_cycle_counter = 0;
                a_state = align_state::RAMP_NEG;
            }
            break;
        }
        case align_state::RAMP_NEG:
        {
            float v_cmd = current_pid.update(-align_target, filtered_i);
            command_pair_voltage(phase_pair::UV, v_cmd);

            if(std::fabs(v_cmd) >= (max_ident_test_voltage - 0.01f)) {
                saturation_cycle_counter++;
            } else {
                saturation_cycle_counter = 0;
            }

            if(std::fabs(filtered_i + align_target) < reach_target_tolerance_amps) {
                saturation_cycle_counter = 0;
                state_cycle_counter = 0;
                a_state = align_state::HOLD_NEG;
            } else if(saturation_cycle_counter >= pair_timeout_cycles) {
                msg->add(mode_msgs.align_current_not_reached, 0);
                saturation_cycle_counter = 0;
                state_cycle_counter = 0;
                a_state = align_state::DONE;
            }
            break;
        }
        case align_state::HOLD_NEG:
        {
            float v_cmd = current_pid.update(-align_target, filtered_i);
            command_pair_voltage(phase_pair::UV, v_cmd);
            state_cycle_counter++;
            if(state_cycle_counter >= align_ramp_hold_cycles) {
                state_cycle_counter = 0;
                a_state = align_state::RAMP_ZERO;
            }
            break;
        }
        case align_state::RAMP_ZERO:
        {
            float v_cmd = current_pid.update(0.0f, filtered_i);
            command_pair_voltage(phase_pair::UV, v_cmd);
            state_cycle_counter++;
            bool reached_zero = std::fabs(filtered_i) < zero_current_tolerance_amps;
            bool timed_out = state_cycle_counter >= ramp_to_zero_timeout_cycles;
            if(reached_zero || timed_out) {
                if(timed_out && !reached_zero) {
                    // Current isn't decaying back to zero -- possible inverter/sensing fault.
                    // Not fatal on its own; proceed to MECH_SETTLE anyway rather than hang.
                    msg->add(mode_msgs.align_settle_timeout, 0);
                }
                state_cycle_counter = 0;
                a_state = align_state::MECH_SETTLE;
            }
            break;
        }
        case align_state::MECH_SETTLE:
        {
            command_pair_voltage(phase_pair::UV, 0.0f);
            state_cycle_counter++;
            if(state_cycle_counter >= align_mech_settle_cycles) {
                a_state = align_state::DONE;
            }
            break;
        }
        case align_state::DONE:
        {
            current_pid.reset();
            current_filter.reset(0.0f);
            state_cycle_counter = 0;
            saturation_cycle_counter = 0;
            // Resistance is always run next, even if only inductance was requested -- inductance
            // needs a valid R to convert its fitted time constant into henries, so it's a hidden
            // dependency. The resulting measured_resistance[] is populated either way.
            stage = ident_stage::RESISTANCE;
            return function_results::CONTINUE;
        }
    }

    return function_results::CONTINUE;
}

mode::function_results pmsm_ident_mode::run_resistance_stage() {

    float loop_i = 0.0f, quiet_i = 0.0f;
    read_pair_currents(current_pair, loop_i, quiet_i);
    float filtered_i = current_filter.update(loop_i);

    float target_current = static_cast<float>(polarity) * test_level_fractions[level_index] * motor_calibration_current;

    switch(r_state) {
        case resistance_state::RAMP_TO_LEVEL:
        {
            float v_cmd = current_pid.update(target_current, filtered_i);
            command_pair_voltage(current_pair, v_cmd);

            // Quiet-phase imbalance diagnostic -- a healthy, balanced motor carries ~0 current on
            // the undriven phase during a pair test.
            if(std::fabs(loop_i) > imbalance_min_loop_current &&
               std::fabs(quiet_i) > imbalance_fraction_threshold * std::fabs(loop_i)) {
                // report phase-imbalance anomaly (pair = current_pair, quiet_i, loop_i)
                msg->add(mode_msgs.resistance_phase_sense_imbalance, 0);
            }

            if(std::fabs(v_cmd) >= (max_ident_test_voltage - 0.01f)) {
                saturation_cycle_counter++;
            } else {
                saturation_cycle_counter = 0;
            }

            if(std::fabs(filtered_i - target_current) < reach_target_tolerance_amps) {
                saturation_cycle_counter = 0;
                state_cycle_counter = 0;
                r_state = resistance_state::SETTLE;
            } else if(saturation_cycle_counter >= pair_timeout_cycles) {
                // report open/high-resistance phase anomaly for current_pair -- note a
                // single truly-open phase should also fail the OTHER pair containing it (each
                // phase appears in exactly 2 of the 3 pairs), a strong diagnostic for which
                // physical phase is at fault even without automated inference here.
                msg->add(mode_msgs.resistance_too_high, 0);
                pair_timed_out = true;
                state_cycle_counter = 0;
                r_state = resistance_state::RAMP_TO_ZERO;
            }
            break;
        }
        case resistance_state::SETTLE:
        {
            float v_cmd = current_pid.update(target_current, filtered_i);
            command_pair_voltage(current_pair, v_cmd);
            state_cycle_counter++;
            if(state_cycle_counter >= current_ramp_settle_cycles) {
                state_cycle_counter = 0;
                sample_i_accum = 0.0f;
                sample_v_accum = 0.0f;
                sample_accum_count = 0;
                r_state = resistance_state::SAMPLE;
            }
            break;
        }
        case resistance_state::SAMPLE:
        {
            float v_cmd = current_pid.update(target_current, filtered_i);
            command_pair_voltage(current_pair, v_cmd);
            sample_i_accum += filtered_i;
            sample_v_accum += v_cmd;
            sample_accum_count++;
            state_cycle_counter++;
            if(state_cycle_counter >= sample_average_cycles) {
                float avg_i = sample_i_accum / static_cast<float>(sample_accum_count);
                float avg_v = sample_v_accum / static_cast<float>(sample_accum_count);
                regression.add_sample(avg_i, avg_v); // V = R*I + V_offset -- fit slope = R
                state_cycle_counter = 0;
                r_state = resistance_state::NEXT_LEVEL_OR_POLARITY;
            }
            break;
        }
        case resistance_state::NEXT_LEVEL_OR_POLARITY:
        {
            level_index++;
            if(level_index < num_test_levels) {
                r_state = resistance_state::RAMP_TO_ZERO;
            } else {
                level_index = 0;
                if(polarity > 0) {
                    r_pos_slope = regression.valid() ? regression.slope() : 0.0f;
                    regression.reset();
                    polarity = -1;
                    r_state = resistance_state::RAMP_TO_ZERO;
                } else {
                    float r_neg_slope = regression.valid() ? regression.slope() : 0.0f;
                    // report diagnostic if r_pos_slope and r_neg_slope disagree significantly
                    msg->add(mode_msgs.resistance_slope_mismatch, 0);
                    pair_results[static_cast<uint8_t>(current_pair)].r_ohms = 0.5f * (r_pos_slope + r_neg_slope);
                    pair_results[static_cast<uint8_t>(current_pair)].resistance_valid = !pair_timed_out;
                    polarity = 1;
                    regression.reset();
                    pair_measurement_complete = true;
                    r_state = resistance_state::RAMP_TO_ZERO;
                }
            }
            break;
        }
        case resistance_state::RAMP_TO_ZERO:
        {
            float v_cmd = current_pid.update(0.0f, filtered_i);
            command_pair_voltage(current_pair, v_cmd);
            state_cycle_counter++;
            bool reached_zero = std::fabs(filtered_i) < zero_current_tolerance_amps;
            bool timed_out = state_cycle_counter >= ramp_to_zero_timeout_cycles;
            if(reached_zero || timed_out) {
                state_cycle_counter = 0;
                if(pair_timed_out || pair_measurement_complete) {
                    r_state = resistance_state::PAIR_DONE;
                } else {
                    r_state = resistance_state::RAMP_TO_LEVEL;
                }
            }
            break;
        }
        case resistance_state::PAIR_DONE:
        {
            current_pid.reset();
            current_filter.reset(0.0f);
            regression.reset();
            level_index = 0;
            polarity = 1;
            pair_timed_out = false;
            pair_measurement_complete = false;
            saturation_cycle_counter = 0;
            state_cycle_counter = 0;

            phase_pair completed_pair = current_pair;
            current_pair = next_pair(current_pair);
            r_state = resistance_state::RAMP_TO_LEVEL;

            if(completed_pair == phase_pair::WU) {
                // all 3 pairs done
                solve_resistance_from_pairs();
                current_pair = phase_pair::UV;
                stage = enable_inductance_measurement ? ident_stage::INDUCTANCE : ident_stage::DONE;
            }
            break;
        }
    }

    return function_results::CONTINUE;
}

mode::function_results pmsm_ident_mode::run_inductance_stage() {

    float loop_i = 0.0f, quiet_i = 0.0f;
    read_pair_currents(current_pair, loop_i, quiet_i);
    float filtered_i = current_filter.update(loop_i);

    switch(l_state) {
        case inductance_state::SETTLE_ZERO:
        {
            command_pair_voltage(current_pair, 0.0f);
            state_cycle_counter++;
            bool settled = std::fabs(filtered_i) < zero_current_tolerance_amps;
            bool timed_out = state_cycle_counter >= ramp_to_zero_timeout_cycles;
            if((settled && state_cycle_counter >= current_ramp_settle_cycles) || timed_out) {
                // report if timed_out and !settled -- current didn't fully decay before
                // starting the inductance step, reducing confidence in Iss/tau below
                msg->add(mode_msgs.current_decay_timeout, 0);

                state_cycle_counter = 0;

                const pair_result& pr = pair_results[static_cast<uint8_t>(current_pair)];
                if(pr.resistance_valid && pr.r_ohms > 1e-4f) {
                    float target_i = inductance_test_current_fraction * motor_calibration_current;
                    v_step = clamp(target_i * pr.r_ohms, -max_ident_test_voltage, max_ident_test_voltage);
                } else {
                    v_step = inductance_fallback_step_voltage;
                    // report reduced-confidence inductance measurement for current_pair (no
                    // valid resistance available to size the step or later convert tau -> L)
                    msg->add(mode_msgs.inductance_no_valid_resistance, 0);
                }

                rise_cycle_index = 0;
                settled_count = 0;
                iss_accum = 0.0f;
                iss_accum_count = 0;
                last_filtered_i_for_settle = filtered_i;
                regression.reset();
                l_state = inductance_state::STEP_RISE;
            }
            break;
        }
        case inductance_state::STEP_RISE:
        {
            command_pair_voltage(current_pair, v_step);

            if(inductance_measuring_iss) {
                // Pass 1: wait for settling, then average into i_ss. Measuring this directly
                // (rather than computing it from v_step/R) is what makes the tau fit below
                // insensitive to the deadtime/Vsat-type voltage error -- that error only changes
                // what i_ss turns out to be, not the shape of the exponential rise.
                float delta = std::fabs(filtered_i - last_filtered_i_for_settle);
                last_filtered_i_for_settle = filtered_i;

                if(delta < inductance_settle_delta_threshold_amps) {
                    settled_count++;
                } else {
                    settled_count = 0;
                }

                rise_cycle_index++;

                if(settled_count >= inductance_settle_consecutive_samples) {
                    iss_accum += filtered_i;
                    iss_accum_count++;
                    if(iss_accum_count >= inductance_settle_consecutive_samples) {
                        i_ss = iss_accum / static_cast<float>(iss_accum_count);
                        ramp_down_start_v = v_step;
                        state_cycle_counter = 0;
                        l_state = inductance_state::RAMP_DOWN;
                    }
                } else if(rise_cycle_index >= inductance_settle_timeout_cycles) {
                    // report inductance pass-1 settle timeout for current_pair
                    msg->add(mode_msgs.inductance_settle_timeout, 0);
                    inductance_pair_failed = true;
                    ramp_down_start_v = v_step;
                    state_cycle_counter = 0;
                    l_state = inductance_state::RAMP_DOWN;
                }
            } else {
                // Pass 2: log-linear regression of the current rise (Iss already known from pass 1).
                if(rise_cycle_index >= inductance_skip_cycles) {
                    float residual = i_ss - filtered_i;
                    if(residual > i_ss * inductance_log_cutoff_frac && residual > 0.0f) {
                        regression.add_sample(static_cast<float>(rise_cycle_index), std::log(residual));
                    } else {
                        // Reached the log-domain cutoff (or noise pushed residual <= 0) -- stop
                        // sampling and move on regardless of exact cycle count.
                        ramp_down_start_v = v_step;
                        state_cycle_counter = 0;
                        l_state = inductance_state::RAMP_DOWN;
                        break;
                    }
                }
                rise_cycle_index++;

                if(rise_cycle_index >= inductance_settle_timeout_cycles) {
                    // Absolute timeout safety net in case the cutoff condition above is never hit.
                    ramp_down_start_v = v_step;
                    state_cycle_counter = 0;
                    l_state = inductance_state::RAMP_DOWN;
                }
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

            if(state_cycle_counter >= inductance_ramp_down_cycles) {
                state_cycle_counter = 0;

                if(inductance_measuring_iss) {
                    if(inductance_pair_failed) {
                        l_state = inductance_state::PAIR_DONE;
                    } else {
                        inductance_measuring_iss = false;
                        l_state = inductance_state::SETTLE_ZERO;
                    }
                } else {
                    const pair_result& pr = pair_results[static_cast<uint8_t>(current_pair)];
                    if(regression.valid() && regression.slope() < -inductance_min_slope_magnitude && pr.resistance_valid) {
                        float tau_seconds = -1.0f / (regression.slope() * pwm_frequency_hz);
                        pair_results[static_cast<uint8_t>(current_pair)].l_henries = pr.r_ohms * tau_seconds;
                        pair_results[static_cast<uint8_t>(current_pair)].inductance_valid = true;
                    } else {
                        // report inductance fit failure/low-confidence for current_pair
                        msg->add(mode_msgs.inductance_fit_failure, 0);
                        pair_results[static_cast<uint8_t>(current_pair)].inductance_valid = false;
                    }
                    l_state = inductance_state::PAIR_DONE;
                }
            }
            break;
        }
        case inductance_state::PAIR_DONE:
        {
            current_filter.reset(0.0f);
            regression.reset();
            inductance_measuring_iss = true;
            inductance_pair_failed = false;
            rise_cycle_index = 0;
            settled_count = 0;
            last_filtered_i_for_settle = 0.0f;
            iss_accum = 0.0f;
            iss_accum_count = 0;
            state_cycle_counter = 0;

            phase_pair completed_pair = current_pair;
            current_pair = next_pair(current_pair);
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

void pmsm_ident_mode::solve_resistance_from_pairs() {
    const pair_result& uv = pair_results[static_cast<uint8_t>(phase_pair::UV)];
    const pair_result& vw = pair_results[static_cast<uint8_t>(phase_pair::VW)];
    const pair_result& wu = pair_results[static_cast<uint8_t>(phase_pair::WU)];

    if(uv.resistance_valid && vw.resistance_valid && wu.resistance_valid) {
        measured_resistance[0] = 0.5f * (uv.r_ohms + wu.r_ohms - vw.r_ohms); // U
        measured_resistance[1] = 0.5f * (uv.r_ohms + vw.r_ohms - wu.r_ohms); // V
        measured_resistance[2] = 0.5f * (vw.r_ohms + wu.r_ohms - uv.r_ohms); // W
    } else {
        // Each phase's formula needs all 3 pair sums -- if any pair failed (e.g. an open/high-
        // resistance phase), no individual phase resistance can be solved exactly. Leave a NaN
        // sentinel rather than fabricate a number; a real open phase will typically also show up
        // as a failure in the OTHER pair sharing that phase, narrowing down which physical phase
        // is at fault (see the anomaly comments in run_resistance_stage()).
        // report resistance identification incomplete/degraded
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
        measured_inductance[0] = 0.5f * (uv.l_henries + wu.l_henries - vw.l_henries); // U
        measured_inductance[1] = 0.5f * (uv.l_henries + vw.l_henries - wu.l_henries); // V
        measured_inductance[2] = 0.5f * (vw.l_henries + wu.l_henries - uv.l_henries); // W
    } else {
        // report inductance identification incomplete/degraded
        msg->add(mode_msgs.inductance_incomplete, 0);
        float nan_val = std::numeric_limits<float>::quiet_NaN();
        measured_inductance[0] = nan_val;
        measured_inductance[1] = nan_val;
        measured_inductance[2] = nan_val;
    }
}
