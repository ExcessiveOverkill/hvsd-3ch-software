#pragma once

#include "mode.h"

/*
Identify PMSM parameters for an unknown motor.
*/

class pmsm_ident_mode : public mode {
    public:
        pmsm_ident_mode(motor_channel* mtr_ch_, Messaging* msg_, time_interface* time_);

    private:

        _Msg_Motor_Mode_PmsmIdent mode_msgs;

        bool enable_resistance_measurement = false;
        bool enable_inductance_measurement = true;
        bool enable_commutation_offset_measurement = false;
        bool enable_back_emf_measurement = false;

        // NOTE: bench testing (2026-08) shows measured_resistance[] runs ~0.8ohm/phase vs ~0.65
        // ohm/phase (1.3ohm line-to-line / 2) measured externally straight across the motor
        // leads -- the gap is the drive's own two-switch-in-series conduction drop (IGBT/MOSFET
        // Vsat/Rds_on), confirmed not a current-sense calibration error (verified by injecting a
        // known current through the shunts directly). basic_deadtime_compensation only corrects
        // the deadtime blanking loss, not this ongoing conduction-resistance term, and a linear
        // V/I regression fundamentally can't separate an ohmic switch drop from true winding R.
        // Left as-is deliberately: current control's feed-forward voltage needs "resistance as
        // seen from the PID's command", which is exactly this (motor + switches) number, not the
        // motor-only value -- so this is the right quantity for that purpose, not a bug to fix.
        // If the motor-only value is ever needed, either calibrate the drive's own series
        // resistance via a short-circuit/known-reference-load run of this same routine and
        // subtract it, or characterize the switches' Vsat/Rds_on from their datasheet (see the
        // use_advanced_deadtime_compensation TODO in motor_channel.cpp).
        float measured_resistance[3] = {0.0f, 0.0f, 0.0f};  // ohms
        float measured_inductance[3] = {0.0f, 0.0f, 0.0f};  // henries
        float measured_commutation_offset = 0.0f;   // radians
        float measured_back_emf = 0.0f; // v / 1000 rpm

        float motor_calibration_current = 4.0f; // amps

        void user_reset() override;

        // function_results user_start_loop() override;
        // function_results user_start_flagged_timer_update() override;
        // function_results user_start_flagged_all_adc_eoc() override;
        // function_results user_start_flagged_systick() override;

        function_results user_run_loop() override;
        function_results user_run_flagged_timer_update() override;
        function_results user_run_flagged_all_adc_eoc() override;
        function_results user_run_flagged_systick() override;

        // function_results user_soft_stop_loop() override;
        // function_results user_soft_stop_flagged_timer_update() override;
        // function_results user_soft_stop_flagged_all_adc_eoc() override;
        // function_results user_soft_stop_flagged_systick() override;

        // ================================================================
        // Resistance / inductance identification.
        //
        // No commutation encoder is available yet, so only fixed phase-pair-axis excitation is
        // possible (no rotating stator flux vector) -- commutation offset and back-EMF
        // measurement stay unimplemented for now. Everything below runs on the all_adc_eoc tick
        // (once per PWM period, right after phase current sampling), per user_run_flagged_all_adc_eoc().
        // ================================================================

        // ---- phase-pair addressing. A pair is excited as +E on one phase, -E on another, with
        // the third ("quiet") phase held at the common reference (0) -- see command_pair_voltage(). ----
        enum class phase_pair : uint8_t { UV = 0, VW = 1, WU = 2 };
        struct pair_phases { uint8_t plus_idx; uint8_t minus_idx; uint8_t quiet_idx; }; // 0=U, 1=V, 2=W
        static constexpr pair_phases pair_phase_table[3] = {
            {0, 1, 2}, // UV: +U -V, quiet W
            {1, 2, 0}, // VW: +V -W, quiet U
            {2, 0, 1}, // WU: +W -U, quiet V
        };

        // ---- top-level stage sequencing ----
        enum class ident_stage : uint8_t { RESISTANCE, INDUCTANCE, DONE };
        ident_stage stage = ident_stage::RESISTANCE;

        // ---- per-pair result storage. r_ohms/l_henries are only meaningful when the
        // corresponding *_valid flag is true -- a pair that times out (open/high-resistance
        // phase, or a failed inductance fit) is left invalid rather than fabricating a value. ----
        struct pair_result {
            float r_ohms = 0.0f;      // Ru+Rv for the UV pair, etc -- exact sum, no cross-coupling
            float l_henries = 0.0f;   // apparent Lu+Lv-2*Muv for the UV pair -- see solve_inductance_from_pairs()
            bool resistance_valid = false;
            bool inductance_valid = false;
        };
        pair_result pair_results[3];

        // ---- tunable constants -- all conservative placeholders, marked TODO where they MUST
        // be bench-tuned against real motor/PID behavior before this mode is used on hardware.
        // Declared before the control objects below since some of their constructor arguments
        // (max_ident_test_voltage) depend on these being already default-initialized. ----
        static constexpr float pid_kp = 1.0f;   // TODO: bench tune against actual R/L
        static constexpr float pid_ki = 50.0f;  // TODO: bench tune
        static constexpr float pid_kd = 0.0f;   // derivative-on-measurement rarely needed for a resistive/RL plant

        float max_ident_test_voltage = 24.0f; // volts, differential -- TODO: bench tune vs VBUS/motor rating

        static constexpr float current_filter_cutoff_hz = 200.0f; // TODO: bench tune vs PWM freq / noise floor

        static constexpr float test_level_fractions[4] = {0.25f, 0.5f, 0.75f, 1.0f};
        static constexpr uint8_t num_test_levels = 4;

        static constexpr uint32_t step_settle_min_dwell_us = 1000e3;  // minimum dwell before the settle-confirmation window below even starts counting, on a new hexagon step
        static constexpr uint32_t step_settle_timeout_us   = 3000e3;  // safety cap if a new step's current/motion never confirms settled -- reported, not fatal
        static constexpr uint32_t current_ramp_settle_us   = 100e3;   // fixed dwell after a same-step (lower-magnitude) level change, before sampling
        static constexpr uint32_t sample_average_us        = 20e3;   // averaging window per regression sample
        static constexpr uint32_t step_timeout_us          = 1000e3; // max time saturated-without-reaching-target before a step is abandoned as invalid
        static constexpr uint32_t inductance_ramp_down_us  = 50e3;
        static constexpr uint32_t inductance_settle_zero_timeout_us = 300e3; // inductance's own pre-ripple current-decay wait
        static constexpr uint32_t inductance_ripple_accum_us = 1000e3; // total ripple-accumulation window per pair, TODO: bench tune (ODrive uses ~1.25s at 1kHz as a reference point)

        static constexpr float reach_target_tolerance_amps = 0.05f;  // TODO: bench tune vs current_filter noise
        static constexpr float zero_current_tolerance_amps = 0.05f;
        static constexpr float imbalance_fraction_threshold = 0.10f; // quiet-phase current vs loop current, fraction
        static constexpr float slope_mismatch_fraction_threshold = 0.20f; // relative disagreement between two slope estimates of the same quantity, TODO: bench tune
        static constexpr float imbalance_min_loop_current = 0.5f;    // amps; ignore imbalance check below this loop current (noise floor)
        static constexpr uint8_t step_settle_consecutive_samples = 50; // consecutive cycles the filtered current must stay at the new step's target before it's trusted as mechanically settled, not just "time has passed"

        static constexpr float resistance_absolute_min_ohms = 0.001f; // sanity bound on solved R -- TODO: bench tune vs expected motor class
        static constexpr float resistance_absolute_max_ohms = 100.0f;

        // The test voltage's sign alternates at this fixed frequency (a full period = one + half
        // and one - half). Originally this was specified as N=1 PWM cycles per half (alternate
        // every single cycle), which keeps each half-cycle short relative to the pair's L/R time
        // constant -- but it also means EVERY sample sits right at a sign transition, so
        // RIPPLE_ACCUM's prev_command_sign/delta pairing has zero margin for error: if the actual
        // ADC-vs-PWM pipeline delay is off by even one cycle from what read_pair_currents()'s
        // comment assumes, the sign pairing is wrong on effectively every sample, causing
        // near-total cancellation in ripple_unfolded_accum rather than just added noise -- which
        // reads as a falsely tiny dI/dt, i.e. a falsely huge, inconsistent measured L (bench
        // testing 2026-08 saw exactly this: ~50-90mH, wildly high and uneven between pairs, with
        // current-sense and deadtime-compensation both ruled out as the cause). A fixed, much
        // lower frequency means only the samples right at a transition edge are at risk from a
        // delay-assumption error; most samples sit mid-plateau where the pairing is correct
        // regardless of the exact delay. Bench testing found 500-1000Hz worked well.
        static constexpr float inductance_ripple_test_frequency_hz = 500.0f; // TODO: bench tune within the 500-1000Hz range

        // Starting test voltage -- unlike a step-response method this needs no prior R to size
        // it, so any reasonable starting point works. Escalated if the resulting ripple current
        // is too small relative to inductance_ripple_target_current_fraction*motor_calibration_
        // current, e.g. because a low test voltage leaves too little headroom above the
        // IGBT/MOSFET's Vsat/Rds_on drop for much of the commanded swing to actually reach the
        // winding. Only the first pair (UV) searches from this starting point -- once a working
        // voltage is found it carries over to the remaining pairs (see PAIR_DONE), since they'll
        // need very close to the same value.
        static constexpr float inductance_test_step_voltage = 6.0f; // TODO: bench tune starting point
        static constexpr float inductance_ripple_target_current_fraction = 0.1f; // target ripple peak amplitude, as a fraction of motor_calibration_current -- TODO: bench tune
        static constexpr float inductance_ripple_accept_fraction_of_target = 0.5f; // accept without further retries once achieved ripple reaches this fraction of the target
        static constexpr float inductance_test_voltage_step_multiplier = 1.5f; // fallback bootstrap step, used only when an attempt yields no measurable signal at all to estimate a proper ratio from
        static constexpr uint8_t inductance_test_voltage_max_retries = 3;
        static constexpr float inductance_ripple_min_dIdt_amps_per_sec = 1.0f; // floor for "is there any measurable signal at all" -- below this, a next-voltage ratio can't be estimated, so fall back to the bootstrap step -- TODO: bench tune
        static constexpr float inductance_absolute_min_henries = 1e-6f; // sanity bound on solved L -- TODO: bench tune vs expected motor class
        static constexpr float inductance_absolute_max_henries = 1.0f;

        // cached cycle-count versions of the *_us constants above, computed once in configure_control_objects()
        uint32_t step_settle_min_dwell_cycles = 0;
        uint32_t step_settle_timeout_cycles = 0;
        uint32_t current_ramp_settle_cycles = 0;
        uint32_t sample_average_cycles = 0;
        uint32_t step_timeout_cycles = 0;
        uint32_t inductance_ramp_down_cycles = 0;
        uint32_t inductance_settle_zero_timeout_cycles = 0;
        uint32_t inductance_ripple_accum_cycles = 0;
        uint32_t inductance_ripple_alternation_cycles = 1; // derived from inductance_ripple_test_frequency_hz, not a fixed cycle count -- see its comment above

        // ---- shared control/fit objects, reused across pairs/stages (value objects, no dynamic
        // allocation). Constructed with safe placeholder dt/sample-rate in the ctor init list
        // (the real PWM frequency isn't configured yet at construction time -- these objects are
        // static-storage-duration and get built before device::init() runs), then reconfigured
        // with the real values in configure_control_objects(), called from user_reset(). ----
        pid_controller current_pid;
        low_pass_filter current_filter;
        linear_regression regression;
        float pwm_frequency_hz = 0.0f;
        float pwm_dt_seconds = 0.0f;
        void configure_control_objects();

        // generic "cycles spent in the current sub-state" counter, reused (reset to 0 on every
        // sub-state transition) across all stages/sub-machines below
        uint32_t state_cycle_counter = 0;
        uint32_t saturation_cycle_counter = 0; // consecutive cycles at/near the output clamp -- drives open-phase/timeout detection
        uint32_t stable_at_target_count = 0;   // consecutive cycles the filtered current has stayed at the current step's target -- see step_settle_consecutive_samples

        // ================= RESISTANCE: 6-step "hexagon" excitation sequence =================
        //
        // Rather than testing each of the 3 phase pairs at both polarities in place (which
        // requires reversing the same pair's current 180 degrees -- an unstable equilibrium
        // transition: the rotor ends up settled exactly opposite the new target, so the torque
        // pulling it to the new alignment is near zero and its direction is decided by noise,
        // producing the slow/unpredictable rotation this replaces), the six (pair, sign)
        // combinations are walked in the same order a six-step BLDC commutation table would use
        // -- each 60 degrees from the last -- so every transition, including what used to be a
        // "polarity flip", is a real, predictable, single-direction torque step.
        //
        // hex_sequence[k], k=0..5, are each 60 degrees apart in that order (verified via Clarke
        // transform): UV+ -30deg, WU- 30deg, VW+ 90deg, UV- 150deg, WU+ 210deg, VW- 270deg.
        struct hex_combo { phase_pair pair; int8_t sign; };
        static constexpr hex_combo hex_sequence[6] = {
            {phase_pair::UV,  1}, {phase_pair::WU, -1}, {phase_pair::VW,  1},
            {phase_pair::UV, -1}, {phase_pair::WU,  1}, {phase_pair::VW, -1},
        };

        // The sequence runs 1.5 hexagon cycles forward (9 steps), then 1.5 cycles backward (9
        // steps), 18 steps total. Only the *later* full cycle of each direction (the last 6 of
        // each 9-step half) is used for measurement -- the first 3 steps of each half are a
        // deliberately-discarded lead-in that walks the rotor onto the sequence's track, so even
        // if it happened to start the whole test sitting exactly on an unstable balance point,
        // that ambiguity is resolved before any data is trusted. Ending 1.5 forward + 1.5 backward
        // nets zero rotor displacement over the whole run -- important if the shaft is attached to
        // mechanics with limited travel. Testing every combo once going forward and again going
        // backward also gives two independent slope estimates per combo, letting
        // resistance_direction_mismatch flag hysteresis/asymmetry a single-direction sweep can't see.
        static constexpr uint8_t hex_combo_count = 6;
        static constexpr uint8_t steps_per_half = 9; // 1.5 hexagon cycles
        static constexpr uint8_t total_steps = 2 * steps_per_half;
        uint8_t step_number = 0; // 0..total_steps-1
        uint8_t hex_cursor = 0;  // index into hex_sequence for the current step

        bool step_is_forward() const { return step_number < steps_per_half; }
        // The last hex_combo_count (one full cycle) steps of each half are recorded; the first
        // steps_per_half-hex_combo_count are a discarded lead-in (see comment above).
        bool step_is_recording() const { return (step_number % steps_per_half) >= (steps_per_half - hex_combo_count); }
        const hex_combo& current_combo() const { return hex_sequence[hex_cursor]; }
        // Moves hex_cursor by one hexagon step in the direction appropriate for `step_number`
        // (forward => +60 degrees, backward => -60 degrees), called when advancing to a new step.
        void advance_hex_cursor() { hex_cursor = step_is_forward() ? (hex_cursor + 1) % hex_combo_count : (hex_cursor + hex_combo_count - 1) % hex_combo_count; }

        // Per-combo slope estimates, kept separate by sweep direction (rather than pooled) so a
        // forward/backward mismatch actually shows up instead of being averaged away.
        struct combo_result { float slope_fwd = 0.0f; float slope_bwd = 0.0f; bool fwd_valid = false; bool bwd_valid = false; };
        combo_result combo_results[6];

        enum class resistance_state : uint8_t { RAMP_TO_LEVEL, SETTLE, SAMPLE, ADVANCE };
        resistance_state r_state = resistance_state::RAMP_TO_LEVEL;
        uint8_t levels_remaining = 0; // counts down num_test_levels..1 within a step -- test_level_fractions[levels_remaining-1] is always the level under test, so counting down tests max-to-min directly
        bool step_is_new_position = true; // true only for a step's first (max) level -- that's the one that just moved 60 degrees and needs a confirmed settle, not just a fixed dwell
        float sample_i_accum = 0.0f;
        float sample_v_accum = 0.0f;
        float sample_quiet_i_accum = 0.0f;
        uint32_t sample_accum_count = 0;

        // ---- INDUCTANCE sub-state: symmetric bipolar square-wave ripple-current method
        // (ODrive-style). Alternates the commanded pair voltage's sign every
        // inductance_ripple_alternation_cycles, accumulating an "unfolded ripple" signal fed into
        // `regression` (x=cycle index) so its slope gives dI/dt directly -- no dependency on
        // r_ohms at all, so inductance and resistance are order-independent. Needs no mechanical
        // settle/realign of its own: for a PMSM the magnet-alignment torque term is linear in the
        // injected current vector, so it cancels exactly over one full alternation period
        // regardless of which pair is driven -- unlike resistance, no hexagon walk is needed here. ----
        enum class inductance_state : uint8_t { SETTLE_ZERO, RIPPLE_ACCUM, RAMP_DOWN, PAIR_DONE };
        inductance_state l_state = inductance_state::SETTLE_ZERO;
        phase_pair current_pair = phase_pair::UV;
        float ripple_unfolded_accum = 0.0f; // running sum of prev_command_sign * (raw current delta), fed to regression each cycle
        float prev_raw_loop_i = 0.0f;
        int8_t prev_command_sign = 1;
        bool have_prev_raw_i = false;
        float ramp_down_start_v = 0.0f; // last commanded ripple voltage, for linear interpolation to 0
        float current_test_voltage = inductance_test_step_voltage; // this pair's working test voltage -- escalated on a too-small dI/dt, reset to inductance_test_step_voltage at the start of each pair
        uint8_t inductance_voltage_retry_count = 0;
        float ripple_voltage_accum = 0.0f; // running sum of |actual achieved ripple voltage|, averaged over the accumulation window and used in place of current_test_voltage in L=V/dI_dt and the next-voltage estimate -- see command_pair_voltage()'s comment on why the nominal command can't be trusted directly

        // ================= helper methods =================

        // Commands a differential test voltage across `pair`'s +/- phases, with the third
        // ("quiet") phase held at the common reference (0). set_phase_voltage's internal
        // common-mode SVPWM offset injection preserves these relative differences exactly (a
        // provable, not approximate, property for a floating-star-point wye motor) -- this is
        // why this mode uses set_phase_voltage rather than the raw scaled PWM entry point.
        // Returns the ACTUAL achieved differential voltage after any clamping inside
        // set_phase_voltage() (VBUS-rail saturation, enforce_current_sense_duty_limit(),
        // enforce_min_pulse_width()) -- not necessarily equal to v_diff. Callers computing V/I
        // (resistance's regression, inductance's L=V/dI_dt) must use this return value, not
        // v_diff, or a saturated command silently biases the result (e.g. a clamped-down actual
        // voltage with the full nominal v_diff used in the formula reads out as a too-high L).
        float command_pair_voltage(phase_pair pair, float v_diff);

        // Resolves the latest measured phase currents (note: one PWM cycle stale relative to the
        // voltage just commanded, due to device::run()'s dispatch ordering -- negligible at the
        // settled/slow-ramp timescales used by resistance, but significant for inductance's
        // per-cycle ripple differencing, see run_inductance_stage()) into this pair's loop current
        // ((I_plus - I_minus)/2) and quiet-phase current (the imbalance/fault diagnostic signal).
        void read_pair_currents(phase_pair pair, float& loop_current_out, float& quiet_current_out);

        // Combines two slope estimates of the same quantity (e.g. a combo's forward vs backward
        // sweep, or a pair's + vs - combo), flagging a mismatch beyond slope_mismatch_fraction_threshold.
        static float combine_slopes(float a, float b, bool valid_a, bool valid_b, bool& out_valid, bool& out_mismatch);

        // Ends the current level (whether it completed normally or timed out without sampling):
        // advances levels_remaining and routes to the next level or, once none remain, to ADVANCE.
        void finish_level();

        function_results run_resistance_stage();
        function_results run_inductance_stage();

        // Combines each pair's +/- combo_results (each of which is itself already a fwd/bwd
        // combine) into pair_results[].r_ohms, applying the absolute plausibility bound. Runs
        // once, after all total_steps hexagon steps are done, before solve_resistance_from_pairs().
        void resolve_pair_results_from_combos();

        // Solve the 2-of-3 linear system once all 3 pairs have been tested, writing
        // measured_resistance[3] / measured_inductance[3]. Any phase whose formula depends on an
        // invalid pair -- or whose solved value falls outside a plausible absolute range, which
        // can happen even from individually-plausible pair measurements if they're mutually
        // inconsistent -- is left as NaN rather than fabricated.
        void solve_resistance_from_pairs();
        void solve_inductance_from_pairs();

};
