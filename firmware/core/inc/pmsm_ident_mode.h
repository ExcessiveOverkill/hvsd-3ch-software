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

        bool enable_resistance_measurement = true;
        bool enable_inductance_measurement = false;
        bool enable_commutation_offset_measurement = false;
        bool enable_back_emf_measurement = false;

        float measured_resistance[3] = {0.0f, 0.0f, 0.0f};  // ohms
        float measured_inductance[3] = {0.0f, 0.0f, 0.0f};  // henries
        float measured_commutation_offset = 0.0f;   // radians
        float measured_back_emf = 0.0f; // v / 1000 rpm

        float motor_calibration_current = 0.2f; // amps

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

        // mode's mtr_ch/msg/time are private to the base class, so this subclass keeps its own
        // copies of the pointers it needs to drive PWM / read currents / (eventually) report.
        // motor_channel* ident_mtr_ch;
        // Messaging* ident_msg;
        // time_interface* ident_time;

        // ---- phase-pair addressing. A pair is excited as +E on one phase, -E on another, with
        // the third ("quiet") phase held at the common reference (0) -- see command_pair_voltage(). ----
        enum class phase_pair : uint8_t { UV = 0, VW = 1, WU = 2 };
        struct pair_phases { uint8_t plus_idx; uint8_t minus_idx; uint8_t quiet_idx; }; // 0=U, 1=V, 2=W
        static constexpr pair_phases pair_phase_table[3] = {
            {0, 1, 2}, // UV: +U -V, quiet W
            {1, 2, 0}, // VW: +V -W, quiet U
            {2, 0, 1}, // WU: +W -U, quiet V
        };
        static phase_pair next_pair(phase_pair p); // UV -> VW -> WU -> UV

        // ---- top-level stage sequencing ----
        enum class ident_stage : uint8_t { ALIGN, RESISTANCE, INDUCTANCE, DONE };
        ident_stage stage = ident_stage::ALIGN;

        // ---- per-pair result storage. r_ohms/l_henries are only meaningful when the
        // corresponding *_valid flag is true -- a pair that times out (open/high-resistance
        // phase, or a failed inductance step) is left invalid rather than fabricating a value. ----
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
        static constexpr float pid_kp = 0.5f;   // TODO: bench tune against actual R/L
        static constexpr float pid_ki = 50.0f;  // TODO: bench tune
        static constexpr float pid_kd = 0.0f;   // derivative-on-measurement rarely needed for a resistive/RL plant

        float max_ident_test_voltage = 24.0f; // volts, differential -- TODO: bench tune vs VBUS/motor rating

        static constexpr float current_filter_cutoff_hz = 200.0f; // TODO: bench tune vs PWM freq / noise floor

        static constexpr float test_level_fractions[4] = {0.25f, 0.5f, 0.75f, 1.0f};
        static constexpr uint8_t num_test_levels = 4;

        static constexpr uint32_t align_ramp_hold_us      = 1000e3; // hold at align current, each polarity
        static constexpr uint32_t align_mech_settle_us    = 1000e3; // wait for velocity to damp after align
        static constexpr uint32_t current_ramp_settle_us  = 100e3;  // settle after PID reaches new level, before sampling
        static constexpr uint32_t sample_average_us       = 20e3;  // averaging window per regression sample
        static constexpr uint32_t pair_timeout_us         = 1000e3; // max time saturated-without-reaching-target before pair invalid
        static constexpr uint32_t ramp_to_zero_timeout_us = 300e3;
        static constexpr uint32_t inductance_settle_timeout_us = 200e3; // pass-1 timeout waiting for Iss
        static constexpr uint32_t inductance_ramp_down_us      = 50e3;

        static constexpr float reach_target_tolerance_amps = 0.05f;  // TODO: bench tune vs current_filter noise
        static constexpr float zero_current_tolerance_amps = 0.05f;
        static constexpr float imbalance_fraction_threshold = 0.10f; // quiet-phase current vs loop current, fraction
        static constexpr float imbalance_min_loop_current = 0.5f;    // amps; ignore imbalance check below this loop current (noise floor)
        static constexpr uint8_t inductance_skip_cycles = 2;         // skip first N cycles of pass-2 transient (switching/ADC-sync artifacts)
        static constexpr float inductance_log_cutoff_frac = 0.05f;   // stop regression once (Iss-I) < 5% of Iss (log-domain noise)
        static constexpr float inductance_settle_delta_threshold_amps = 0.02f; // successive-sample delta, absolute
        static constexpr uint8_t inductance_settle_consecutive_samples = 20;   // also reused as the post-settle averaging window length
        static constexpr float inductance_test_current_fraction = 0.5f; // fraction of motor_calibration_current targeted for the L step
        static constexpr float inductance_fallback_step_voltage = 6.0f; // used only if this pair's R was invalid -- TODO: bench tune
        static constexpr float inductance_min_slope_magnitude = 1e-4f;  // reject near-zero/positive fit slopes as bad data

        // cached cycle-count versions of the *_us constants above, computed once in configure_control_objects()
        uint32_t align_ramp_hold_cycles = 0;
        uint32_t align_mech_settle_cycles = 0;
        uint32_t current_ramp_settle_cycles = 0;
        uint32_t sample_average_cycles = 0;
        uint32_t pair_timeout_cycles = 0;
        uint32_t ramp_to_zero_timeout_cycles = 0;
        uint32_t inductance_settle_timeout_cycles = 0;
        uint32_t inductance_ramp_down_cycles = 0;

        // ---- shared control/fit objects, reused across pairs/stages (value objects, no dynamic
        // allocation). Constructed with safe placeholder dt/sample-rate in the ctor init list
        // (the real PWM frequency isn't configured yet at construction time -- these objects are
        // static-storage-duration and get built before device::init() runs), then reconfigured
        // with the real values in configure_control_objects(), called from user_reset(). ----
        pid_controller current_pid;
        low_pass_filter current_filter;
        linear_regression regression;
        bool control_objects_configured = false;
        float pwm_frequency_hz = 0.0f;
        float pwm_dt_seconds = 0.0f;
        void configure_control_objects();

        // ---- ALIGN sub-state: nudge the rotor forward/back to a magnetic detent and let
        // residual velocity damp out before any real measurement, since BEMF from a free-
        // spinning rotor would corrupt the DC-ish current measurements below. ----
        enum class align_state : uint8_t { RAMP_POS, HOLD_POS, RAMP_NEG, HOLD_NEG, RAMP_ZERO, MECH_SETTLE, DONE };
        align_state a_state = align_state::RAMP_POS;

        // generic "cycles spent in the current sub-state" counter, reused (reset to 0 on every
        // sub-state transition) across all three stages below
        uint32_t state_cycle_counter = 0;

        // ---- RESISTANCE sub-state: multi-level, same-polarity slope fit per pair. The constant
        // deadtime/Vsat-type voltage error is rejected by the fit's intercept (see plan), without
        // needing to know its magnitude. ----
        enum class resistance_state : uint8_t {
            RAMP_TO_LEVEL, SETTLE, SAMPLE, NEXT_LEVEL_OR_POLARITY, RAMP_TO_ZERO, PAIR_DONE
        };
        resistance_state r_state = resistance_state::RAMP_TO_LEVEL;
        phase_pair current_pair = phase_pair::UV;
        uint8_t level_index = 0;
        int8_t polarity = 1; // +1 then -1
        uint32_t saturation_cycle_counter = 0; // consecutive cycles at/near the output clamp -- drives open-phase/timeout detection
        bool pair_timed_out = false;
        bool pair_measurement_complete = false;
        float r_pos_slope = 0.0f; // stashed after the +polarity sweep, before regression.reset() for -polarity
        float sample_i_accum = 0.0f;
        float sample_v_accum = 0.0f;
        uint32_t sample_accum_count = 0;

        // ---- INDUCTANCE sub-state: two-pass step response. Pass 1 measures Iss directly (making
        // the fit insensitive to the same deadtime/Vsat error); pass 2 fits tau via a log-linear
        // regression of the current rise. ----
        enum class inductance_state : uint8_t { SETTLE_ZERO, STEP_RISE, RAMP_DOWN, PAIR_DONE };
        inductance_state l_state = inductance_state::SETTLE_ZERO;
        bool inductance_measuring_iss = true; // true = pass 1 (measure Iss), false = pass 2 (log-linear regression)
        bool inductance_pair_failed = false;
        float v_step = 0.0f;
        float i_ss = 0.0f;
        uint32_t rise_cycle_index = 0;
        uint32_t settled_count = 0;
        float last_filtered_i_for_settle = 0.0f;
        float iss_accum = 0.0f;
        uint32_t iss_accum_count = 0;
        float ramp_down_start_v = 0.0f; // v_step captured at the start of a ramp-down, for linear interpolation to 0

        // ================= helper methods =================

        // Commands a differential test voltage across `pair`'s +/- phases, with the third
        // ("quiet") phase held at the common reference (0). set_phase_voltage's internal
        // common-mode SVPWM offset injection preserves these relative differences exactly (a
        // provable, not approximate, property for a floating-star-point wye motor) -- this is
        // why this mode uses set_phase_voltage rather than the raw scaled PWM entry point.
        // Returns false if VBUS isn't valid yet.
        bool command_pair_voltage(phase_pair pair, float v_diff);

        // Resolves the latest measured phase currents (note: one PWM cycle stale relative to the
        // voltage just commanded, due to device::run()'s dispatch ordering -- negligible at the
        // settled/slow-ramp timescales used here) into this pair's loop current
        // ((I_plus - I_minus)/2) and quiet-phase current (the imbalance/fault diagnostic signal).
        void read_pair_currents(phase_pair pair, float& loop_current_out, float& quiet_current_out);

        function_results run_align_stage();
        function_results run_resistance_stage();
        function_results run_inductance_stage();

        // Solve the 2-of-3 linear system once all 3 pairs have been tested, writing
        // measured_resistance[3] / measured_inductance[3]. Any phase whose formula depends on an
        // invalid pair is left as NaN rather than fabricated.
        void solve_resistance_from_pairs();
        void solve_inductance_from_pairs();

};
