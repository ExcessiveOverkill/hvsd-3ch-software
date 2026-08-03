#pragma once

#include "mode.h"
#include "cordic.h"

/*
Open-loop demo mode: commands a rotating 3-phase voltage vector of fixed magnitude whose
electrical angle oscillates forward and backward (angular velocity follows a sine wave,
integrated into an angle) rather than spinning continuously. No rotor position feedback is
used or needed -- this just proves all three phases of a channel are wired and switching
correctly by making the motor visibly wiggle.
*/

class sine_wiggle_mode : public mode {
    public:
        sine_wiggle_mode(motor_channel* mtr_ch_, Messaging* msg_, time_interface* time_, cordic* cordic_);

    private:
        cordic* cordic_drv;

        // Selects the amplitude source: false = open-loop fixed voltage vector, true = closed-loop
        // Id/Iq current control. Plain runtime-mutable flag (not constexpr), matching
        // pmsm_ident_mode's enable_resistance_measurement/enable_inductance_measurement convention
        // -- no runtime register/config system exists yet, so this is flipped by editing the
        // default and reflashing.
        bool use_current_control = true;

        // ---- tunables -- compile-time constants, matching pmsm_ident_mode's convention since
        // there's no runtime register/config system for motor-mode parameters yet. ----
        static constexpr float peak_speed_rad_per_sec = 40.0f;    // TODO: bench tune -- peak electrical angular velocity
        static constexpr float oscillation_freq_hz = 0.1f;       // TODO: bench tune -- how often the velocity reverses

        // ---- open-loop voltage mode ----
        static constexpr float voltage_amplitude_v = 2.5f;       // TODO: bench tune vs motor rating / VBUS headroom
        static constexpr float amplitude_ramp_rate_v_per_sec = 20.0f; // amplitude ramp-in rate at mode start

        // ---- closed-loop current-control mode -- Id is regulated to the wiggle's active
        // amplitude and Iq to 0, matching the open-loop path's d=amplitude,q=0 convention (there's
        // no real rotor-aligned frame here -- no encoder -- so "d" vs "q" is just a label; keeping
        // the same one in both modes avoids an unexplained 90 degree behavioral shift when toggling
        // use_current_control). ----
        static constexpr float current_amplitude_amps = 1.5f;      // TODO: bench tune -- well under board_hw::ipms.max_phase_current_amps
        static constexpr float current_ramp_rate_amps_per_sec = 5.0f; // Id-target ramp-in rate at mode start
        static constexpr float current_pid_kp = 1.0f;   // TODO: bench tune against actual R/L, shared by id_pid/iq_pid
        static constexpr float current_pid_ki = 50.0f;  // TODO: bench tune
        static constexpr float current_pid_kd = 0.0f;
        static constexpr float current_filter_cutoff_hz = 200.0f; // TODO: bench tune vs PWM freq / noise floor
        static constexpr float max_current_control_voltage_v = 24.0f; // volts, TODO: bench tune vs VBUS/motor rating -- id_pid/iq_pid output clamp

        // Real PWM frequency isn't known until the motor channel's timer is configured, which
        // happens after this object (static-storage-duration, held in device::channel_modes) is
        // constructed -- see configure_control_objects(), called from user_reset(), same pattern
        // as pmsm_ident_mode.
        float pwm_frequency_hz = 0.0f;
        float pwm_dt_seconds = 0.0f;
        void configure_control_objects();

        float elapsed_time_seconds = 0.0f; // wrapped each oscillation period to bound fp growth on long runs
        float theta = 0.0f;                // accumulated electrical angle, wrapped to (-pi, pi]
        slew_rate_limiter amplitude_ramp;  // open-loop voltage amplitude ramp-in
        slew_rate_limiter current_ramp;    // current-control Id-target ramp-in

        pid_controller id_pid;
        pid_controller iq_pid;
        low_pass_filter id_filter;
        low_pass_filter iq_filter;

        // Angle in effect when the most recently commanded voltage was applied -- get_phase_currents()
        // is one PWM cycle stale relative to the voltage just commanded (device::run()'s dispatch
        // order runs the mode before the phase-current ADC handler refreshes it each cycle, see
        // pmsm_ident_mode.h's read_pair_currents() comment), so a freshly-read current must be
        // Park-transformed against the angle that was active when it was produced, not the current
        // cycle's angle. Initialized to match theta's initial value of 0 (sin=0, cos=1).
        float prev_sin_theta = 0.0f;
        float prev_cos_theta = 1.0f;

        void user_reset() override;
        function_results user_run_flagged_all_adc_eoc() override;
};
