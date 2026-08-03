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

        // ---- tunables -- compile-time constants, matching pmsm_ident_mode's convention since
        // there's no runtime register/config system for motor-mode parameters yet. ----
        static constexpr float voltage_amplitude_v = 2.5f;       // TODO: bench tune vs motor rating / VBUS headroom
        static constexpr float peak_speed_rad_per_sec = 12.0f;    // TODO: bench tune -- peak electrical angular velocity
        static constexpr float oscillation_freq_hz = 0.1f;       // TODO: bench tune -- how often the velocity reverses
        static constexpr float amplitude_ramp_rate_v_per_sec = 20.0f; // amplitude ramp-in rate at mode start

        // Real PWM frequency isn't known until the motor channel's timer is configured, which
        // happens after this object (static-storage-duration, held in device::channel_modes) is
        // constructed -- see configure_control_objects(), called from user_reset(), same pattern
        // as pmsm_ident_mode.
        float pwm_frequency_hz = 0.0f;
        float pwm_dt_seconds = 0.0f;
        void configure_control_objects();

        float elapsed_time_seconds = 0.0f; // wrapped each oscillation period to bound fp growth on long runs
        float theta = 0.0f;                // accumulated electrical angle, wrapped to (-pi, pi]
        slew_rate_limiter amplitude_ramp;

        void user_reset() override;
        function_results user_run_flagged_all_adc_eoc() override;
};
