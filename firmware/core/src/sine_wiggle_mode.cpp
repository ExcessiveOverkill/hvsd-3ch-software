#include "sine_wiggle_mode.h"

#include <cmath>

sine_wiggle_mode::sine_wiggle_mode(motor_channel* mtr_ch_, Messaging* msg_, time_interface* time_, cordic* cordic_)
    : mode(mtr_ch_, msg_, time_),
      cordic_drv(cordic_),
      amplitude_ramp(amplitude_ramp_rate_v_per_sec, 1.0f / 1000.0f),
      current_ramp(current_ramp_rate_amps_per_sec, 1.0f / 1000.0f),
      id_pid(current_pid_kp, current_pid_ki, current_pid_kd, 1.0f / 1000.0f, -max_current_control_voltage_v, max_current_control_voltage_v),
      iq_pid(current_pid_kp, current_pid_ki, current_pid_kd, 1.0f / 1000.0f, -max_current_control_voltage_v, max_current_control_voltage_v),
      id_filter(current_filter_cutoff_hz, 1000.0f),
      iq_filter(current_filter_cutoff_hz, 1000.0f)
{
}

void sine_wiggle_mode::configure_control_objects() {
    pwm_frequency_hz = static_cast<float>(mtr_ch->calculate_pwm_cycles_from_us(1000000));
    pwm_dt_seconds = 1.0f / pwm_frequency_hz;

    amplitude_ramp = slew_rate_limiter(amplitude_ramp_rate_v_per_sec, pwm_dt_seconds);
    current_ramp = slew_rate_limiter(current_ramp_rate_amps_per_sec, pwm_dt_seconds);
    id_pid = pid_controller(current_pid_kp, current_pid_ki, current_pid_kd, pwm_dt_seconds, -max_current_control_voltage_v, max_current_control_voltage_v);
    iq_pid = pid_controller(current_pid_kp, current_pid_ki, current_pid_kd, pwm_dt_seconds, -max_current_control_voltage_v, max_current_control_voltage_v);
    id_filter = low_pass_filter(current_filter_cutoff_hz, pwm_frequency_hz);
    iq_filter = low_pass_filter(current_filter_cutoff_hz, pwm_frequency_hz);
}

void sine_wiggle_mode::user_reset() {
    elapsed_time_seconds = 0.0f;
    theta = 0.0f;
    prev_sin_theta = 0.0f;
    prev_cos_theta = 1.0f;

    configure_control_objects();

    amplitude_ramp.reset(0.0f);
    current_ramp.reset(0.0f);
    id_pid.reset();
    iq_pid.reset();
    id_filter.reset(0.0f);
    iq_filter.reset(0.0f);
}

mode::function_results sine_wiggle_mode::user_run_flagged_all_adc_eoc() {
    constexpr float two_pi = 2.0f * 3.14159265358979323846f;

    elapsed_time_seconds += pwm_dt_seconds;
    elapsed_time_seconds = std::fmod(elapsed_time_seconds, 1.0f / oscillation_freq_hz);

    float omega = peak_speed_rad_per_sec * std::sin(two_pi * oscillation_freq_hz * elapsed_time_seconds);
    theta = wrap_angle(theta + omega * pwm_dt_seconds);

    float sin_theta, cos_theta;
    cordic_drv->sin_cos(theta, sin_theta, cos_theta);

    foc::dq voltage_cmd;

    if(use_current_control) {
        // get_phase_currents() is one PWM cycle stale relative to the voltage just commanded (see
        // prev_sin_theta/prev_cos_theta's comment in the header) -- Park-transform this reading
        // against the angle that was actually in effect when it was produced, not `theta`.
        float iu, iv, iw;
        mtr_ch->get_phase_currents(iu, iv, iw);
        foc::alpha_beta i_ab = foc::clarke(foc::three_phase{iu, iv, iw});
        foc::dq i_dq = foc::park(i_ab, prev_sin_theta, prev_cos_theta);

        float id_filt = id_filter.update(i_dq.d);
        float iq_filt = iq_filter.update(i_dq.q);

        float id_target = current_ramp.update(current_amplitude_amps);

        voltage_cmd.d = id_pid.update(id_target, id_filt);
        voltage_cmd.q = iq_pid.update(0.0f, iq_filt);
    }
    else {
        // d=amplitude, q=0 -> a vector of fixed magnitude at angle theta. There's no real
        // rotor-aligned dq frame here (open loop, no encoder), so this is just the cleanest way to
        // reuse the existing park/clarke transforms to get a rotating vector of known magnitude/angle.
        voltage_cmd.d = amplitude_ramp.update(voltage_amplitude_v);
        voltage_cmd.q = 0.0f;
    }

    foc::alpha_beta ab = foc::inverse_park(voltage_cmd, sin_theta, cos_theta);
    foc::three_phase abc = foc::inverse_clarke(ab);

    float achieved_u, achieved_v, achieved_w;
    mtr_ch->set_phase_voltage(abc.u, abc.v, abc.w, &achieved_u, &achieved_v, &achieved_w);

    prev_sin_theta = sin_theta;
    prev_cos_theta = cos_theta;

    return function_results::CONTINUE;
}
