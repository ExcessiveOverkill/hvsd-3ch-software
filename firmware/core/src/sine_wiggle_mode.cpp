#include "sine_wiggle_mode.h"

#include <cmath>

sine_wiggle_mode::sine_wiggle_mode(motor_channel* mtr_ch_, Messaging* msg_, time_interface* time_, cordic* cordic_)
    : mode(mtr_ch_, msg_, time_),
      cordic_drv(cordic_),
      amplitude_ramp(amplitude_ramp_rate_v_per_sec, 1.0f / 1000.0f)
{
}

void sine_wiggle_mode::configure_control_objects() {
    pwm_frequency_hz = static_cast<float>(mtr_ch->calculate_pwm_cycles_from_us(1000000));
    pwm_dt_seconds = 1.0f / pwm_frequency_hz;

    amplitude_ramp = slew_rate_limiter(amplitude_ramp_rate_v_per_sec, pwm_dt_seconds);
}

void sine_wiggle_mode::user_reset() {
    elapsed_time_seconds = 0.0f;
    theta = 0.0f;

    configure_control_objects();

    amplitude_ramp.reset(0.0f);
}

mode::function_results sine_wiggle_mode::user_run_flagged_all_adc_eoc() {
    constexpr float two_pi = 2.0f * 3.14159265358979323846f;

    elapsed_time_seconds += pwm_dt_seconds;
    elapsed_time_seconds = std::fmod(elapsed_time_seconds, 1.0f / oscillation_freq_hz);

    float omega = peak_speed_rad_per_sec * std::sin(two_pi * oscillation_freq_hz * elapsed_time_seconds);
    theta = wrap_angle(theta + omega * pwm_dt_seconds);

    float amplitude = amplitude_ramp.update(voltage_amplitude_v);

    float sin_theta, cos_theta;
    cordic_drv->sin_cos(theta, sin_theta, cos_theta);

    // d=amplitude, q=0 -> a vector of fixed magnitude `amplitude` at angle theta. There's no
    // real rotor-aligned dq frame here (open loop, no encoder), so this is just the cleanest way
    // to reuse the existing park/clarke transforms to get a rotating vector of known magnitude/angle.
    foc::alpha_beta ab = foc::inverse_park(foc::dq{amplitude, 0.0f}, sin_theta, cos_theta);
    foc::three_phase abc = foc::inverse_clarke(ab);

    float achieved_u, achieved_v, achieved_w;
    mtr_ch->set_phase_voltage(abc.u, abc.v, abc.w, &achieved_u, &achieved_v, &achieved_w);

    return function_results::CONTINUE;
}
