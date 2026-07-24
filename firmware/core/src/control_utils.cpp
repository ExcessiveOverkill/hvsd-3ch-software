#include "control_utils.h"

pid_controller::pid_controller(float kp, float ki, float kd, float dt_seconds, float out_min, float out_max)
    : kp(kp), ki(ki), kd(kd), dt(dt_seconds), out_min(out_min), out_max(out_max) {
}

float pid_controller::update(float setpoint, float measurement) {
    float error = setpoint - measurement;

    float p_term = kp * error;

    integrator += ki * error * dt;
    integrator = clamp(integrator, out_min, out_max); // clamped-integrator anti-windup

    // derivative-on-measurement avoids a derivative kick when setpoint steps
    float d_term = 0.0f;
    if(has_last_measurement) {
        d_term = -kd * (measurement - last_measurement) / dt;
    }
    last_measurement = measurement;
    has_last_measurement = true;

    float output = p_term + integrator + d_term;
    return clamp(output, out_min, out_max);
}

void pid_controller::set_gains(float kp_, float ki_, float kd_) {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void pid_controller::set_output_limits(float out_min_, float out_max_) {
    out_min = out_min_;
    out_max = out_max_;
    integrator = clamp(integrator, out_min, out_max);
}

void pid_controller::reset() {
    integrator = 0.0f;
    last_measurement = 0.0f;
    has_last_measurement = false;
}

low_pass_filter::low_pass_filter(float cutoff_hz, float sample_hz) {
    float dt = 1.0f / sample_hz;
    float rc = 1.0f / (2.0f * 3.14159265358979323846f * cutoff_hz);
    alpha = dt / (rc + dt);
}

float low_pass_filter::update(float input) {
    filtered += alpha * (input - filtered);
    return filtered;
}

void low_pass_filter::reset(float value) {
    filtered = value;
}

slew_rate_limiter::slew_rate_limiter(float max_rate_per_sec, float dt_seconds)
    : max_delta_per_step(max_rate_per_sec * dt_seconds) {
}

float slew_rate_limiter::update(float target) {
    float delta = clamp(target - output, -max_delta_per_step, max_delta_per_step);
    output += delta;
    return output;
}

void slew_rate_limiter::reset(float value) {
    output = value;
}

void linear_regression::reset() {
    sum_x = 0.0;
    sum_y = 0.0;
    sum_xy = 0.0;
    sum_xx = 0.0;
    n = 0;
}

void linear_regression::add_sample(float x, float y) {
    double xd = x;
    double yd = y;
    sum_x += xd;
    sum_y += yd;
    sum_xy += xd * yd;
    sum_xx += xd * xd;
    n++;
}

float linear_regression::slope() const {
    if(n < 2) return 0.0f;
    double denom = static_cast<double>(n) * sum_xx - sum_x * sum_x;
    if(denom == 0.0) return 0.0f; // degenerate (all x identical)
    return static_cast<float>((static_cast<double>(n) * sum_xy - sum_x * sum_y) / denom);
}

float linear_regression::intercept() const {
    if(n < 2) return 0.0f;
    return static_cast<float>((sum_y - static_cast<double>(slope()) * sum_x) / static_cast<double>(n));
}

namespace foc {

alpha_beta clarke(three_phase abc) {
    // amplitude-invariant form, using all three phases for robustness against imbalance
    alpha_beta result;
    result.alpha = (2.0f / 3.0f) * (abc.u - 0.5f * abc.v - 0.5f * abc.w);
    result.beta = (2.0f / 3.0f) * (0.8660254037844386f * (abc.v - abc.w)); // sqrt(3)/2
    return result;
}

three_phase inverse_clarke(alpha_beta ab) {
    three_phase result;
    result.u = ab.alpha;
    result.v = -0.5f * ab.alpha + 0.8660254037844386f * ab.beta;
    result.w = -0.5f * ab.alpha - 0.8660254037844386f * ab.beta;
    return result;
}

dq park(alpha_beta ab, float sin_theta, float cos_theta) {
    dq result;
    result.d = ab.alpha * cos_theta + ab.beta * sin_theta;
    result.q = -ab.alpha * sin_theta + ab.beta * cos_theta;
    return result;
}

alpha_beta inverse_park(dq dq_in, float sin_theta, float cos_theta) {
    alpha_beta result;
    result.alpha = dq_in.d * cos_theta - dq_in.q * sin_theta;
    result.beta = dq_in.d * sin_theta + dq_in.q * cos_theta;
    return result;
}

} // namespace foc
