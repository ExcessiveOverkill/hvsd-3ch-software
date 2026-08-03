#pragma once

#include <cstdint>
#include <cmath>

// Generic math/control-theory building blocks for servo control loops (current,
// velocity, position). Deliberately has no hardware or Messaging dependency so it
// stays reusable across loops and testable off-target.

template<typename T>
constexpr T clamp(T value, T min, T max) {
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

constexpr float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

// Normalizes an angle in radians to (-pi, pi], the domain expected by cordic::sin_cos.
inline float wrap_angle(float angle_rad) {
    constexpr float pi = 3.14159265358979323846f;
    constexpr float two_pi = 2.0f * pi;
    angle_rad = std::fmod(angle_rad + pi, two_pi);
    if(angle_rad < 0.0f) angle_rad += two_pi;
    return angle_rad - pi;
}

// Discrete PID controller with clamped-integrator anti-windup and derivative-on-measurement
// (avoids derivative kick from setpoint steps). Runs at a fixed sample period set at
// construction, matching the fixed-rate ADC/PWM ISR cadence control loops run at.
class pid_controller {
    public:
        pid_controller(float kp, float ki, float kd, float dt_seconds, float out_min, float out_max);

        float update(float setpoint, float measurement);

        void set_gains(float kp, float ki, float kd);
        void set_output_limits(float out_min, float out_max);
        void reset();

    private:
        float kp;
        float ki;
        float kd;
        float dt;
        float out_min;
        float out_max;

        float integrator = 0.0f;
        float last_measurement = 0.0f;
        bool has_last_measurement = false;
};

// First-order (single-pole) IIR low-pass filter.
class low_pass_filter {
    public:
        // cutoff_hz is the -3dB corner frequency; sample_hz is the update() call rate.
        low_pass_filter(float cutoff_hz, float sample_hz);

        float update(float input);
        void reset(float value = 0.0f);
        float value() const { return filtered; }

    private:
        float alpha;
        float filtered = 0.0f;
};

// Limits the rate of change of a reference signal, e.g. for smoothly ramping current,
// velocity, or position setpoints.
class slew_rate_limiter {
    public:
        slew_rate_limiter(float max_rate_per_sec, float dt_seconds);

        float update(float target);
        void reset(float value);

    private:
        float max_delta_per_step;
        float output = 0.0f;
};

// Incremental (streaming) least-squares line fit: y = slope*x + intercept. Keeps only running
// sums rather than buffering samples, so it's safe to feed one sample per PWM cycle from an
// ISR-driven control loop for an arbitrarily long fit without growing memory.
class linear_regression {
    public:
        void reset();
        void add_sample(float x, float y);

        // Undefined (returns 0.0f) if count() < 2 -- check valid() before trusting these for
        // anything decision-relevant.
        float slope() const;
        float intercept() const;
        uint32_t count() const { return n; }
        bool valid() const { return n >= 2; }

    private:
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_xy = 0.0;
        double sum_xx = 0.0;
        uint32_t n = 0;
};

struct current_loop_gains {
    float kp;
    float ki;
};

// Computes PI current-loop gains via pole-zero cancellation: wc = 2*pi*(pwm_frequency_hz /
// current_loop_bandwidth_divisor), Kp = wc*l_henries, Ki = wc*r_ohms.
//
// Deliberately conservative rather than the most aggressive stable bandwidth: identified L is
// documented elsewhere as "apparent loop inductance," not a validated per-phase value -- it can
// be biased high OR low by uncancelled mutual coupling, direction unknown. Real hardware also
// has ~1 PWM cycle of measurement delay the ideal continuous-time model doesn't capture. Backing
// off bandwidth is the only sound response to a direction-unknown error.
//
// If r_ohms/l_henries are non-finite (e.g. NaN from failed identification) or fall outside
// [r_min_ohms, r_max_ohms] / [l_min_henries, l_max_henries], or pwm_frequency_hz <= 0, returns a
// hardcoded conservative fallback instead of computing from bad data. Default bounds are loose;
// callers with tighter domain knowledge (e.g. a specific ident routine's own plausibility
// bounds) can pass narrower ones.
current_loop_gains compute_current_loop_gains(
    float r_ohms, float l_henries, float pwm_frequency_hz,
    float r_min_ohms = 1e-3f, float r_max_ohms = 100.0f,
    float l_min_henries = 1e-6f, float l_max_henries = 1.0f);

// Stateless Clarke/Park transforms for field-oriented control. sin_theta/cos_theta are
// expected to come from cordic::sin_cos(wrap_angle(theta), ...) - kept as explicit
// parameters here so this file has no dependency on the CORDIC peripheral.
namespace foc {

    struct three_phase {
        float u;
        float v;
        float w;
    };

    struct alpha_beta {
        float alpha;
        float beta;
    };

    struct dq {
        float d;
        float q;
    };

    // Amplitude-invariant Clarke transform, using all three phases rather than assuming
    // a balanced 2-phase-only input so it stays robust to phase imbalance/noise.
    alpha_beta clarke(three_phase abc);
    three_phase inverse_clarke(alpha_beta ab);

    dq park(alpha_beta ab, float sin_theta, float cos_theta);
    alpha_beta inverse_park(dq dq_in, float sin_theta, float cos_theta);

} // namespace foc
