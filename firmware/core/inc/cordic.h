#pragma once

#include <cstdint>
#include "stm32g473xx.h"

// Floating-point driver for the STM32G4 CORDIC coprocessor (RM0440 ch. 17).
// Blocking/polled: a 32-bit computation takes on the order of ~15-30 core
// clocks, well under interrupt-latency overhead, so all calls busy-wait on
// CORDIC_CSR_RRDY rather than using CORDIC_IRQn.
class cordic {
    public:

    void init(); // enables the AHB1 CORDIC clock

    // angle_rad should be wrapped to (-pi, pi]; results are exact sin/cos of that angle
    void sin_cos(float angle_rad, float& sin_out, float& cos_out);
    float sine(float angle_rad);
    float cosine(float angle_rad);

    // atan2(y, x) in radians; x and y must be pre-scaled into (-1, 1)
    float atan2(float y, float x);

    // sqrt(x*x + y*y); x and y must be pre-scaled into (-1, 1)
    float magnitude(float x, float y);

    struct dual_result {
        float primary;
        float secondary;
    };

    // Low-level escape hatch for functions/precision/scale not covered by the
    // wrappers above (e.g. square root, natural log, hyperbolic functions).
    // func is a CORDIC_CSR FUNC code (RM0440 Table 91): 0=Cosine, 1=Sine,
    // 2=Phase, 3=Modulus, 4=Arctangent, 5=Hyperbolic cosine, 6=Hyperbolic
    // sine, 7=Arctanh, 8=Natural log, 9=Square root. Consult RM0440 ch. 17
    // for the input-domain/scale requirements of each function before use.
    dual_result calculate(uint8_t func, float arg1, float arg2, bool two_args, bool two_results,
                           uint8_t precision = default_precision, uint8_t scale = 0);

    private:

    static constexpr uint8_t default_precision = 6; // 24 iterations: full accuracy for q1.31 data

    static int32_t float_to_q1_31(float value);
    static float q1_31_to_float(int32_t value);
    static void wait_ready();
};
