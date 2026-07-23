#include "cordic.h"

#include <cmath>

namespace {
    constexpr float pi = 3.14159265358979323846f;

    constexpr uint8_t func_cosine = 0;
    constexpr uint8_t func_phase  = 2;
    constexpr uint8_t func_modulus = 3;
}

void cordic::init()
{
    // CORDIC is on AHB1, see RM0440 CORDIC chapter
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    (void)RCC->AHB1ENR;
}

int32_t cordic::float_to_q1_31(float value)
{
    if(value >= 1.0f){ return INT32_MAX; }
    if(value < -1.0f){ return INT32_MIN; }
    return static_cast<int32_t>(value * 2147483648.0f);
}

float cordic::q1_31_to_float(int32_t value)
{
    return static_cast<float>(value) / 2147483648.0f;
}

void cordic::wait_ready()
{
    while(!(CORDIC->CSR & CORDIC_CSR_RRDY)){}
}

cordic::dual_result cordic::calculate(uint8_t func, float arg1, float arg2, bool two_args, bool two_results,
                                       uint8_t precision, uint8_t scale)
{
    uint32_t csr = (static_cast<uint32_t>(func) << CORDIC_CSR_FUNC_Pos)
                 | (static_cast<uint32_t>(precision) << CORDIC_CSR_PRECISION_Pos)
                 | (static_cast<uint32_t>(scale) << CORDIC_CSR_SCALE_Pos);
    // ARGSIZE/RESSIZE left at 0: always full 32-bit q1.31 for float-level precision
    if(two_results){ csr |= CORDIC_CSR_NRES; }
    if(two_args){ csr |= CORDIC_CSR_NARGS; }
    CORDIC->CSR = csr;

    CORDIC->WDATA = static_cast<uint32_t>(float_to_q1_31(arg1));
    if(two_args){
        CORDIC->WDATA = static_cast<uint32_t>(float_to_q1_31(arg2));
    }

    wait_ready();

    dual_result result{};
    result.primary = q1_31_to_float(static_cast<int32_t>(CORDIC->RDATA));
    if(two_results){
        result.secondary = q1_31_to_float(static_cast<int32_t>(CORDIC->RDATA));
    }
    return result;
}

void cordic::sin_cos(float angle_rad, float& sin_out, float& cos_out)
{
    // angle argument is angle/pi in q1.31; FUNC=Cosine with two results gives cos then sin
    dual_result result = calculate(func_cosine, angle_rad / pi, 0.0f, false, true);
    cos_out = result.primary;
    sin_out = result.secondary;
}

float cordic::sine(float angle_rad)
{
    float sin_val, cos_val;
    sin_cos(angle_rad, sin_val, cos_val);
    return sin_val;
}

float cordic::cosine(float angle_rad)
{
    float sin_val, cos_val;
    sin_cos(angle_rad, sin_val, cos_val);
    return cos_val;
}

float cordic::atan2(float y, float x)
{
    // Phase function: primary argument is x, secondary is y; result is angle/pi
    dual_result result = calculate(func_phase, x, y, true, false);
    return result.primary * pi;
}

float cordic::magnitude(float x, float y)
{
    // Modulus function: primary argument is x, secondary is y
    dual_result result = calculate(func_modulus, x, y, true, false);
    return result.primary;
}
