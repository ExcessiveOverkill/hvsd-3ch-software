#include "motor_pwm.h"

#include "board_hw.h"
#include "gpio_ll.h"

namespace motor_pwm {

namespace {

uint16_t pwm_arr = 0u;

void enable_timer_clock(TIM_TypeDef* timer) {
    if (timer == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    } else if (timer == TIM15) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    } else if (timer == TIM2) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    } else if (timer == TIM3) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    }

    (void)RCC->APB2ENR;
    (void)RCC->APB1ENR1;
}

} // namespace

void init() {
    // TODO: fill
}

} // namespace motor_pwm
