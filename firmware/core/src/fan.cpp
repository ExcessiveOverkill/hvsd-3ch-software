#include "fan.h"

#include "board_hw.h"
#include "gpio_ll.h"

void fan_interface::init() {
    // init gpio
    gpio_ll::configure_alternate({board_hw::fan_pwm_port, board_hw::fan_pwm_pin}, board_hw::fan_pwm_pin_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::low);
    gpio_ll::configure_alternate({board_hw::fan_tach_port, board_hw::fan_tach_pin}, board_hw::fan_tach_pin_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::low);

    // enable timer clock, TODO: make this not timer instance specific
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;


    // pwm timer

    board_hw::fan_pwm_timer->CR1 = TIM_CR1_ARPE; // enable auto-reload preload

    board_hw::fan_pwm_timer->AF1 = 0;   // disable break input

    // configure for 25khz PWM frequency
    board_hw::fan_pwm_timer->PSC = 0; // no prescaler
    board_hw::fan_pwm_timer->ARR = board_hw::fan_pwm_timer_ker_clk / 25000 - 1; // auto-reload value for 25 kHz frequency

    board_hw::fan_pwm_timer->CCMR1 = (0b110u << TIM_CCMR1_OC1M_Pos) // PWM mode 1 on OC1
                                  | TIM_CCMR1_OC1PE; // enable preload for OC1
    
    board_hw::fan_pwm_timer->CCER = TIM_CCER_CC1E; // enable output for channel 1
    
    board_hw::fan_pwm_timer->CCR1 = board_hw::fan_pwm_timer->ARR / 2;   // set duty cycle to 50% initially
    
    board_hw::fan_pwm_timer->BDTR = TIM_BDTR_MOE; // enable main output
    board_hw::fan_pwm_timer->CR1 |= TIM_CR1_CEN; // enable timer


    // tachometer timer
    // use fan tach as timer clock source

    board_hw::fan_tach_timer->CR1 = TIM_CR1_ARPE; // enable auto-reload preload

    board_hw::fan_tach_timer->TISEL = 0; // select CH1 as trigger input

    board_hw::fan_tach_timer->SMCR = (0b00101 << TIM_SMCR_TS_Pos)  // trigger selection: tim_ti1fp1
                                    | (0b0111 << TIM_SMCR_SMS_Pos); // slave mode: external clock mode 1 (clock from trigger)

    board_hw::fan_tach_timer->PSC = 0; // no prescaler
    
    board_hw::fan_tach_timer->ARR = 0xFFFF; // max auto-reload value for 16-bit timer

    board_hw::fan_tach_timer->CCMR1 = (0b1111 << TIM_CCMR1_IC1F_Pos) // input capture filter, set to max to filter out noise
                                    | (0b01 << TIM_CCMR1_CC1S_Pos);  // CC1 channel is configured as input, IC1 is mapped on TI1

    board_hw::fan_tach_timer->CCER = TIM_CCER_CC1P; // capture on falling edge (active low from fan tach)
    board_hw::fan_tach_timer->CR1 |= TIM_CR1_CEN; // enable timer

}

void fan_interface::systick_flagged_handler() {
    update_counter++;
    if (update_counter >= update_divider) {
        update_counter = 0;

        // calculate RPM from captured timer value

        uint32_t ticks = board_hw::fan_tach_timer->CNT;
        board_hw::fan_tach_timer->CNT = 0; // reset counter for next measurement

        rpm = ticks * board_hw::systick_tick_hz / update_divider * 60 / pulses_per_revolution;
    }
}

void fan_interface::set_speed_percent(uint8_t percent) {
    if (percent > 100) {
        percent = 100;
    }

    // set duty cycle of PWM timer based on percent
    board_hw::fan_pwm_timer->CCR1 = board_hw::fan_pwm_timer->ARR * percent / 100;
}

uint16_t fan_interface::get_rpm() {
    return rpm;
}
