#include "board_hw.h"

namespace board_hw {
motor_hw_config motor_configs[3] = {
    // Motor A
    {
        .phase_u_adc_port = phase_a_u_adc_port,
        .phase_u_adc_pin = phase_a_u_adc_pin,
        .phase_u_adc_channel = phase_a_u_adc_channel,
        .phase_v_adc_port = phase_a_v_adc_port,
        .phase_v_adc_pin = phase_a_v_adc_pin,
        .phase_v_adc_channel = phase_a_v_adc_channel,
        .phase_w_adc_port = phase_a_w_adc_port,
        .phase_w_adc_pin = phase_a_w_adc_pin,
        .phase_w_adc_channel = phase_a_w_adc_channel,
        .phase_u_adc_timer_reg_extsel = phase_u_adc_timer_a_reg_extsel,
        .phase_v_adc_timer_reg_extsel = phase_v_adc_timer_a_reg_extsel,
        .phase_w_adc_timer_reg_extsel = phase_w_adc_timer_a_reg_extsel,

        // TIM
        .timer = phase_pwm_a_timer,
        .timer_itr = phase_pwm_a_timer_itr,

        // GPIOs for PWM outputs
        .phase_u_high_port = phase_pwm_a_u_high_port,
        .phase_u_high_pin = phase_pwm_a_u_high_pin,
        .phase_u_high_pin_alternate_function = phase_pwm_a_u_high_pin_alternate_function,
        .phase_u_low_port = phase_pwm_a_u_low_port,
        .phase_u_low_pin = phase_pwm_a_u_low_pin,
        .phase_u_low_pin_alternate_function = phase_pwm_a_u_low_pin_alternate_function,
        .phase_u_ch = phase_pwm_a_u_ch,
        .phase_v_high_port = phase_pwm_a_v_high_port,
        .phase_v_high_pin = phase_pwm_a_v_high_pin,
        .phase_v_high_pin_alternate_function = phase_pwm_a_v_high_pin_alternate_function,
        .phase_v_low_port = phase_pwm_a_v_low_port,
        .phase_v_low_pin = phase_pwm_a_v_low_pin,
        .phase_v_low_pin_alternate_function = phase_pwm_a_v_low_pin_alternate_function,
        .phase_v_ch = phase_pwm_a_v_ch,
        .phase_w_high_port = phase_pwm_a_w_high_port,
        .phase_w_high_pin = phase_pwm_a_w_high_pin,
        .phase_w_high_pin_alternate_function = phase_pwm_a_w_high_pin_alternate_function,
        .phase_w_low_port = phase_pwm_a_w_low_port,
        .phase_w_low_pin = phase_pwm_a_w_low_pin,
        .phase_w_low_pin_alternate_function = phase_pwm_a_w_low_pin_alternate_function,
        .phase_w_ch = phase_pwm_a_w_ch,

        // Break GPIOs
        .break_port = phase_pwm_a_break_port,
        .break_pin = phase_pwm_a_break_pin,
        .break_pin_alternate_function = phase_pwm_a_break_pin_alternate_function,
        .break_2_port = phase_pwm_a_break_2_port,
        .break_2_pin = phase_pwm_a_break_2_pin,
        .break_2_pin_alternate_function = phase_pwm_a_break_2_pin_alternate_function,

        // Fault GPIOs
        .fault_port = inverter_a_fault_port,
        .fault_pin = inverter_a_fault_pin,

    },

    // Motor B
    {
        .phase_u_adc_port = phase_b_u_adc_port,
        .phase_u_adc_pin = phase_b_u_adc_pin,
        .phase_u_adc_channel = phase_b_u_adc_channel,
        .phase_v_adc_port = phase_b_v_adc_port,
        .phase_v_adc_pin = phase_b_v_adc_pin,
        .phase_v_adc_channel = phase_b_v_adc_channel,
        .phase_w_adc_port = phase_b_w_adc_port,
        .phase_w_adc_pin = phase_b_w_adc_pin,
        .phase_w_adc_channel = phase_b_w_adc_channel,
        .phase_u_adc_timer_reg_extsel = phase_u_adc_timer_b_reg_extsel,
        .phase_v_adc_timer_reg_extsel = phase_v_adc_timer_b_reg_extsel,
        .phase_w_adc_timer_reg_extsel = phase_w_adc_timer_b_reg_extsel,

        // TIM
        .timer = phase_pwm_b_timer,
        .timer_itr = phase_pwm_b_timer_itr,

        // GPIOs for PWM outputs
        .phase_u_high_port = phase_pwm_b_u_high_port,
        .phase_u_high_pin = phase_pwm_b_u_high_pin,
        .phase_u_high_pin_alternate_function = phase_pwm_b_u_high_pin_alternate_function,
        .phase_u_low_port = phase_pwm_b_u_low_port,
        .phase_u_low_pin = phase_pwm_b_u_low_pin,
        .phase_u_low_pin_alternate_function = phase_pwm_b_u_low_pin_alternate_function,
        .phase_u_ch = phase_pwm_b_u_ch,
        .phase_v_high_port = phase_pwm_b_v_high_port,
        .phase_v_high_pin = phase_pwm_b_v_high_pin,
        .phase_v_high_pin_alternate_function = phase_pwm_b_v_high_pin_alternate_function,
        .phase_v_low_port = phase_pwm_b_v_low_port,
        .phase_v_low_pin = phase_pwm_b_v_low_pin,
        .phase_v_low_pin_alternate_function = phase_pwm_b_v_low_pin_alternate_function,
        .phase_v_ch = phase_pwm_b_v_ch,
        .phase_w_high_port = phase_pwm_b_w_high_port,
        .phase_w_high_pin = phase_pwm_b_w_high_pin,
        .phase_w_high_pin_alternate_function = phase_pwm_b_w_high_pin_alternate_function,
        .phase_w_low_port = phase_pwm_b_w_low_port,
        .phase_w_low_pin = phase_pwm_b_w_low_pin,
        .phase_w_low_pin_alternate_function = phase_pwm_b_w_low_pin_alternate_function,
        .phase_w_ch = phase_pwm_b_w_ch,

        // Break GPIOs
        .break_port = phase_pwm_b_break_port,
        .break_pin = phase_pwm_b_break_pin,
        .break_pin_alternate_function = phase_pwm_b_break_pin_alternate_function,
        .break_2_port = phase_pwm_b_break_2_port,
        .break_2_pin = phase_pwm_b_break_2_pin,
        .break_2_pin_alternate_function = phase_pwm_b_break_2_pin_alternate_function,

        // Fault GPIOs
        .fault_port = inverter_b_fault_port,
        .fault_pin = inverter_b_fault_pin,
    },

    // Motor C
    {
        .phase_u_adc_port = phase_c_u_adc_port,
        .phase_u_adc_pin = phase_c_u_adc_pin,
        .phase_u_adc_channel = phase_c_u_adc_channel,
        .phase_v_adc_port = phase_c_v_adc_port,
        .phase_v_adc_pin = phase_c_v_adc_pin,
        .phase_v_adc_channel = phase_c_v_adc_channel,
        .phase_w_adc_port = phase_c_w_adc_port,
        .phase_w_adc_pin = phase_c_w_adc_pin,
        .phase_w_adc_channel = phase_c_w_adc_channel,
        .phase_u_adc_timer_reg_extsel = phase_u_adc_timer_c_reg_extsel,
        .phase_v_adc_timer_reg_extsel = phase_v_adc_timer_c_reg_extsel,
        .phase_w_adc_timer_reg_extsel = phase_w_adc_timer_c_reg_extsel,

        // TIM
        .timer = phase_pwm_c_timer,
        .timer_itr = phase_pwm_c_timer_itr,

        // GPIOs for PWM outputs
        .phase_u_high_port = phase_pwm_c_u_high_port,
        .phase_u_high_pin = phase_pwm_c_u_high_pin,
        .phase_u_high_pin_alternate_function = phase_pwm_c_u_high_pin_alternate_function,
        .phase_u_low_port = phase_pwm_c_u_low_port,
        .phase_u_low_pin = phase_pwm_c_u_low_pin,
        .phase_u_low_pin_alternate_function = phase_pwm_c_u_low_pin_alternate_function,
        .phase_u_ch = phase_pwm_c_u_ch,
        .phase_v_high_port = phase_pwm_c_v_high_port,
        .phase_v_high_pin = phase_pwm_c_v_high_pin,
        .phase_v_high_pin_alternate_function = phase_pwm_c_v_high_pin_alternate_function,
        .phase_v_low_port = phase_pwm_c_v_low_port,
        .phase_v_low_pin = phase_pwm_c_v_low_pin,
        .phase_v_low_pin_alternate_function = phase_pwm_c_v_low_pin_alternate_function,
        .phase_v_ch = phase_pwm_c_v_ch,
        .phase_w_high_port = phase_pwm_c_w_high_port,
        .phase_w_high_pin = phase_pwm_c_w_high_pin,
        .phase_w_high_pin_alternate_function = phase_pwm_c_w_high_pin_alternate_function,
        .phase_w_low_port = phase_pwm_c_w_low_port,
        .phase_w_low_pin = phase_pwm_c_w_low_pin,
        .phase_w_low_pin_alternate_function = phase_pwm_c_w_low_pin_alternate_function,
        .phase_w_ch = phase_pwm_c_w_ch,

        // Break GPIOs
        .break_port = phase_pwm_c_break_port,
        .break_pin = phase_pwm_c_break_pin,
        .break_pin_alternate_function = phase_pwm_c_break_pin_alternate_function,
        .break_2_port = phase_pwm_c_break_2_port,
        .break_2_pin = phase_pwm_c_break_2_pin,
        .break_2_pin_alternate_function = phase_pwm_c_break_2_pin_alternate_function,

        // Fault GPIOs
        .fault_port = inverter_c_fault_port,
        .fault_pin = inverter_c_fault_pin,
    }
};
} // namespace board_hw