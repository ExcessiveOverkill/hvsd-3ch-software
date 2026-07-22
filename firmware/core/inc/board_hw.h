#pragma once

#include <cstdint>

#include "stm32g473xx.h"

namespace board_hw {

struct motor_hw_config {

    // ADC
    GPIO_TypeDef* phase_u_adc_port;
    uint8_t phase_u_adc_pin;
    uint8_t phase_u_adc_channel;

    GPIO_TypeDef* phase_v_adc_port;
    uint8_t phase_v_adc_pin;
    uint8_t phase_v_adc_channel;

    GPIO_TypeDef* phase_w_adc_port;
    uint8_t phase_w_adc_pin;
    uint8_t phase_w_adc_channel;

    uint8_t phase_u_adc_timer_reg_extsel;
    uint8_t phase_v_adc_timer_reg_extsel;
    uint8_t phase_w_adc_timer_reg_extsel;

    // TIM
    TIM_TypeDef* timer;
    uint16_t timer_itr;
    
    GPIO_TypeDef* phase_u_high_port;
    uint8_t phase_u_high_pin;
    uint8_t phase_u_high_pin_alternate_function;
    GPIO_TypeDef* phase_u_low_port;
    uint8_t phase_u_low_pin;
    uint8_t phase_u_low_pin_alternate_function;
    uint8_t phase_u_ch;

    GPIO_TypeDef* phase_v_high_port;
    uint8_t phase_v_high_pin;
    uint8_t phase_v_high_pin_alternate_function;
    GPIO_TypeDef* phase_v_low_port;
    uint8_t phase_v_low_pin;
    uint8_t phase_v_low_pin_alternate_function;
    uint8_t phase_v_ch;

    GPIO_TypeDef* phase_w_high_port;
    uint8_t phase_w_high_pin;
    uint8_t phase_w_high_pin_alternate_function;
    GPIO_TypeDef* phase_w_low_port;
    uint8_t phase_w_low_pin;
    uint8_t phase_w_low_pin_alternate_function;
    uint8_t phase_w_ch;

    GPIO_TypeDef* break_port;
    uint8_t break_pin;
    uint8_t break_pin_alternate_function;
    GPIO_TypeDef* break_2_port;
    uint8_t break_2_pin;
    uint8_t break_2_pin_alternate_function;

    // inverter fault
    GPIO_TypeDef* fault_port;
    uint8_t fault_pin;

};

extern motor_hw_config motor_configs[3];

// current sense hardware
inline constexpr float current_sense_resistance = 0.0025f; // 2.5 milliohm shunt
inline constexpr float current_sense_gain = 12.1f / 1.0f; // gain of the current sense amplifier

// VBUS sense hardware
inline constexpr float vbus_sense_resistor_top = 2.25e6f; // 2.25 MOhm
inline constexpr float vbus_sense_resistor_bottom = 10e3f; // 10 kOhm

// gate supply sense hardware
inline constexpr float gate_supply_sense_resistor_top = 10e3f; // 10 kOhm
inline constexpr float gate_supply_sense_resistor_bottom = 1e3f; // 1 kOhm

// VREF
inline constexpr bool use_external_vref = false; // set to true if vref is tied to 3.3v
inline constexpr float external_vref_voltage = 3.3f;
inline constexpr float internal_vref_voltage = 2.90f; // 2.048v, 2.5v, or 2.90v, see RM0440 table 197

static_assert(internal_vref_voltage == 2.048f || internal_vref_voltage == 2.5f || internal_vref_voltage == 2.90f, "Invalid internal VREF voltage, must be 2.048v, 2.5v, or 2.90v");
inline constexpr float vref_voltage = use_external_vref ? external_vref_voltage : internal_vref_voltage; // VREF voltage for ADC reference
inline constexpr float vref_half_voltage = vref_voltage / 2.0f; // VREF/2 voltage for current sense reference

// Clock config, see RM0440 7.2

inline constexpr uint32_t sysclk_hz = 170e6;    // 170 MHz max
// inline constexpr uint32_t adc_clk_hz = 42.5e6;  // 52 MHz max, must use PLL instead of ADC prescaler to avoid performance degradation, see ES0430 2.7.11
inline constexpr uint32_t adc_clk_hz = 42.5e6;  // 52 MHz max, will use sysclk / 4 as ADC clock, timer triggering must have a prescaler that is a multiple of 4 to prevent performance degradation, see ES0430 2.7.11
inline constexpr uint32_t phase_pwm_timer_prescaler = 4u; // prescaler for motor PWM timers, must be a multiple of 4 to prevent ADC performance degradation, see ES0430 2.7.11

// clock sources
inline constexpr uint32_t hse_clock_hz = 25e6;  // 25 MHz external oscillator
inline constexpr uint32_t hsi_clock_hz = 16e6;  // 16 MHz internal RC oscillator
inline constexpr uint32_t lsi_clock_hz = 32e3;  // 32 kHz internal RC oscillator (used for independent watchdog)

// HSE clock configuration (normal operation)
inline constexpr uint32_t hse_PLL_M_div = 5u;   // 25 MHz / 5 = 5 MHz PLL input frequency
inline constexpr uint32_t hse_PLL_N_mult = 68u; // 5 MHz * 68 = 340 MHz VCO frequency

// HSI clock configuration (fallback when HSE fails)
inline constexpr uint32_t hsi_PLL_M_div = 4u;   // 16 MHz / 4 = 4 MHz PLL input frequency
inline constexpr uint32_t hsi_PLL_N_mult = 85u; // 4 MHz * 85 = 340 MHz VCO frequency

// clock dividers
inline constexpr uint32_t PLL_R_div = 2u;   // 340 MHz / 2 = 170 MHz system clock frequency
inline constexpr uint32_t PLL_P_div = 8u;   // 340 MHz / 8 = 42.5 MHz ADC clock frequency   // not used
inline constexpr uint32_t AHB_clk_div = 1u;  // no AHP prescaler
inline constexpr uint32_t APB1_clk_div = 1u;  // no APB1 prescaler
inline constexpr uint32_t APB2_clk_div = 1u;  // no APB2 prescaler
inline constexpr uint32_t systick_clk_div = 1u; // SysTick clock source divider (1 or 8) (not the timer div)

inline constexpr uint32_t adc_clk_div = 4u; // ADC clock divider from system clock


// verify pll and divider settings produce expected frequencies
static_assert(hse_clock_hz / hse_PLL_M_div * hse_PLL_N_mult == 340e6, "HSE PLL settings do not produce expected VCO frequency");
static_assert(hsi_clock_hz / hsi_PLL_M_div * hsi_PLL_N_mult == 340e6, "HSI PLL settings do not produce expected VCO frequency");
static_assert(340e6 / PLL_R_div == sysclk_hz, "PLL R divider does not produce expected system clock frequency");
static_assert(340e6 / PLL_P_div == adc_clk_hz, "PLL P divider does not produce expected ADC clock frequency");
static_assert(sysclk_hz / adc_clk_div == adc_clk_hz, "ADC clock divider does not produce expected ADC clock frequency");

// Derived clock frequencies
inline constexpr uint32_t AHB_clk_hz = sysclk_hz / AHB_clk_div;
inline constexpr uint32_t APB1_peripheral_clk_hz = sysclk_hz / APB1_clk_div;
inline constexpr uint32_t APB2_peripheral_clk_hz = sysclk_hz / APB2_clk_div;
inline constexpr uint32_t APB1_timer_clk_hz = APB1_peripheral_clk_hz * (APB1_clk_div == 1u ? 1u : 2u); // Timer clocks are multiplied by 2 when APB prescaler is not 1
inline constexpr uint32_t APB2_timer_clk_hz = APB2_peripheral_clk_hz * (APB2_clk_div == 1u ? 1u : 2u); // Timer clocks are multiplied by 2 when APB prescaler is not 1
inline constexpr uint32_t systick_clk_source_hz = sysclk_hz / systick_clk_div; // SysTick clock source frequency

// Specific timer clock frequencies, see RM0440 Figure 17. Clock tree
inline constexpr uint32_t TIM1_ker_clk = APB2_timer_clk_hz;
inline constexpr uint32_t TIM8_ker_clk = APB2_timer_clk_hz;
inline constexpr uint32_t TIM20_ker_clk = APB2_timer_clk_hz;
inline constexpr uint32_t TIM15_ker_clk = APB2_timer_clk_hz;
inline constexpr uint32_t TIM16_ker_clk = APB2_timer_clk_hz;
inline constexpr uint32_t TIM17_ker_clk = APB2_timer_clk_hz;
inline constexpr uint32_t HRTIM_ker_clk = APB2_timer_clk_hz;
inline constexpr uint32_t TIM2_ker_clk = APB1_timer_clk_hz;
inline constexpr uint32_t TIM3_ker_clk = APB1_timer_clk_hz;
inline constexpr uint32_t TIM4_ker_clk = APB1_timer_clk_hz;
inline constexpr uint32_t TIM5_ker_clk = APB1_timer_clk_hz;
inline constexpr uint32_t TIM6_ker_clk = APB1_timer_clk_hz;
inline constexpr uint32_t TIM7_ker_clk = APB1_timer_clk_hz;

static_assert(adc_clk_hz == TIM1_ker_clk / phase_pwm_timer_prescaler, "Phase PWM timer prescaler must result in timer running at ADC clock frequency, see ES0430 2.7.11");

// Systick config
inline constexpr uint32_t systick_tick_hz = 1e3; // 1 kHz tick frequency for millisecond ticks
inline constexpr uint32_t systick_reload_value = systick_clk_source_hz / systick_tick_hz - 1u; // SysTick reload value for desired tick frequency
static_assert(systick_reload_value <= 0xFFFFFFu, "SysTick reload value must fit in 24 bits");
static_assert(sysclk_hz / systick_clk_div / systick_tick_hz - 1u == systick_reload_value, "SysTick reload value calculation mismatch, likely due to integer division rounding error");


// status LEDs
inline GPIO_TypeDef* const led_0_port = GPIOA;  inline constexpr uint8_t led_0_pin = 11u;   // LED on board near MCU, sig out LED_0
inline GPIO_TypeDef* const led_1_port = GPIOA;  inline constexpr uint8_t led_1_pin = 12u;   // LED on board near MCU, sig out LED_1
inline GPIO_TypeDef* const comm_port_1_led_0_port = GPIOE;  inline constexpr uint8_t comm_port_1_led_0_pin = 6u;    // Communication port 1 LED 0, sig out PORT_1_LED_0
inline GPIO_TypeDef* const comm_port_1_led_1_port = GPIOE;  inline constexpr uint8_t comm_port_1_led_1_pin = 5u;    // Communication port 1 LED 1, sig out PORT_1_LED_1
inline GPIO_TypeDef* const comm_port_2_led_0_port = GPIOC;  inline constexpr uint8_t comm_port_2_led_0_pin = 13u;   // Communication port 2 LED 0, sig out PORT_2_LED_0
inline GPIO_TypeDef* const comm_port_2_led_1_port = GPIOE;  inline constexpr uint8_t comm_port_2_led_1_pin = 4u;    // Communication port 2 LED 1, sig out PORT_2_LED_1



// ADCs

// ADC instances
inline ADC_TypeDef* const phase_u_adc = ADC1;
inline ADC_TypeDef* const phase_v_adc = ADC2;
inline ADC_TypeDef* const phase_w_adc = ADC3;

// Trigger inputs from timers for synchronizing ADC conversions to PWM, see RM0440 table 166 and 168
inline constexpr uint8_t phase_u_adc_timer_a_reg_extsel = 0b01001;  // ADC1 TIM1_TRGO
inline constexpr uint8_t phase_v_adc_timer_a_reg_extsel = 0b01001;  // ADC2 TIM1_TRGO
inline constexpr uint8_t phase_w_adc_timer_a_reg_extsel = 0b01001;  // ADC3 TIM1_TRGO

inline constexpr uint8_t phase_u_adc_timer_b_reg_extsel = 0b00111;  // ADC1 TIM8_TRGO
inline constexpr uint8_t phase_v_adc_timer_b_reg_extsel = 0b00111;  // ADC2 TIM8_TRGO
inline constexpr uint8_t phase_w_adc_timer_b_reg_extsel = 0b00111;  // ADC3 TIM8_TRGO

inline constexpr uint8_t phase_u_adc_timer_c_reg_extsel = 0b10000;  // ADC1 TIM20_TRGO
inline constexpr uint8_t phase_v_adc_timer_c_reg_extsel = 0b10000;  // ADC2 TIM20_TRGO
inline constexpr uint8_t phase_w_adc_timer_c_reg_extsel = 0b10000;  // ADC3 TIM20_TRGO

inline ADC_TypeDef* const vref_half_0 = ADC1;
inline ADC_TypeDef* const vref_half_1 = ADC2;
inline ADC_TypeDef* const vref_half_2 = ADC3;


// ADC GPIOs and channels

// Motor A phase U ADC input, sig in PHASE_I_SENSE_A_U
inline GPIO_TypeDef* const phase_a_u_adc_port = GPIOA;
inline constexpr uint8_t phase_a_u_adc_pin = 2u;
inline constexpr uint8_t phase_a_u_adc_channel = 3u;

// Motor A phase V ADC input, sig in PHASE_I_SENSE_A_V
inline GPIO_TypeDef* const phase_a_v_adc_port = GPIOA;
inline constexpr uint8_t phase_a_v_adc_pin = 6u;
inline constexpr uint8_t phase_a_v_adc_channel = 3u;

// Motor A phase W ADC input, sig in PHASE_I_SENSE_A_W
inline GPIO_TypeDef* const phase_a_w_adc_port = GPIOB;
inline constexpr uint8_t phase_a_w_adc_pin = 1u;
inline constexpr uint8_t phase_a_w_adc_channel = 1u;

// Motor B phase U ADC input, sig in PHASE_I_SENSE_B_U
inline GPIO_TypeDef* const phase_b_u_adc_port = GPIOA;
inline constexpr uint8_t phase_b_u_adc_pin = 3u;
inline constexpr uint8_t phase_b_u_adc_channel = 4u;

// Motor B phase V ADC input, sig in PHASE_I_SENSE_B_V
inline GPIO_TypeDef* const phase_b_v_adc_port = GPIOA;
inline constexpr uint8_t phase_b_v_adc_pin = 7u;
inline constexpr uint8_t phase_b_v_adc_channel = 4u;

// Motor B phase W ADC input, sig in PHASE_I_SENSE_B_W
inline GPIO_TypeDef* const phase_b_w_adc_port = GPIOE;
inline constexpr uint8_t phase_b_w_adc_pin = 7u;
inline constexpr uint8_t phase_b_w_adc_channel = 4u;

// Motor C phase U ADC input, sig in PHASE_I_SENSE_C_U
inline GPIO_TypeDef* const phase_c_u_adc_port = GPIOB;
inline constexpr uint8_t phase_c_u_adc_pin = 14u;
inline constexpr uint8_t phase_c_u_adc_channel = 5u;

// Motor C phase V ADC input, sig in PHASE_I_SENSE_C_V
inline GPIO_TypeDef* const phase_c_v_adc_port = GPIOC;
inline constexpr uint8_t phase_c_v_adc_pin = 4u;
inline constexpr uint8_t phase_c_v_adc_channel = 5u;

// Motor C phase W ADC input, sig in PHASE_I_SENSE_C_W
inline GPIO_TypeDef* const phase_c_w_adc_port = GPIOB;
inline constexpr uint8_t phase_c_w_adc_pin = 13u;
inline constexpr uint8_t phase_c_w_adc_channel = 5u;

// vref/2 for phase current sense reference, sig in VREF_HALF
inline GPIO_TypeDef* const vref_half_0_adc_port = GPIOC;
inline GPIO_TypeDef* const vref_half_1_adc_port = GPIOC;
inline GPIO_TypeDef* const vref_half_2_adc_port = GPIOD;
inline constexpr uint8_t vref_half_0_adc_pin = 1u;
inline constexpr uint8_t vref_half_1_adc_pin = 1u;
inline constexpr uint8_t vref_half_2_adc_pin = 11u;
inline constexpr uint8_t vref_half_0_adc_channel = 7u;
inline constexpr uint8_t vref_half_1_adc_channel = 7u;
inline constexpr uint8_t vref_half_2_adc_channel = 8u;


// AUX ADC inputs

// IPM thermistors
inline ADC_TypeDef* const aux_adc = ADC4;
inline GPIO_TypeDef* const inverter_a_therm_port = GPIOD;   // sig in INVERTER_A_THERM
inline constexpr uint8_t inverter_a_therm_pin = 10u;
inline constexpr uint8_t inverter_a_therm_channel = 7u;
inline GPIO_TypeDef* const inverter_b_therm_port = GPIOD;   // sig in INVERTER_B_THERM
inline constexpr uint8_t inverter_b_therm_pin = 12u;
inline constexpr uint8_t inverter_b_therm_channel = 9u;
inline GPIO_TypeDef* const inverter_c_therm_port = GPIOD;   // sig in INVERTER_C_THERM
inline constexpr uint8_t inverter_c_therm_pin = 13u;
inline constexpr uint8_t inverter_c_therm_channel = 10u;

// IPM IC temperature sensors
inline GPIO_TypeDef* const inverter_a_temp_port = GPIOE;    // sig in INVERTER_A_TEMP
inline constexpr uint8_t inverter_a_temp_pin = 15u;
inline constexpr uint8_t inverter_a_temp_channel = 2u;
inline GPIO_TypeDef* const inverter_b_temp_port = GPIOB;    // sig in INVERTER_B_TEMP
inline constexpr uint8_t inverter_b_temp_pin = 12u;
inline constexpr uint8_t inverter_b_temp_channel = 3u;
inline GPIO_TypeDef* const inverter_c_temp_port = GPIOB;    // sig in INVERTER_C_TEMP
inline constexpr uint8_t inverter_c_temp_pin = 15u;
inline constexpr uint8_t inverter_c_temp_channel = 5u;

// Gate supply voltage sense
inline GPIO_TypeDef* const gate_supply_sense_port = GPIOD;  // sig in GATE_SUPPLY_SENSE
inline constexpr uint8_t gate_supply_sense_pin = 14u;
inline constexpr uint8_t gate_supply_sense_channel = 11u;

// VBUS voltage sense
inline ADC_TypeDef* const vbus_sense_adc = ADC2;    // sig in N/A, not connected to any pin, fed from OPAMP2 internal output
inline constexpr uint8_t vbus_sense_channel = 16u;



// OPAMPs

// VREF/2 buffer for current sense reference
inline OPAMP_TypeDef* const vref_half_opamp = OPAMP5;
inline GPIO_TypeDef* const vref_half_opamp_out_port = GPIOA;    // sig out VREF_HALF
inline constexpr uint8_t vref_half_opamp_vout_pin = 8u;
inline GPIO_TypeDef* const vref_half_opamp_vinp_port = GPIOC;
inline constexpr uint8_t vref_half_opamp_vinp_pin = 3u;

// VBUS voltage sense buffer
inline OPAMP_TypeDef* const vbus_sense_opamp = OPAMP2;
// Note: OPAMP2 VOUT is internally connected to an ADC input, so no GPIO configuration is needed for the output
inline GPIO_TypeDef* const vbus_sense_opamp_vinp_port = GPIOB;  // sig in VBUS_SENSE
inline constexpr uint8_t vbus_sense_opamp_vinp_pin = 0u;



// DACs

// VCXO trim
inline DAC_TypeDef* const vcxo_trim_dac = DAC1;
inline GPIO_TypeDef* const vcxo_trim_dac_port = GPIOA;    // sig out VCXO_TRIM
inline constexpr uint8_t vcxo_trim_dac_pin = 4u;
inline constexpr uint8_t vcxo_trim_dac_channel = 1u;



// Communication

// UART
// main uart
inline USART_TypeDef* const comm_uart = USART1;
inline GPIO_TypeDef* const comm_uart_rx_port = GPIOA;   // sig in COMM_RX
inline constexpr uint8_t comm_uart_rx_pin = 10u;
inline constexpr uint8_t comm_uart_rx_alternate_function = 7u; // AF7 for USART1 on PA10, see DS12712 Table 13
inline GPIO_TypeDef* const comm_uart_tx_port = GPIOA;   // sig out COMM_TX
inline constexpr uint8_t comm_uart_tx_pin = 9u;
inline constexpr uint8_t comm_uart_tx_alternate_function = 7u; // AF7 for USART1 on PA9, see DS12712 Table 13
inline constexpr uint32_t comm_uart_timer_ker_clk = APB2_peripheral_clk_hz; // USART1 is on APB2, see RM0440 Figure 17. Clock tree
// snoop uart
inline USART_TypeDef* const comm_snoop_uart = USART3;
inline GPIO_TypeDef* const comm_snoop_uart_rx_port = GPIOD;   // sig in COMM_SNOOP_RX
inline constexpr uint8_t comm_snoop_uart_rx_pin = 9u;
// aux uart (not used currently)
inline USART_TypeDef* const comm_aux_uart = USART2;
inline GPIO_TypeDef* const comm_aux_uart_rx_port = GPIOD;   // sig in AUX_COMM_RX
inline constexpr uint8_t comm_aux_uart_rx_pin = 6u;
inline GPIO_TypeDef* const comm_aux_uart_tx_port = GPIOD;   // sig out AUX_COMM_TX
inline constexpr uint8_t comm_aux_uart_tx_pin = 5u;

// Comm GPIO
inline GPIO_TypeDef* const comm_tx_passthrough_disable_port = GPIOG; // sig out COMM_TX_PASSTHROUGH_DISABLE
inline constexpr uint8_t comm_tx_passthrough_disable_pin = 5u;
inline GPIO_TypeDef* const comm_rx_passthrough_enable_port = GPIOG; // sig out COMM_RX_PASSTHROUGH_ENABLE
inline constexpr uint8_t comm_rx_passthrough_enable_pin = 6u;
inline GPIO_TypeDef* const comm_aux_drive_enable_port = GPIOD; // sig out AUX_COMM_DRIVE_ENABLE
inline constexpr uint8_t comm_aux_drive_enable_pin = 4u;



// Timers

// VCXO trim PWM timer
inline TIM_TypeDef* const vcxo_trim_timer = TIM2;
inline GPIO_TypeDef* const vcxo_trim_timer_port = GPIOD;    // sig out
inline constexpr uint8_t vcxo_trim_timer_pin = 3u;
inline constexpr uint8_t vcxo_trim_timer_ch = 1u;

// Comm UART RX sync timer
inline TIM_TypeDef* const comm_uart_rx_sync_timer = TIM5;
inline GPIO_TypeDef* const comm_uart_rx_sync_timer_port = GPIOB;    // sig in COMM_RX
inline constexpr uint8_t comm_uart_rx_sync_timer_pin = 2u;
inline constexpr uint8_t comm_uart_rx_sync_timer_ch = 1u;

// Fan PWM timer
inline TIM_TypeDef* const fan_pwm_timer = TIM17;
inline GPIO_TypeDef* const fan_pwm_port = GPIOE;   // sig out FAN_PWM
inline constexpr uint8_t fan_pwm_pin = 1u;
inline constexpr uint8_t fan_pwm_pin_alternate_function = 4u; // AF4 for TIM17 on PE1, see DS12712 Table 13
inline constexpr uint8_t fan_pwm_ch = 1u;
inline constexpr uint32_t fan_pwm_timer_ker_clk = TIM17_ker_clk;

// Fan tachometer timer
inline TIM_TypeDef* const fan_tach_timer = TIM15;
inline GPIO_TypeDef* const fan_tach_port = GPIOF;  // sig in FAN_TACH
inline constexpr uint8_t fan_tach_pin = 9u;
inline constexpr uint8_t fan_tach_pin_alternate_function = 3u; // AF3 for TIM15 on PF9, see DS12712 Table 13
inline constexpr uint8_t fan_tach_ch = 1u;
inline constexpr uint32_t fan_tach_timer_ker_clk = TIM15_ker_clk;

// Motor A PWM timer
inline TIM_TypeDef* const phase_pwm_a_timer = TIM1;
inline GPIO_TypeDef* const phase_pwm_a_u_high_port = GPIOE; // sig out PHASE_PWM_HIGH_A_U
inline constexpr uint8_t phase_pwm_a_u_high_pin = 9u;
inline constexpr uint8_t phase_pwm_a_u_high_pin_alternate_function = 2u; // AF2 for TIM1 on PE9, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_a_u_low_port = GPIOE;  // sig out PHASE_PWM_LOW_A_U
inline constexpr uint8_t phase_pwm_a_u_low_pin = 8u;
inline constexpr uint8_t phase_pwm_a_u_low_pin_alternate_function = 2u; // AF2 for TIM1 on PE8, see DS12712 Table 13
inline constexpr uint8_t phase_pwm_a_u_ch = 1u;
inline GPIO_TypeDef* const phase_pwm_a_v_high_port = GPIOE; // sig out PHASE_PWM_HIGH_A_V
inline constexpr uint8_t phase_pwm_a_v_high_pin = 11u;
inline constexpr uint8_t phase_pwm_a_v_high_pin_alternate_function = 2u; // AF2 for TIM1 on PE11, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_a_v_low_port = GPIOE;  // sig out PHASE_PWM_LOW_A_V
inline constexpr uint8_t phase_pwm_a_v_low_pin = 10u;
inline constexpr uint8_t phase_pwm_a_v_low_pin_alternate_function = 2u; // AF2 for TIM1 on PE10, see DS12712 Table 13
inline constexpr uint8_t phase_pwm_a_v_ch = 2u;
inline GPIO_TypeDef* const phase_pwm_a_w_high_port = GPIOE; // sig out PHASE_PWM_HIGH_A_W
inline constexpr uint8_t phase_pwm_a_w_high_pin = 13u;
inline constexpr uint8_t phase_pwm_a_w_high_pin_alternate_function = 2u; // AF2 for TIM1 on PE13, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_a_w_low_port = GPIOE;  // sig out PHASE_PWM_LOW_A_W
inline constexpr uint8_t phase_pwm_a_w_low_pin = 12u;
inline constexpr uint8_t phase_pwm_a_w_low_pin_alternate_function = 2u; // AF2 for TIM1 on PE12, see DS12712 Table 13
inline constexpr uint8_t phase_pwm_a_w_ch = 3u;
inline GPIO_TypeDef* const phase_pwm_a_break_port = GPIOB;  // sig in/out TIMER_A_BREAK
inline constexpr uint8_t phase_pwm_a_break_pin = 10u;
inline constexpr uint8_t phase_pwm_a_break_pin_alternate_function = 12u; // AF12 for TIM1 on PB10, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_a_break_2_port = GPIOE;  // sig in/out TIMER_A_BREAK_2
inline constexpr uint8_t phase_pwm_a_break_2_pin = 14u;
inline constexpr uint8_t phase_pwm_a_break_2_pin_alternate_function = 6u; // AF6 for TIM1 on PE14, see DS12712 Table 13

// Motor B PWM timer
inline TIM_TypeDef* const phase_pwm_b_timer = TIM8;
inline GPIO_TypeDef* const phase_pwm_b_u_high_port = GPIOA;
inline constexpr uint8_t phase_pwm_b_u_high_pin = 15u;
inline constexpr uint8_t phase_pwm_b_u_high_pin_alternate_function = 2u; // AF2 for TIM8 on PA15, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_b_u_low_port = GPIOC;
inline constexpr uint8_t phase_pwm_b_u_low_pin = 10u;
inline constexpr uint8_t phase_pwm_b_u_low_pin_alternate_function = 4u; // AF4 for TIM8 on PC10, see DS12712 Table 13
inline constexpr uint8_t phase_pwm_b_u_ch = 1u;
inline GPIO_TypeDef* const phase_pwm_b_v_high_port = GPIOC;
inline constexpr uint8_t phase_pwm_b_v_high_pin = 7u;
inline constexpr uint8_t phase_pwm_b_v_high_pin_alternate_function = 4u; // AF4 for TIM8 on PC7, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_b_v_low_port = GPIOC;
inline constexpr uint8_t phase_pwm_b_v_low_pin = 11u;
inline constexpr uint8_t phase_pwm_b_v_low_pin_alternate_function = 4u; // AF4 for TIM8 on PC11, see DS12712 Table 13
inline constexpr uint8_t phase_pwm_b_v_ch = 2u;
inline GPIO_TypeDef* const phase_pwm_b_w_high_port = GPIOB;
inline constexpr uint8_t phase_pwm_b_w_high_pin = 9u;
inline constexpr uint8_t phase_pwm_b_w_high_pin_alternate_function = 10u; // AF10 for TIM8 on PB9, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_b_w_low_port = GPIOB;
inline constexpr uint8_t phase_pwm_b_w_low_pin = 5u;
inline constexpr uint8_t phase_pwm_b_w_low_pin_alternate_function = 3u; // AF3 for TIM8 on PB5, see DS12712 Table 13
inline constexpr uint8_t phase_pwm_b_w_ch = 3u;
inline GPIO_TypeDef* const phase_pwm_b_break_port = GPIOB;  // sig in/out TIMER_B_BREAK
inline constexpr uint8_t phase_pwm_b_break_pin = 7u;
inline constexpr uint8_t phase_pwm_b_break_pin_alternate_function = 5u; // AF5 for TIM8 on PB7, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_b_break_2_port = GPIOB;  // sig in/out TIMER_B_BREAK_2
inline constexpr uint8_t phase_pwm_b_break_2_pin = 6u;
inline constexpr uint8_t phase_pwm_b_break_2_pin_alternate_function = 10u; // AF10 for TIM8 on PB6, see DS12712 Table 13

// Motor C PWM timer
inline TIM_TypeDef* const phase_pwm_c_timer = TIM20;
inline GPIO_TypeDef* const phase_pwm_c_u_high_port = GPIOE;
inline constexpr uint8_t phase_pwm_c_u_high_pin = 2u;
inline constexpr uint8_t phase_pwm_c_u_high_pin_alternate_function = 6u; // AF6 for TIM20 on PE2, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_c_u_low_port = GPIOG;
inline constexpr uint8_t phase_pwm_c_u_low_pin = 0u;
inline constexpr uint8_t phase_pwm_c_u_low_pin_alternate_function = 2u; // AF2 for TIM20 on PG0, see DS12712 Table 13
inline constexpr uint8_t phase_pwm_c_u_ch = 1u;
inline GPIO_TypeDef* const phase_pwm_c_v_high_port = GPIOE;
inline constexpr uint8_t phase_pwm_c_v_high_pin = 3u;
inline constexpr uint8_t phase_pwm_c_v_high_pin_alternate_function = 6u; // AF6 for TIM20 on PE3, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_c_v_low_port = GPIOG;
inline constexpr uint8_t phase_pwm_c_v_low_pin = 1u;
inline constexpr uint8_t phase_pwm_c_v_low_pin_alternate_function = 2u; // AF2 for TIM20 on PG1, see DS12712 Table 13
inline constexpr uint8_t phase_pwm_c_v_ch = 2u;
inline GPIO_TypeDef* const phase_pwm_c_w_high_port = GPIOC;
inline constexpr uint8_t phase_pwm_c_w_high_pin = 8u;
inline constexpr uint8_t phase_pwm_c_w_high_pin_alternate_function = 6u; // AF6 for TIM20 on PC8, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_c_w_low_port = GPIOG;
inline constexpr uint8_t phase_pwm_c_w_low_pin = 2u;
inline constexpr uint8_t phase_pwm_c_w_low_pin_alternate_function = 2u; // AF2 for TIM20 on PG2, see DS12712 Table 13
inline constexpr uint8_t phase_pwm_c_w_ch = 3u;
inline GPIO_TypeDef* const phase_pwm_c_break_port = GPIOG;  // sig in/out TIMER_C_BREAK
inline constexpr uint8_t phase_pwm_c_break_pin = 3u;
inline constexpr uint8_t phase_pwm_c_break_pin_alternate_function = 2u; // AF2 for TIM20 on PG3, see DS12712 Table 13
inline GPIO_TypeDef* const phase_pwm_c_break_2_port = GPIOG;  // sig in/out TIMER_C_BREAK_2
inline constexpr uint8_t phase_pwm_c_break_2_pin = 4u;
inline constexpr uint8_t phase_pwm_c_break_2_pin_alternate_function = 2u; // AF2 for TIM20 on PG4, see DS12712 Table 13

// Motor timer internal trigger (tim_itr) input for syncing timers together, see RM0440 table 267
inline constexpr uint16_t phase_pwm_a_timer_itr = 0b01000;    // TIM1_ITR4 = TIM5_TRGO
inline constexpr uint16_t phase_pwm_b_timer_itr = 0b01000;    // TIM8_ITR4 = TIM5_TRGO
inline constexpr uint16_t phase_pwm_c_timer_itr = 0b01000;    // TIM20_ITR4 = TIM5_TRGO
    
// Aux GPIO

// Inverter disable, sig out INVERTER_DISABLE
inline GPIO_TypeDef* const inverter_disable_port = GPIOC;
inline constexpr uint8_t inverter_disable_pin = 9u;

// Inverter A fault, sig in INVERTER_A_FAULT
inline GPIO_TypeDef* const inverter_a_fault_port = GPIOF;
inline constexpr uint8_t inverter_a_fault_pin = 11u;

// Inverter B fault, sig in INVERTER_B_FAULT
inline GPIO_TypeDef* const inverter_b_fault_port = GPIOF;
inline constexpr uint8_t inverter_b_fault_pin = 12u;

// Inverter C fault, sig in INVERTER_C_FAULT
inline GPIO_TypeDef* const inverter_c_fault_port = GPIOF;
inline constexpr uint8_t inverter_c_fault_pin = 13u;

// STO fbk (from drive), sig out STO_FBK
inline GPIO_TypeDef* const sto_fbk_port = GPIOF;
inline constexpr uint8_t sto_fbk_pin = 6u;

struct IPM_Config {
    float max_phase_current_amps;
    float min_gate_supply_voltage;
    float max_gate_supply_voltage;
    float max_vbus_voltage;
    float min_ic_temp_celsius;
    float max_ic_temp_celsius;
    float min_thermistor_temp_celsius;
    float max_thermistor_temp_celsius;
    bool enable_thermistor_checks;
    uint16_t deadtime_ns;
    uint16_t min_pulse_width_ns;
    uint32_t phase_max_pwm_frequency_hz;
    uint32_t phase_min_pwm_frequency_hz;
};

inline constexpr IPM_Config NFAM2065L4B = { // values from datasheet
    .max_phase_current_amps = 40.0f,
    .min_gate_supply_voltage = 14.0f,
    .max_gate_supply_voltage = 16.0f,
    .max_vbus_voltage = 400.0f,
    .min_ic_temp_celsius = -40.0f,
    .max_ic_temp_celsius = 150.0f,
    .min_thermistor_temp_celsius = -40.0f,
    .max_thermistor_temp_celsius = 125.0f,
    .enable_thermistor_checks = false,
    .deadtime_ns = 1500,
    .min_pulse_width_ns = 1500,
    .phase_max_pwm_frequency_hz = 20000,
    .phase_min_pwm_frequency_hz = 5000,
};

inline constexpr IPM_Config ipms = NFAM2065L4B; // select the IPMs installed on the board

inline constexpr uint32_t phase_max_pwm_frequency_hz = ipms.phase_max_pwm_frequency_hz;
inline constexpr uint32_t phase_min_pwm_frequency_hz = ipms.phase_min_pwm_frequency_hz;
static_assert(phase_min_pwm_frequency_hz > TIM1_ker_clk / phase_pwm_timer_prescaler / 0xFFFF / 2, "Phase PWM timer frequency too low for 16-bit timer");

inline constexpr uint16_t phase_pwm_timer_deadtime_ns = ipms.deadtime_ns;
inline constexpr uint16_t phase_pwm_timer_deadtime_ticks =
    static_cast<uint16_t>((static_cast<uint64_t>(phase_pwm_timer_deadtime_ns) * TIM1_ker_clk) / 1000000000ULL);

inline constexpr uint32_t phase_pwm_timer_min_low_ticks = 1u; // minimum low time for PWM timer to allow sampling and gate drive, in timer ticks
inline constexpr uint32_t phase_pwm_timer_max_high_ticks = 1u; // maximum high time for PWM timer before gate drive voltage drops, in timer ticks
inline constexpr uint16_t phase_pwm_min_usable_ticks = 
    static_cast<uint16_t>((static_cast<uint64_t>(ipms.min_pulse_width_ns) * TIM1_ker_clk) / 1000000000ULL);

} // namespace board_hw
