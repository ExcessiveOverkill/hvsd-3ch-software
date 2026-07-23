#pragma once

#include "stm32g473xx.h"

#include "board_hw.h"
#include "gpio_ll.h"
#include "time.h"

#include "aura.hpp"
#include "reg_shell.hpp"

#include "motor_channel.h"
#include "cordic.h"

#include "fan.h"
#include "communication.h"
#include "leds.h"

#include "mode.h"


class device {
public:
    device() = default;

    void init();
    void run();

private:
    // C-style shell callback trampoline to the active device instance.
    static device* active_instance;
    static void shell_putc(char c);

    // ---- Internal init helpers --------------------------------------------
    void cpu_init();
    void safety_init();
    void systick_init();
    void opamps_init();
    void shell_init();

    void start_pwm_timers_at_sync_cnt(uint32_t cnt);
    void start_pwm_timers_now();

    static time_interface time;

    static cordic cordic_drv;

    static Messaging msg;
    RegShell shell;

    fan_interface fan;
    communication_interface comm;
    led_interface leds;

    static motor_channel motor_channels[3];

    mode* channel_current_mode_instance[3] = {
        nullptr,
        nullptr,
        nullptr
    };

    struct mode_options {
        mode none;
    };

    static mode_options channel_modes[3];

    // ---- Change PWM frequency for all channels --------------------------------------
    bool force_restart_pwm_timers(float frequency_hz);    
    bool change_pwm_frequency(float frequency_hz);
    struct pwm_frequency{
        float hz_float;
        uint32_t hz_int;
        float period_s;
        uint32_t period_timer_ticks;
        uint32_t period_us;
    } pwm_freq;

    // ---- IRQ flags (set in handler, cleared in flagged_ method) -----------
    volatile bool systick_flag = false;
    bool          systick_missed = false;
    void flagged_systick();

    volatile bool tim1_update_flag    = false;
    bool          tim1_update_missed  = false;
    void flagged_tim1_update();

    volatile bool tim8_update_flag    = false;
    bool          tim8_update_missed  = false;
    void flagged_tim8_update();

    volatile bool tim20_update_flag    = false;
    bool          tim20_update_missed  = false;
    void flagged_tim20_update();

    volatile bool adc1_eoc_flag = false;
    bool          adc1_eoc_missed = false;
    void flagged_adc1_eoc();

    volatile bool adc2_eoc_flag = false;
    bool          adc2_eoc_missed = false;
    void flagged_adc2_eoc();

    volatile bool adc2_injected_eoc_flag = false;
    bool          adc2_injected_eoc_missed = false;
    void flagged_adc2_injected_eoc();

    volatile bool adc3_eoc_flag = false;
    bool          adc3_eoc_missed = false;
    void flagged_adc3_eoc();

    volatile bool adc4_eoc_flag = false;
    bool          adc4_eoc_missed = false;
    void flagged_adc4_eoc();

    // ---- Missed-IRQ tracking ----------------------------------------------
    enum class IRQ : int16_t {
        NONE = -1,
        WWDG_IRQHandler = 0,
        PVD_PVM_IRQHandler,
        RTC_TAMP_LSECSS_IRQHandler,
        RTC_WKUP_IRQHandler,
        FLASH_IRQHandler,
        RCC_IRQHandler,
        EXTI0_IRQHandler,
        EXTI1_IRQHandler,
        EXTI2_IRQHandler,
        EXTI3_IRQHandler,
        EXTI4_IRQHandler,
        DMA1_Channel1_IRQHandler,
        DMA1_Channel2_IRQHandler,
        DMA1_Channel3_IRQHandler,
        DMA1_Channel4_IRQHandler,
        DMA1_Channel5_IRQHandler,
        DMA1_Channel6_IRQHandler,
        DMA1_Channel7_IRQHandler,
        DMA1_Channel8_IRQHandler,
        ADC1_2_IRQHandler,
        USB_HP_IRQHandler,
        USB_LP_IRQHandler,
        FDCAN1_IT0_IRQHandler,
        FDCAN1_IT1_IRQHandler,
        EXTI9_5_IRQHandler,
        TIM1_BRK_TIM15_IRQHandler,
        TIM1_UP_TIM16_IRQHandler,
        TIM1_TRG_COM_TIM17_IRQHandler,
        TIM1_CC_IRQHandler,
        TIM2_IRQHandler,
        TIM3_IRQHandler,
        TIM4_IRQHandler,
        I2C1_EV_IRQHandler,
        I2C1_ER_IRQHandler,
        I2C2_EV_IRQHandler,
        I2C2_ER_IRQHandler,
        SPI1_IRQHandler,
        SPI2_IRQHandler,
        USART1_IRQHandler,
        USART2_IRQHandler,
        USART3_IRQHandler,
        EXTI15_10_IRQHandler,
        RTC_Alarm_IRQHandler,
        USBWakeUp_IRQHandler,
        TIM8_BRK_IRQHandler,
        TIM8_UP_IRQHandler,
        TIM8_TRG_COM_IRQHandler,
        TIM8_CC_IRQHandler,
        ADC3_IRQHandler,
        FMC_IRQHandler,
        LPTIM1_IRQHandler,
        TIM5_IRQHandler,
        SPI3_IRQHandler,
        UART4_IRQHandler,
        UART5_IRQHandler,
        TIM6_DAC_IRQHandler,
        TIM7_DAC_IRQHandler,
        DMA2_Channel1_IRQHandler,
        DMA2_Channel2_IRQHandler,
        DMA2_Channel3_IRQHandler,
        DMA2_Channel4_IRQHandler,
        DMA2_Channel5_IRQHandler,
        DMA2_Channel6_IRQHandler,
        DMA2_Channel7_IRQHandler,
        DMA2_Channel8_IRQHandler,
        ADC4_IRQHandler,
        ADC5_IRQHandler,
        UCPD1_IRQHandler,
        COMP1_2_3_IRQHandler,
        COMP4_5_6_IRQHandler,
        COMP7_IRQHandler,
        CRS_IRQHandler,
        SAI1_IRQHandler,
        TIM20_BRK_IRQHandler,
        TIM20_UP_IRQHandler,
        TIM20_TRG_COM_IRQHandler,
        TIM20_CC_IRQHandler,
        FPU_IRQHandler,
        I2C4_EV_IRQHandler,
        I2C4_ER_IRQHandler,
        SPI4_IRQHandler,
        FDCAN2_IT0_IRQHandler,
        FDCAN2_IT1_IRQHandler,
        FDCAN3_IT0_IRQHandler,
        FDCAN3_IT1_IRQHandler,
        RNG_IRQHandler,
        LPUART1_IRQHandler,
        I2C3_EV_IRQHandler,
        I2C3_ER_IRQHandler,
        DMAMUX_OVR_IRQHandler,
        QUADSPI_IRQHandler,
        CORDIC_IRQHandler,
    };

    static IRQ missed_irq;
    static void missed_irq_handler(IRQ irq);

public:
    // ---- Cortex-M4 core exception handlers --------------------------------
    void NMI_Handler();
    void HardFault_Handler();
    void MemManage_Handler();
    void BusFault_Handler();
    void UsageFault_Handler();
    void SVC_Handler();
    void DebugMon_Handler();
    void PendSV_Handler();
    void SysTick_Handler();

    // ---- STM32G473 peripheral IRQ handlers --------------------------------
    void WWDG_IRQHandler();
    void PVD_PVM_IRQHandler();
    void RTC_TAMP_LSECSS_IRQHandler();
    void RTC_WKUP_IRQHandler();
    void FLASH_IRQHandler();
    void RCC_IRQHandler();
    void EXTI0_IRQHandler();
    void EXTI1_IRQHandler();
    void EXTI2_IRQHandler();
    void EXTI3_IRQHandler();
    void EXTI4_IRQHandler();
    void DMA1_Channel1_IRQHandler();
    void DMA1_Channel2_IRQHandler();
    void DMA1_Channel3_IRQHandler();
    void DMA1_Channel4_IRQHandler();
    void DMA1_Channel5_IRQHandler();
    void DMA1_Channel6_IRQHandler();
    void DMA1_Channel7_IRQHandler();
    void DMA1_Channel8_IRQHandler();
    void ADC1_2_IRQHandler();
    void USB_HP_IRQHandler();
    void USB_LP_IRQHandler();
    void FDCAN1_IT0_IRQHandler();
    void FDCAN1_IT1_IRQHandler();
    void EXTI9_5_IRQHandler();
    void TIM1_BRK_TIM15_IRQHandler();
    void TIM1_UP_TIM16_IRQHandler();
    void TIM1_TRG_COM_TIM17_IRQHandler();
    void TIM1_CC_IRQHandler();
    void TIM2_IRQHandler();
    void TIM3_IRQHandler();
    void TIM4_IRQHandler();
    void I2C1_EV_IRQHandler();
    void I2C1_ER_IRQHandler();
    void I2C2_EV_IRQHandler();
    void I2C2_ER_IRQHandler();
    void SPI1_IRQHandler();
    void SPI2_IRQHandler();
    void USART1_IRQHandler();
    void USART2_IRQHandler();
    void USART3_IRQHandler();
    void EXTI15_10_IRQHandler();
    void RTC_Alarm_IRQHandler();
    void USBWakeUp_IRQHandler();
    void TIM8_BRK_IRQHandler();
    void TIM8_UP_IRQHandler();
    void TIM8_TRG_COM_IRQHandler();
    void TIM8_CC_IRQHandler();
    void ADC3_IRQHandler();
    void FMC_IRQHandler();
    void LPTIM1_IRQHandler();
    void TIM5_IRQHandler();
    void SPI3_IRQHandler();
    void UART4_IRQHandler();
    void UART5_IRQHandler();
    void TIM6_DAC_IRQHandler();
    void TIM7_DAC_IRQHandler();
    void DMA2_Channel1_IRQHandler();
    void DMA2_Channel2_IRQHandler();
    void DMA2_Channel3_IRQHandler();
    void DMA2_Channel4_IRQHandler();
    void DMA2_Channel5_IRQHandler();
    void DMA2_Channel6_IRQHandler();
    void DMA2_Channel7_IRQHandler();
    void DMA2_Channel8_IRQHandler();
    void ADC4_IRQHandler();
    void ADC5_IRQHandler();
    void UCPD1_IRQHandler();
    void COMP1_2_3_IRQHandler();
    void COMP4_5_6_IRQHandler();
    void COMP7_IRQHandler();
    void CRS_IRQHandler();
    void SAI1_IRQHandler();
    void TIM20_BRK_IRQHandler();
    void TIM20_UP_IRQHandler();
    void TIM20_TRG_COM_IRQHandler();
    void TIM20_CC_IRQHandler();
    void FPU_IRQHandler();
    void I2C4_EV_IRQHandler();
    void I2C4_ER_IRQHandler();
    void SPI4_IRQHandler();
    void FDCAN2_IT0_IRQHandler();
    void FDCAN2_IT1_IRQHandler();
    void FDCAN3_IT0_IRQHandler();
    void FDCAN3_IT1_IRQHandler();
    void RNG_IRQHandler();
    void LPUART1_IRQHandler();
    void I2C3_EV_IRQHandler();
    void I2C3_ER_IRQHandler();
    void DMAMUX_OVR_IRQHandler();
    void QUADSPI_IRQHandler();
    void CORDIC_IRQHandler();
};

/*
IRQ priorities:

highest
0:
1:
2:  IPM fault EXTI
3:
4:
5:
6:  phase ADC end of conversion
7:  phase PWM timer UEV, aux ADC end of conversion
8:
9:
10:
11:
12:
13:
14:
15: systick
lowest
*/