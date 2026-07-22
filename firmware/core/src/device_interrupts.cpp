#include "device.h"

// Default (weak) implementations for all STM32G473 peripheral IRQ handlers.
// Override by providing a non-weak definition in device.cpp.
// Active handlers (TIM1_UP, SysTick, etc.) are defined in device.cpp directly.

__attribute__((weak)) void device::NMI_Handler()                   { missed_irq_handler(IRQ::NONE); }
__attribute__((weak)) void device::HardFault_Handler()             { while(1) {} }
__attribute__((weak)) void device::MemManage_Handler()             { while(1) {} }
__attribute__((weak)) void device::BusFault_Handler()              { while(1) {} }
__attribute__((weak)) void device::UsageFault_Handler()            { while(1) {} }
__attribute__((weak)) void device::SVC_Handler()                   {}
__attribute__((weak)) void device::DebugMon_Handler()              {}
__attribute__((weak)) void device::PendSV_Handler()                {}

__attribute__((weak)) void device::WWDG_IRQHandler()               { missed_irq_handler(IRQ::WWDG_IRQHandler); }
__attribute__((weak)) void device::PVD_PVM_IRQHandler()            { missed_irq_handler(IRQ::PVD_PVM_IRQHandler); }
__attribute__((weak)) void device::RTC_TAMP_LSECSS_IRQHandler()   { missed_irq_handler(IRQ::RTC_TAMP_LSECSS_IRQHandler); }
__attribute__((weak)) void device::RTC_WKUP_IRQHandler()           { missed_irq_handler(IRQ::RTC_WKUP_IRQHandler); }
__attribute__((weak)) void device::FLASH_IRQHandler()              { missed_irq_handler(IRQ::FLASH_IRQHandler); }
__attribute__((weak)) void device::RCC_IRQHandler()                { missed_irq_handler(IRQ::RCC_IRQHandler); }
__attribute__((weak)) void device::EXTI0_IRQHandler()              { missed_irq_handler(IRQ::EXTI0_IRQHandler); }
__attribute__((weak)) void device::EXTI1_IRQHandler()              { missed_irq_handler(IRQ::EXTI1_IRQHandler); }
__attribute__((weak)) void device::EXTI2_IRQHandler()              { missed_irq_handler(IRQ::EXTI2_IRQHandler); }
__attribute__((weak)) void device::EXTI3_IRQHandler()              { missed_irq_handler(IRQ::EXTI3_IRQHandler); }
__attribute__((weak)) void device::EXTI4_IRQHandler()              { missed_irq_handler(IRQ::EXTI4_IRQHandler); }
__attribute__((weak)) void device::DMA1_Channel1_IRQHandler()      { missed_irq_handler(IRQ::DMA1_Channel1_IRQHandler); }
__attribute__((weak)) void device::DMA1_Channel2_IRQHandler()      { missed_irq_handler(IRQ::DMA1_Channel2_IRQHandler); }
__attribute__((weak)) void device::DMA1_Channel3_IRQHandler()      { missed_irq_handler(IRQ::DMA1_Channel3_IRQHandler); }
__attribute__((weak)) void device::DMA1_Channel4_IRQHandler()      { missed_irq_handler(IRQ::DMA1_Channel4_IRQHandler); }
__attribute__((weak)) void device::DMA1_Channel5_IRQHandler()      { missed_irq_handler(IRQ::DMA1_Channel5_IRQHandler); }
__attribute__((weak)) void device::DMA1_Channel6_IRQHandler()      { missed_irq_handler(IRQ::DMA1_Channel6_IRQHandler); }
__attribute__((weak)) void device::DMA1_Channel7_IRQHandler()      { missed_irq_handler(IRQ::DMA1_Channel7_IRQHandler); }
__attribute__((weak)) void device::DMA1_Channel8_IRQHandler()      { missed_irq_handler(IRQ::DMA1_Channel8_IRQHandler); }
__attribute__((weak)) void device::ADC1_2_IRQHandler()             { missed_irq_handler(IRQ::ADC1_2_IRQHandler); }
__attribute__((weak)) void device::USB_HP_IRQHandler()             { missed_irq_handler(IRQ::USB_HP_IRQHandler); }
__attribute__((weak)) void device::USB_LP_IRQHandler()             { missed_irq_handler(IRQ::USB_LP_IRQHandler); }
__attribute__((weak)) void device::FDCAN1_IT0_IRQHandler()         { missed_irq_handler(IRQ::FDCAN1_IT0_IRQHandler); }
__attribute__((weak)) void device::FDCAN1_IT1_IRQHandler()         { missed_irq_handler(IRQ::FDCAN1_IT1_IRQHandler); }
__attribute__((weak)) void device::EXTI9_5_IRQHandler()            { missed_irq_handler(IRQ::EXTI9_5_IRQHandler); }
__attribute__((weak)) void device::TIM1_BRK_TIM15_IRQHandler()     { missed_irq_handler(IRQ::TIM1_BRK_TIM15_IRQHandler); }
// TIM1_UP_TIM16_IRQHandler is defined (non-weak) in device.cpp
__attribute__((weak)) void device::TIM1_TRG_COM_TIM17_IRQHandler() { missed_irq_handler(IRQ::TIM1_TRG_COM_TIM17_IRQHandler); }
__attribute__((weak)) void device::TIM1_CC_IRQHandler()            { missed_irq_handler(IRQ::TIM1_CC_IRQHandler); }
__attribute__((weak)) void device::TIM2_IRQHandler()               { missed_irq_handler(IRQ::TIM2_IRQHandler); }
__attribute__((weak)) void device::TIM3_IRQHandler()               { missed_irq_handler(IRQ::TIM3_IRQHandler); }
__attribute__((weak)) void device::TIM4_IRQHandler()               { missed_irq_handler(IRQ::TIM4_IRQHandler); }
__attribute__((weak)) void device::I2C1_EV_IRQHandler()            { missed_irq_handler(IRQ::I2C1_EV_IRQHandler); }
__attribute__((weak)) void device::I2C1_ER_IRQHandler()            { missed_irq_handler(IRQ::I2C1_ER_IRQHandler); }
__attribute__((weak)) void device::I2C2_EV_IRQHandler()            { missed_irq_handler(IRQ::I2C2_EV_IRQHandler); }
__attribute__((weak)) void device::I2C2_ER_IRQHandler()            { missed_irq_handler(IRQ::I2C2_ER_IRQHandler); }
__attribute__((weak)) void device::SPI1_IRQHandler()               { missed_irq_handler(IRQ::SPI1_IRQHandler); }
__attribute__((weak)) void device::SPI2_IRQHandler()               { missed_irq_handler(IRQ::SPI2_IRQHandler); }
__attribute__((weak)) void device::USART1_IRQHandler()             { missed_irq_handler(IRQ::USART1_IRQHandler); }
__attribute__((weak)) void device::USART2_IRQHandler()             { missed_irq_handler(IRQ::USART2_IRQHandler); }
__attribute__((weak)) void device::USART3_IRQHandler()             { missed_irq_handler(IRQ::USART3_IRQHandler); }
__attribute__((weak)) void device::EXTI15_10_IRQHandler()          { missed_irq_handler(IRQ::EXTI15_10_IRQHandler); }
__attribute__((weak)) void device::RTC_Alarm_IRQHandler()          { missed_irq_handler(IRQ::RTC_Alarm_IRQHandler); }
__attribute__((weak)) void device::USBWakeUp_IRQHandler()          { missed_irq_handler(IRQ::USBWakeUp_IRQHandler); }
__attribute__((weak)) void device::TIM8_BRK_IRQHandler()           { missed_irq_handler(IRQ::TIM8_BRK_IRQHandler); }
__attribute__((weak)) void device::TIM8_UP_IRQHandler()            { missed_irq_handler(IRQ::TIM8_UP_IRQHandler); }
__attribute__((weak)) void device::TIM8_TRG_COM_IRQHandler()       { missed_irq_handler(IRQ::TIM8_TRG_COM_IRQHandler); }
__attribute__((weak)) void device::TIM8_CC_IRQHandler()            { missed_irq_handler(IRQ::TIM8_CC_IRQHandler); }
__attribute__((weak)) void device::ADC3_IRQHandler()               { missed_irq_handler(IRQ::ADC3_IRQHandler); }
__attribute__((weak)) void device::FMC_IRQHandler()                { missed_irq_handler(IRQ::FMC_IRQHandler); }
__attribute__((weak)) void device::LPTIM1_IRQHandler()             { missed_irq_handler(IRQ::LPTIM1_IRQHandler); }
__attribute__((weak)) void device::TIM5_IRQHandler()               { missed_irq_handler(IRQ::TIM5_IRQHandler); }
__attribute__((weak)) void device::SPI3_IRQHandler()               { missed_irq_handler(IRQ::SPI3_IRQHandler); }
__attribute__((weak)) void device::UART4_IRQHandler()              { missed_irq_handler(IRQ::UART4_IRQHandler); }
__attribute__((weak)) void device::UART5_IRQHandler()              { missed_irq_handler(IRQ::UART5_IRQHandler); }
__attribute__((weak)) void device::TIM6_DAC_IRQHandler()           { missed_irq_handler(IRQ::TIM6_DAC_IRQHandler); }
__attribute__((weak)) void device::TIM7_DAC_IRQHandler()           { missed_irq_handler(IRQ::TIM7_DAC_IRQHandler); }
__attribute__((weak)) void device::DMA2_Channel1_IRQHandler()      { missed_irq_handler(IRQ::DMA2_Channel1_IRQHandler); }
__attribute__((weak)) void device::DMA2_Channel2_IRQHandler()      { missed_irq_handler(IRQ::DMA2_Channel2_IRQHandler); }
__attribute__((weak)) void device::DMA2_Channel3_IRQHandler()      { missed_irq_handler(IRQ::DMA2_Channel3_IRQHandler); }
__attribute__((weak)) void device::DMA2_Channel4_IRQHandler()      { missed_irq_handler(IRQ::DMA2_Channel4_IRQHandler); }
__attribute__((weak)) void device::DMA2_Channel5_IRQHandler()      { missed_irq_handler(IRQ::DMA2_Channel5_IRQHandler); }
__attribute__((weak)) void device::DMA2_Channel6_IRQHandler()      { missed_irq_handler(IRQ::DMA2_Channel6_IRQHandler); }
__attribute__((weak)) void device::DMA2_Channel7_IRQHandler()      { missed_irq_handler(IRQ::DMA2_Channel7_IRQHandler); }
__attribute__((weak)) void device::DMA2_Channel8_IRQHandler()      { missed_irq_handler(IRQ::DMA2_Channel8_IRQHandler); }
__attribute__((weak)) void device::ADC4_IRQHandler()               { missed_irq_handler(IRQ::ADC4_IRQHandler); }
__attribute__((weak)) void device::ADC5_IRQHandler()               { missed_irq_handler(IRQ::ADC5_IRQHandler); }
__attribute__((weak)) void device::UCPD1_IRQHandler()              { missed_irq_handler(IRQ::UCPD1_IRQHandler); }
__attribute__((weak)) void device::COMP1_2_3_IRQHandler()          { missed_irq_handler(IRQ::COMP1_2_3_IRQHandler); }
__attribute__((weak)) void device::COMP4_5_6_IRQHandler()          { missed_irq_handler(IRQ::COMP4_5_6_IRQHandler); }
__attribute__((weak)) void device::COMP7_IRQHandler()              { missed_irq_handler(IRQ::COMP7_IRQHandler); }
__attribute__((weak)) void device::CRS_IRQHandler()                { missed_irq_handler(IRQ::CRS_IRQHandler); }
__attribute__((weak)) void device::SAI1_IRQHandler()               { missed_irq_handler(IRQ::SAI1_IRQHandler); }
__attribute__((weak)) void device::TIM20_BRK_IRQHandler()          { missed_irq_handler(IRQ::TIM20_BRK_IRQHandler); }
__attribute__((weak)) void device::TIM20_UP_IRQHandler()           { missed_irq_handler(IRQ::TIM20_UP_IRQHandler); }
__attribute__((weak)) void device::TIM20_TRG_COM_IRQHandler()      { missed_irq_handler(IRQ::TIM20_TRG_COM_IRQHandler); }
__attribute__((weak)) void device::TIM20_CC_IRQHandler()           { missed_irq_handler(IRQ::TIM20_CC_IRQHandler); }
__attribute__((weak)) void device::FPU_IRQHandler()                { missed_irq_handler(IRQ::FPU_IRQHandler); }
__attribute__((weak)) void device::I2C4_EV_IRQHandler()            { missed_irq_handler(IRQ::I2C4_EV_IRQHandler); }
__attribute__((weak)) void device::I2C4_ER_IRQHandler()            { missed_irq_handler(IRQ::I2C4_ER_IRQHandler); }
__attribute__((weak)) void device::SPI4_IRQHandler()               { missed_irq_handler(IRQ::SPI4_IRQHandler); }
__attribute__((weak)) void device::FDCAN2_IT0_IRQHandler()         { missed_irq_handler(IRQ::FDCAN2_IT0_IRQHandler); }
__attribute__((weak)) void device::FDCAN2_IT1_IRQHandler()         { missed_irq_handler(IRQ::FDCAN2_IT1_IRQHandler); }
__attribute__((weak)) void device::FDCAN3_IT0_IRQHandler()         { missed_irq_handler(IRQ::FDCAN3_IT0_IRQHandler); }
__attribute__((weak)) void device::FDCAN3_IT1_IRQHandler()         { missed_irq_handler(IRQ::FDCAN3_IT1_IRQHandler); }
__attribute__((weak)) void device::RNG_IRQHandler()                { missed_irq_handler(IRQ::RNG_IRQHandler); }
__attribute__((weak)) void device::LPUART1_IRQHandler()            { missed_irq_handler(IRQ::LPUART1_IRQHandler); }
__attribute__((weak)) void device::I2C3_EV_IRQHandler()            { missed_irq_handler(IRQ::I2C3_EV_IRQHandler); }
__attribute__((weak)) void device::I2C3_ER_IRQHandler()            { missed_irq_handler(IRQ::I2C3_ER_IRQHandler); }
__attribute__((weak)) void device::DMAMUX_OVR_IRQHandler()         { missed_irq_handler(IRQ::DMAMUX_OVR_IRQHandler); }
__attribute__((weak)) void device::QUADSPI_IRQHandler()            { missed_irq_handler(IRQ::QUADSPI_IRQHandler); }
__attribute__((weak)) void device::CORDIC_IRQHandler()             { missed_irq_handler(IRQ::CORDIC_IRQHandler); }
