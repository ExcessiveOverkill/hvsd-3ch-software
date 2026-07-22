/* interrupt_catch.h
 * Routes C-linkage interrupt handler names (from the vector table) to the
 * device class instance. Include this file exactly ONCE, after the Device
 * pointer is defined in main.cpp.
 */

extern "C" {

void NMI_Handler()                        { Device->NMI_Handler(); }
void HardFault_Handler()                  { Device->HardFault_Handler(); }
void MemManage_Handler()                  { Device->MemManage_Handler(); }
void BusFault_Handler()                   { Device->BusFault_Handler(); }
void UsageFault_Handler()                 { Device->UsageFault_Handler(); }
void SVC_Handler()                        { Device->SVC_Handler(); }
void DebugMon_Handler()                   { Device->DebugMon_Handler(); }
void PendSV_Handler()                     { Device->PendSV_Handler(); }
void SysTick_Handler()                    { Device->SysTick_Handler(); }

void WWDG_IRQHandler()                    { Device->WWDG_IRQHandler(); }
void PVD_PVM_IRQHandler()                 { Device->PVD_PVM_IRQHandler(); }
void RTC_TAMP_LSECSS_IRQHandler()         { Device->RTC_TAMP_LSECSS_IRQHandler(); }
void RTC_WKUP_IRQHandler()                { Device->RTC_WKUP_IRQHandler(); }
void FLASH_IRQHandler()                   { Device->FLASH_IRQHandler(); }
void RCC_IRQHandler()                     { Device->RCC_IRQHandler(); }
void EXTI0_IRQHandler()                   { Device->EXTI0_IRQHandler(); }
void EXTI1_IRQHandler()                   { Device->EXTI1_IRQHandler(); }
void EXTI2_IRQHandler()                   { Device->EXTI2_IRQHandler(); }
void EXTI3_IRQHandler()                   { Device->EXTI3_IRQHandler(); }
void EXTI4_IRQHandler()                   { Device->EXTI4_IRQHandler(); }
void DMA1_Channel1_IRQHandler()           { Device->DMA1_Channel1_IRQHandler(); }
void DMA1_Channel2_IRQHandler()           { Device->DMA1_Channel2_IRQHandler(); }
void DMA1_Channel3_IRQHandler()           { Device->DMA1_Channel3_IRQHandler(); }
void DMA1_Channel4_IRQHandler()           { Device->DMA1_Channel4_IRQHandler(); }
void DMA1_Channel5_IRQHandler()           { Device->DMA1_Channel5_IRQHandler(); }
void DMA1_Channel6_IRQHandler()           { Device->DMA1_Channel6_IRQHandler(); }
void DMA1_Channel7_IRQHandler()           { Device->DMA1_Channel7_IRQHandler(); }
void DMA1_Channel8_IRQHandler()           { Device->DMA1_Channel8_IRQHandler(); }
void ADC1_2_IRQHandler()                  { Device->ADC1_2_IRQHandler(); }
void USB_HP_IRQHandler()                  { Device->USB_HP_IRQHandler(); }
void USB_LP_IRQHandler()                  { Device->USB_LP_IRQHandler(); }
void FDCAN1_IT0_IRQHandler()              { Device->FDCAN1_IT0_IRQHandler(); }
void FDCAN1_IT1_IRQHandler()              { Device->FDCAN1_IT1_IRQHandler(); }
void EXTI9_5_IRQHandler()                 { Device->EXTI9_5_IRQHandler(); }
void TIM1_BRK_TIM15_IRQHandler()          { Device->TIM1_BRK_TIM15_IRQHandler(); }
void TIM1_UP_TIM16_IRQHandler()           { Device->TIM1_UP_TIM16_IRQHandler(); }
void TIM1_TRG_COM_TIM17_IRQHandler()      { Device->TIM1_TRG_COM_TIM17_IRQHandler(); }
void TIM1_CC_IRQHandler()                 { Device->TIM1_CC_IRQHandler(); }
void TIM2_IRQHandler()                    { Device->TIM2_IRQHandler(); }
void TIM3_IRQHandler()                    { Device->TIM3_IRQHandler(); }
void TIM4_IRQHandler()                    { Device->TIM4_IRQHandler(); }
void I2C1_EV_IRQHandler()                 { Device->I2C1_EV_IRQHandler(); }
void I2C1_ER_IRQHandler()                 { Device->I2C1_ER_IRQHandler(); }
void I2C2_EV_IRQHandler()                 { Device->I2C2_EV_IRQHandler(); }
void I2C2_ER_IRQHandler()                 { Device->I2C2_ER_IRQHandler(); }
void SPI1_IRQHandler()                    { Device->SPI1_IRQHandler(); }
void SPI2_IRQHandler()                    { Device->SPI2_IRQHandler(); }
void USART1_IRQHandler()                  { Device->USART1_IRQHandler(); }
void USART2_IRQHandler()                  { Device->USART2_IRQHandler(); }
void USART3_IRQHandler()                  { Device->USART3_IRQHandler(); }
void EXTI15_10_IRQHandler()               { Device->EXTI15_10_IRQHandler(); }
void RTC_Alarm_IRQHandler()               { Device->RTC_Alarm_IRQHandler(); }
void USBWakeUp_IRQHandler()               { Device->USBWakeUp_IRQHandler(); }
void TIM8_BRK_IRQHandler()                { Device->TIM8_BRK_IRQHandler(); }
void TIM8_UP_IRQHandler()                 { Device->TIM8_UP_IRQHandler(); }
void TIM8_TRG_COM_IRQHandler()            { Device->TIM8_TRG_COM_IRQHandler(); }
void TIM8_CC_IRQHandler()                 { Device->TIM8_CC_IRQHandler(); }
void ADC3_IRQHandler()                    { Device->ADC3_IRQHandler(); }
void FMC_IRQHandler()                     { Device->FMC_IRQHandler(); }
void LPTIM1_IRQHandler()                  { Device->LPTIM1_IRQHandler(); }
void TIM5_IRQHandler()                    { Device->TIM5_IRQHandler(); }
void SPI3_IRQHandler()                    { Device->SPI3_IRQHandler(); }
void UART4_IRQHandler()                   { Device->UART4_IRQHandler(); }
void UART5_IRQHandler()                   { Device->UART5_IRQHandler(); }
void TIM6_DAC_IRQHandler()                { Device->TIM6_DAC_IRQHandler(); }
void TIM7_DAC_IRQHandler()                { Device->TIM7_DAC_IRQHandler(); }
void DMA2_Channel1_IRQHandler()           { Device->DMA2_Channel1_IRQHandler(); }
void DMA2_Channel2_IRQHandler()           { Device->DMA2_Channel2_IRQHandler(); }
void DMA2_Channel3_IRQHandler()           { Device->DMA2_Channel3_IRQHandler(); }
void DMA2_Channel4_IRQHandler()           { Device->DMA2_Channel4_IRQHandler(); }
void DMA2_Channel5_IRQHandler()           { Device->DMA2_Channel5_IRQHandler(); }
void DMA2_Channel6_IRQHandler()           { Device->DMA2_Channel6_IRQHandler(); }
void DMA2_Channel7_IRQHandler()           { Device->DMA2_Channel7_IRQHandler(); }
void DMA2_Channel8_IRQHandler()           { Device->DMA2_Channel8_IRQHandler(); }
void ADC4_IRQHandler()                    { Device->ADC4_IRQHandler(); }
void ADC5_IRQHandler()                    { Device->ADC5_IRQHandler(); }
void UCPD1_IRQHandler()                   { Device->UCPD1_IRQHandler(); }
void COMP1_2_3_IRQHandler()               { Device->COMP1_2_3_IRQHandler(); }
void COMP4_5_6_IRQHandler()               { Device->COMP4_5_6_IRQHandler(); }
void COMP7_IRQHandler()                   { Device->COMP7_IRQHandler(); }
void CRS_IRQHandler()                     { Device->CRS_IRQHandler(); }
void SAI1_IRQHandler()                    { Device->SAI1_IRQHandler(); }
void TIM20_BRK_IRQHandler()               { Device->TIM20_BRK_IRQHandler(); }
void TIM20_UP_IRQHandler()                { Device->TIM20_UP_IRQHandler(); }
void TIM20_TRG_COM_IRQHandler()           { Device->TIM20_TRG_COM_IRQHandler(); }
void TIM20_CC_IRQHandler()                { Device->TIM20_CC_IRQHandler(); }
void FPU_IRQHandler()                     { Device->FPU_IRQHandler(); }
void I2C4_EV_IRQHandler()                 { Device->I2C4_EV_IRQHandler(); }
void I2C4_ER_IRQHandler()                 { Device->I2C4_ER_IRQHandler(); }
void SPI4_IRQHandler()                    { Device->SPI4_IRQHandler(); }
void FDCAN2_IT0_IRQHandler()              { Device->FDCAN2_IT0_IRQHandler(); }
void FDCAN2_IT1_IRQHandler()              { Device->FDCAN2_IT1_IRQHandler(); }
void FDCAN3_IT0_IRQHandler()              { Device->FDCAN3_IT0_IRQHandler(); }
void FDCAN3_IT1_IRQHandler()              { Device->FDCAN3_IT1_IRQHandler(); }
void RNG_IRQHandler()                     { Device->RNG_IRQHandler(); }
void LPUART1_IRQHandler()                 { Device->LPUART1_IRQHandler(); }
void I2C3_EV_IRQHandler()                 { Device->I2C3_EV_IRQHandler(); }
void I2C3_ER_IRQHandler()                 { Device->I2C3_ER_IRQHandler(); }
void DMAMUX_OVR_IRQHandler()              { Device->DMAMUX_OVR_IRQHandler(); }
void QUADSPI_IRQHandler()                 { Device->QUADSPI_IRQHandler(); }
void CORDIC_IRQHandler()                  { Device->CORDIC_IRQHandler(); }

} // extern "C"
