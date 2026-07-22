#include "motor_adc.h"

#include "board_hw.h"
#include "gpio_ll.h"

namespace motor_adc {

namespace {

volatile uint16_t sample_buffer[3] = {0u, 0u, 0u};

void enable_adc_clock(ADC_TypeDef* adc) {
    if (adc == ADC1 || adc == ADC2) {
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    } else if (adc == ADC3) {
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
    }
    (void)RCC->AHB2ENR;
}

void enable_dma_clock() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;
}

} // namespace

void init() {
    // TODO: fill
}


} // namespace motor_adc
