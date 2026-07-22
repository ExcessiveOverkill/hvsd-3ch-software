#include "stm32g4xx.h"

void SystemInit(void) {
    /* Enable FPU — set CP10 and CP11 to full access (Cortex-M4F) */
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));

    /* TODO: configure system clocks (PLL, flash wait states, peripheral clocks) */
}
