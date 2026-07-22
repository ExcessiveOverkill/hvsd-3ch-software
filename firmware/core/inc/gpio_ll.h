#pragma once

#include <cstdint>

#include "stm32g473xx.h"

namespace gpio_ll {

enum class Pull : uint32_t {
    none = 0b00u,
    up = 0b01u,
    down = 0b10u,
};

enum class Speed : uint32_t {
    low = 0b00u,
    medium = 0b01u,
    high = 0b10u,
    very_high = 0b11u,
};

struct Pin {
    GPIO_TypeDef* port;
    uint8_t number;
};

void enable_port_clock(GPIO_TypeDef* port);

void configure_output(Pin pin, Pull pull = Pull::none, Speed speed = Speed::low, bool default_high = false);
void configure_alternate(Pin pin, uint8_t af, Pull pull = Pull::none, Speed speed = Speed::very_high);
void configure_analog(Pin pin, Pull pull = Pull::none);
void configure_input(Pin pin, Pull pull = Pull::none);

// Routes the given pin's EXTI line to this pin's port via SYSCFG->EXTICRx.
// Must be called before relying on EXTI->IMR1/FTSR1/RTSR1 for this pin's line,
// since the EXTICR mux otherwise defaults to GPIOA on reset.
void configure_exti_line(Pin pin);

void write(Pin pin, bool high);
bool read(Pin pin);
void toggle(Pin pin);

} // namespace gpio_ll
