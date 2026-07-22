#include "gpio_ll.h"

namespace gpio_ll {

namespace {

uint32_t pin_shift(uint8_t pin) {
    return static_cast<uint32_t>(pin) * 2u;
}

void set_moder(GPIO_TypeDef* gpio, uint8_t pin, uint32_t mode2b) {
    const uint32_t shift = pin_shift(pin);
    const uint32_t mask = 0x3u << shift;
    gpio->MODER = (gpio->MODER & ~mask) | (mode2b << shift);
}

void set_pupdr(GPIO_TypeDef* gpio, uint8_t pin, Pull pull) {
    const uint32_t shift = pin_shift(pin);
    const uint32_t mask = 0x3u << shift;
    gpio->PUPDR = (gpio->PUPDR & ~mask) | (static_cast<uint32_t>(pull) << shift);
}

void set_speed(GPIO_TypeDef* gpio, uint8_t pin, Speed speed) {
    const uint32_t shift = pin_shift(pin);
    const uint32_t mask = 0x3u << shift;
    gpio->OSPEEDR = (gpio->OSPEEDR & ~mask) | (static_cast<uint32_t>(speed) << shift);
}

} // namespace

void enable_port_clock(GPIO_TypeDef* port) {
    if (port == GPIOA) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    } else if (port == GPIOB) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    } else if (port == GPIOC) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    } else if (port == GPIOD) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
    } else if (port == GPIOE) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
    } else if (port == GPIOF) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
    } else if (port == GPIOG) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
    }

    (void)RCC->AHB2ENR;
}

void configure_output(Pin pin, Pull pull, Speed speed, bool default_high) {
    GPIO_TypeDef* gpio = pin.port;
    if (gpio == nullptr || pin.number > 15u) {
        return;
    }

    enable_port_clock(gpio);

    set_moder(gpio, pin.number, 0b01u);
    set_pupdr(gpio, pin.number, pull);
    set_speed(gpio, pin.number, speed);
    gpio->OTYPER &= ~(1u << pin.number);

    write(pin, default_high);
}

void configure_input(Pin pin, Pull pull) {
    GPIO_TypeDef* gpio = pin.port;
    if (gpio == nullptr || pin.number > 15u) {
        return;
    }

    enable_port_clock(gpio);

    set_moder(gpio, pin.number, 0b00u);
    set_pupdr(gpio, pin.number, pull);
}

void configure_alternate(Pin pin, uint8_t af, Pull pull, Speed speed) {
    GPIO_TypeDef* gpio = pin.port;
    if (gpio == nullptr || pin.number > 15u || af > 15u) {
        return;
    }

    enable_port_clock(gpio);

    set_moder(gpio, pin.number, 0b10u);
    set_pupdr(gpio, pin.number, pull);
    set_speed(gpio, pin.number, speed);
    gpio->OTYPER &= ~(1u << pin.number);

    const uint32_t af_shift = static_cast<uint32_t>(pin.number % 8u) * 4u;
    const uint32_t af_mask = 0xFu << af_shift;

    if (pin.number < 8u) {
        gpio->AFR[0] = (gpio->AFR[0] & ~af_mask) | (static_cast<uint32_t>(af) << af_shift);
    } else {
        gpio->AFR[1] = (gpio->AFR[1] & ~af_mask) | (static_cast<uint32_t>(af) << af_shift);
    }
}

void configure_analog(Pin pin, Pull pull) {
    GPIO_TypeDef* gpio = pin.port;
    if (gpio == nullptr || pin.number > 15u) {
        return;
    }

    enable_port_clock(gpio);
    set_moder(gpio, pin.number, 0b11u);
    set_pupdr(gpio, pin.number, pull);
}

void write(Pin pin, bool high) {
    GPIO_TypeDef* gpio = pin.port;
    if (gpio == nullptr || pin.number > 15u) {
        return;
    }

    const uint32_t bit = 1u << pin.number;
    gpio->BSRR = high ? bit : (bit << 16u);
}

bool read(Pin pin) {
    GPIO_TypeDef* gpio = pin.port;
    if (gpio == nullptr || pin.number > 15u) {
        return false;
    }

    return (gpio->IDR & (1u << pin.number)) != 0u;
}

void configure_exti_line(Pin pin) {
    GPIO_TypeDef* gpio = pin.port;
    if (gpio == nullptr || pin.number > 15u) {
        return;
    }

    uint32_t port_code = 0;
    if (gpio == GPIOA) {
        port_code = 0;
    } else if (gpio == GPIOB) {
        port_code = 1;
    } else if (gpio == GPIOC) {
        port_code = 2;
    } else if (gpio == GPIOD) {
        port_code = 3;
    } else if (gpio == GPIOE) {
        port_code = 4;
    } else if (gpio == GPIOF) {
        port_code = 5;
    } else if (gpio == GPIOG) {
        port_code = 6;
    } else {
        return;
    }

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // EXTICR lives in SYSCFG
    (void)RCC->APB2ENR;

    const uint8_t reg_index = pin.number / 4u;
    const uint32_t shift = static_cast<uint32_t>(pin.number % 4u) * 4u;
    const uint32_t mask = 0xFu << shift;
    SYSCFG->EXTICR[reg_index] = (SYSCFG->EXTICR[reg_index] & ~mask) | (port_code << shift);
}

void toggle(Pin pin) {
    GPIO_TypeDef* gpio = pin.port;
    if (gpio == nullptr || pin.number > 15u) {
        return;
    }

    const uint32_t bit = 1u << pin.number;
    const bool current = (gpio->ODR & bit) != 0u;
    gpio->BSRR = current ? (bit << 16u) : bit;
}

} // namespace gpio_ll
