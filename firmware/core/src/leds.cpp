#include "leds.h"
#include "gpio_ll.h"
#include "board_hw.h"

void led_interface::init(time_interface* time_interface) {
    time = time_interface;

    // board leds
    gpio_ll::enable_port_clock(board_hw::led_0_port);
    gpio_ll::enable_port_clock(board_hw::led_1_port);
    gpio_ll::configure_output({board_hw::led_0_port, board_hw::led_0_pin});
    gpio_ll::configure_output({board_hw::led_1_port, board_hw::led_1_pin});

    // communication port leds
    gpio_ll::enable_port_clock(board_hw::comm_port_1_led_0_port);
    gpio_ll::enable_port_clock(board_hw::comm_port_1_led_1_port);
    gpio_ll::enable_port_clock(board_hw::comm_port_2_led_0_port);
    gpio_ll::enable_port_clock(board_hw::comm_port_2_led_1_port);
    gpio_ll::configure_output({board_hw::comm_port_1_led_0_port, board_hw::comm_port_1_led_0_pin});
    gpio_ll::configure_output({board_hw::comm_port_1_led_1_port, board_hw::comm_port_1_led_1_pin});
    gpio_ll::configure_output({board_hw::comm_port_2_led_0_port, board_hw::comm_port_2_led_0_pin});
    gpio_ll::configure_output({board_hw::comm_port_2_led_1_port, board_hw::comm_port_2_led_1_pin});

    set_state(led_id::COMM_PORT_1_LED_0, led_state::BLINK_MEDIUM);
    set_state(led_id::COMM_PORT_1_LED_1, led_state::BLINK_MEDIUM_INVERTED);
    set_state(led_id::COMM_PORT_2_LED_0, led_state::BLINK_MEDIUM);
    set_state(led_id::COMM_PORT_2_LED_1, led_state::BLINK_MEDIUM_INVERTED);
    set_state(led_id::BOARD_LED_0, led_state::BLINK_MEDIUM);
    set_state(led_id::BOARD_LED_1, led_state::BLINK_MEDIUM_INVERTED);
}

void led_interface::set_state(led_id id, led_state state) {
    switch (id) {
        case led_id::COMM_PORT_1_LED_0:
            comm_port_1_led_0_state = state;
            break;
        case led_id::COMM_PORT_1_LED_1:
            comm_port_1_led_1_state = state;
            break;
        case led_id::COMM_PORT_2_LED_0:
            comm_port_2_led_0_state = state;
            break;
        case led_id::COMM_PORT_2_LED_1:
            comm_port_2_led_1_state = state;
            break;
        case led_id::BOARD_LED_0:
            board_led_0_state = state;
            break;
        case led_id::BOARD_LED_1:
            board_led_1_state = state;
            break;
    }
}

void led_interface::update() {
    // Implementation for updating the LED states based on the current time
    uint64_t current_time = time->get_microseconds();

    bool fast_blink = (current_time & (0b1 << 16)) != 0; // 65,536us period
    bool medium_blink = (current_time & (0b1 << 18)) != 0; // 262,144us period
    bool slow_blink = (current_time & (0b1 << 20)) != 0; // 1,048,576us period

    update_led(led_id::COMM_PORT_1_LED_0, comm_port_1_led_0_state, fast_blink, medium_blink, slow_blink);
    update_led(led_id::COMM_PORT_1_LED_1, comm_port_1_led_1_state, fast_blink, medium_blink, slow_blink);
    update_led(led_id::COMM_PORT_2_LED_0, comm_port_2_led_0_state, fast_blink, medium_blink, slow_blink);
    update_led(led_id::COMM_PORT_2_LED_1, comm_port_2_led_1_state, fast_blink, medium_blink, slow_blink);
    update_led(led_id::BOARD_LED_0, board_led_0_state, fast_blink, medium_blink, slow_blink);
    update_led(led_id::BOARD_LED_1, board_led_1_state, fast_blink, medium_blink, slow_blink);
}

void led_interface::update_led(led_id id, led_state state, bool fast_blink, bool medium_blink, bool slow_blink) {
    GPIO_TypeDef* port;
    uint8_t pin;

    switch (id) {
        case led_id::COMM_PORT_1_LED_0:
            port = board_hw::comm_port_1_led_0_port;
            pin = board_hw::comm_port_1_led_0_pin;
            break;
        case led_id::COMM_PORT_1_LED_1:
            port = board_hw::comm_port_1_led_1_port;
            pin = board_hw::comm_port_1_led_1_pin;
            break;
        case led_id::COMM_PORT_2_LED_0:
            port = board_hw::comm_port_2_led_0_port;
            pin = board_hw::comm_port_2_led_0_pin;
            break;
        case led_id::COMM_PORT_2_LED_1:
            port = board_hw::comm_port_2_led_1_port;
            pin = board_hw::comm_port_2_led_1_pin;
            break;
        case led_id::BOARD_LED_0:
            port = board_hw::led_0_port;
            pin = board_hw::led_0_pin;
            break;
        case led_id::BOARD_LED_1:
            port = board_hw::led_1_port;
            pin = board_hw::led_1_pin;
            break;
    }

    bool output_state = false;

    switch (state) {
        case led_state::OFF:
            output_state = false;
            break;
        case led_state::ON:
            output_state = true;
            break;
        case led_state::BLINK_SLOW:
            output_state = slow_blink;
            break;
        case led_state::BLINK_SLOW_INVERTED:
            output_state = !slow_blink;
            break;
        case led_state::BLINK_MEDIUM:
            output_state = medium_blink;
            break;
        case led_state::BLINK_MEDIUM_INVERTED:
            output_state = !medium_blink;
            break;
        case led_state::BLINK_FAST:
            output_state = fast_blink;
            break;
        case led_state::BLINK_FAST_INVERTED:
            output_state = !fast_blink;
            break;
        default:
            output_state = false;
            break;
    }

    gpio_ll::write({port, pin}, output_state);
}
    