#pragma once

#include <cstdint>
#include "time.h"

class led_interface{
    public:
        void init(time_interface* time_interface);
        
        enum class led_state {
            OFF,
            ON,
            BLINK_SLOW,
            BLINK_MEDIUM,
            BLINK_FAST,
            BLINK_SLOW_INVERTED,
            BLINK_MEDIUM_INVERTED,
            BLINK_FAST_INVERTED
        };

        enum class led_id {
            COMM_PORT_1_LED_0,
            COMM_PORT_1_LED_1,
            COMM_PORT_2_LED_0,
            COMM_PORT_2_LED_1,
            BOARD_LED_0,
            BOARD_LED_1
        };

        void set_state(led_id id, led_state state);

        void update();

    private:
        led_state comm_port_1_led_0_state = led_state::OFF;
        led_state comm_port_1_led_1_state = led_state::OFF;
        led_state comm_port_2_led_0_state = led_state::OFF;
        led_state comm_port_2_led_1_state = led_state::OFF;
        led_state board_led_0_state = led_state::OFF;
        led_state board_led_1_state = led_state::OFF;

        time_interface* time = nullptr;

        void update_led(led_id id, led_state state, bool fast_blink, bool medium_blink, bool slow_blink);
};