#pragma once

#include <cstdint>

class fan_interface {
public:
	void init();
    void systick_flagged_handler();
    void set_speed_percent(uint8_t percent); // percent should be between 0 and 100
    uint16_t get_rpm();

private:
    uint16_t rpm = 0;
    uint8_t update_divider = 100; // number of systick updates to use to calculate RPM
    uint8_t update_counter = 0; // counter for systick updates, counts down from update_divider to 0
    uint8_t pulses_per_revolution = 2; // number of tach pulses per revolution of the fan, used for RPM calculation
};
