#pragma once

#include <cstdint>

class time_interface {
    public:
        static void init();
        static uint64_t get_microseconds();
        static uint64_t get_microseconds_update();
        static void sync_timer(uint32_t offset);

    private:
        static volatile uint64_t microseconds;
        static volatile uint64_t microseconds_offset;
        static volatile uint32_t last_timer_cnt;

};

#ifndef MSG_GET_TIME_US
#define MSG_GET_TIME_US() time_interface::get_microseconds_update()
#endif