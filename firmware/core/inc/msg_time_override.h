#pragma once

#include "time.h"

#ifndef MSG_GET_TIME_US
#define MSG_GET_TIME_US() time_interface::get_microseconds_update()
#endif
