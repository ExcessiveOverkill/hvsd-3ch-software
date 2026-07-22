#pragma once

#include <cstdint>

class communication_interface {
public:
    void init();

    void send_string_blocking(const char* str);
    void send_byte_blocking(uint8_t byte);
    bool receive_byte_nonblocking(uint8_t& byte);

private:
    uint16_t BRR_from_baud(uint32_t baud);

    inline void set_tx_passthrough(bool enable);
    inline void set_rx_passthrough(bool enable);
};