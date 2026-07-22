#include "communication.h"

#include "board_hw.h"
#include "gpio_ll.h"

void communication_interface::init() {

    gpio_ll::configure_alternate({board_hw::comm_uart_rx_port, board_hw::comm_uart_rx_pin}, board_hw::comm_uart_rx_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::high);
    gpio_ll::configure_alternate({board_hw::comm_uart_tx_port, board_hw::comm_uart_tx_pin}, board_hw::comm_uart_tx_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::high);

    gpio_ll::configure_output({board_hw::comm_tx_passthrough_disable_port, board_hw::comm_tx_passthrough_disable_pin}, gpio_ll::Pull::none, gpio_ll::Speed::high, true);
    gpio_ll::configure_output({board_hw::comm_rx_passthrough_enable_port, board_hw::comm_rx_passthrough_enable_pin}, gpio_ll::Pull::none, gpio_ll::Speed::high, false);
    // gpio_ll::configure_output({board_hw::comm_aux_drive_enable_port, board_hw::comm_aux_drive_enable_pin}, gpio_ll::Pull::none, gpio_ll::Speed::high, true);
    
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // RCC->CCIPR2 |= (0b01u << RCC_CCIPR_USART1SEL_Pos); // select SYSCLK as USART1 clock source

    uint16_t brr = BRR_from_baud(115200);
    board_hw::comm_uart->BRR = brr;

    board_hw::comm_uart->CR1 |= USART_CR1_FIFOEN; // enable FIFO mode

    board_hw::comm_uart->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // enable transmitter, receiver, and UART


}

uint16_t communication_interface::BRR_from_baud(uint32_t baud) {
    // calculate USART_BRR value for given baud rate, using current peripheral clock (PCLK2 for USART1)
    // see RM0440 40.5.7 for details on USART_BRR calculation

    uint32_t pclk = board_hw::comm_uart_timer_ker_clk; // get current peripheral clock frequency

    uint16_t prescaler = 1;
    switch (board_hw::comm_uart->PRESC) {   // see RM0440 40.8.14 USART_PRESC bits
        case 0b0000: prescaler = 1; break;
        case 0b0001: prescaler = 2; break;
        case 0b0010: prescaler = 4; break;
        case 0b0011: prescaler = 6; break;
        case 0b0100: prescaler = 8; break;
        case 0b0101: prescaler = 10; break;
        case 0b0110: prescaler = 12; break;
        case 0b0111: prescaler = 16; break;
        case 0b1000: prescaler = 32; break;
        case 0b1001: prescaler = 64; break;
        case 0b1010: prescaler = 128; break;
        case 0b1011: prescaler = 256; break;
    }
    pclk /= prescaler;

    uint8_t over_sampling = (board_hw::comm_uart->CR1 & USART_CR1_OVER8) ? 2 : 1;
    
    pclk /= over_sampling;

    uint16_t div = pclk / baud;
    
    volatile uint32_t check_ker_clk = div * baud * over_sampling * prescaler;   // TODO: use this to verify that the calculated BRR value will produce the desired baud rate within some tolerance

    return div;
}

void communication_interface::send_byte_blocking(uint8_t byte) {
    while(!(board_hw::comm_uart->ISR & USART_ISR_TC));
    set_tx_passthrough(false); // disable passthrough to allow transmitting own data
    board_hw::comm_uart->TDR = byte; // write byte to transmit data register
    while(!(board_hw::comm_uart->ISR & USART_ISR_TC));
    set_tx_passthrough(true); // re-enable passthrough after transmission
}

bool communication_interface::receive_byte_nonblocking(uint8_t& byte) {
    if (board_hw::comm_uart->ISR & USART_ISR_RXNE) {
        byte = static_cast<uint8_t>(board_hw::comm_uart->RDR); // read received byte from receive data register
        return true;
    } else {
        return false; // no byte received
    }
}

void communication_interface::send_string_blocking(const char* str) {
    while (*str) {  // must use a null-terminated string
        send_byte_blocking(static_cast<uint8_t>(*str));
        str++;
    }
}

inline void communication_interface::set_tx_passthrough(bool enable) {
    // when enabled, device cannot transmit its own data and instead passes through data from the other COM port
    gpio_ll::write({board_hw::comm_tx_passthrough_disable_port, board_hw::comm_tx_passthrough_disable_pin}, !enable);
}

inline void communication_interface::set_rx_passthrough(bool enable) {
    // when enabled, device passes recieved data from the COM port to the other COM port, still can receive data itself
    gpio_ll::write({board_hw::comm_rx_passthrough_enable_port, board_hw::comm_rx_passthrough_enable_pin}, enable);
}