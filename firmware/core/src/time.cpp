#include "time.h"
#include "board_hw.h"

volatile uint64_t time_interface::microseconds = 0;
volatile uint64_t time_interface::microseconds_offset = 0;
volatile uint32_t time_interface::last_timer_cnt = 0;

void time_interface::init() {
    // timer 5 is used to trigger off of the uart rx line for comms,
    // as well as trigger the main PWM timers

    // TODO: make this more hardware agnostic
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN; // enable timer 5 clock

    board_hw::comm_uart_rx_sync_timer->PSC = 170-1; // 1 MHz timer clock (1 us resolution)
    board_hw::comm_uart_rx_sync_timer->ARR = 0xFFFFFFFF; // max for 32-bit timer

    board_hw::comm_uart_rx_sync_timer->EGR |= TIM_EGR_UG; // update registers

    board_hw::comm_uart_rx_sync_timer->CR2 |= 0b0101 << TIM_CR2_MMS_Pos; // trigger trgo on output compare ch2

    board_hw::comm_uart_rx_sync_timer->CR1 |= 0b1 << TIM_CR1_CEN_Pos; // enable timer

    last_timer_cnt = board_hw::comm_uart_rx_sync_timer->CNT;
}

uint64_t time_interface::get_microseconds() {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint64_t current_microseconds = microseconds + microseconds_offset;
    __set_PRIMASK(primask);
    return current_microseconds;
}

uint64_t time_interface::get_microseconds_update() {
    // update the microseconds counter based on the current timer count
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t current_timer_cnt = board_hw::comm_uart_rx_sync_timer->CNT;    // 32 bit timer
    // Unsigned subtraction preserves elapsed ticks across one or more wraps modulo 2^32.
    uint32_t delta_ticks = static_cast<uint32_t>(current_timer_cnt - last_timer_cnt);
    microseconds = microseconds + delta_ticks;

    last_timer_cnt = current_timer_cnt;
    uint64_t current_microseconds = microseconds + microseconds_offset;

    __set_PRIMASK(primask);

    return current_microseconds;
}

void time_interface::sync_timer(uint32_t offset) {
    // TODO: implement
}