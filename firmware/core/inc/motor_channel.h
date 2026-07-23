#pragma once

#include <cstdint>
#include "stm32g473xx.h"
#include "board_hw.h"
#include "aura.hpp"
#include "time.h"

class motor_channel {
    public:
        const board_hw::motor_hw_config* config;

        motor_channel(uint8_t channel_num_, const board_hw::motor_hw_config* config_, Messaging* msg_, time_interface* time_);
        
        void init();
        void main_output_disable();
        uint8_t main_output_enable();
        void stop_timer();
        void start_timer_unsynced();
        void start_timer_synced();
        void set_scaled_pwm_values(int16_t phase_u, int16_t phase_v, int16_t phase_w);
        bool set_phase_voltage(float phase_u_voltage, float phase_v_voltage, float phase_w_voltage);
        void set_pwm_frequency(float frequency_hz, float* resulting_frequency_hz = nullptr, uint16_t* resulting_arr = nullptr);
        void manual_phase_adc_trigger();
        static void manual_aux_adc_trigger();
        static void enable_phase_adcs();
        static void disable_phase_adcs();
        static bool phase_adcs_enabled();
        static void enable_aux_adc();
        static void disable_aux_adc();
        static bool aux_adc_enabled();
        void reset_faults();
        uint8_t get_channel_num() const { return channel_num; }
        inline static uint8_t get_phase_adc_sample_index() { return phase_adc_sample_index; }
        void all_phase_adc_eoc_flagged_handler();
        static void aux_adc_eoc_flagged_handler();
        static void phase_adc2_injected_eoc_flagged_handler();
        void ipm_exti_irq_handler();
        void enable_phase_adc_timer_trigger();
        void disable_phase_adc_timer_trigger();
        bool is_timer_running();
        inline static float get_gate_supply_voltage() { return gate_supply_voltage; }
        inline static float get_VBUS_voltage() { return VBUS_voltage; }
        inline float get_IPM_IC_temp() { return IPM_IC_temp[channel_num]; }
        inline float get_IPM_IC_thermistor_temp() { return IPM_IC_thermistor_temp[channel_num]; }
        void get_phase_currents(float& phase_u, float& phase_v, float& phase_w) {
            phase_u = phase_u_current;
            phase_v = phase_v_current;
            phase_w = phase_w_current;
        }
        void zero_phase_adcs() {
            phase_u_adc_offset = board_hw::phase_u_adc->DR << phase_adc_result_left_shift;
            phase_v_adc_offset = board_hw::phase_v_adc->DR << phase_adc_result_left_shift;
            phase_w_adc_offset = board_hw::phase_w_adc->DR << phase_adc_result_left_shift;
        }

        bool main_output_enabled() {
            return (config->timer->BDTR & TIM_BDTR_MOE) != 0;
        }

        static uint32_t calculate_pwm_cycles_from_us(uint32_t microseconds);

        static bool global_safety_checks();
        bool channel_safety_checks();
        static bool aux_adc_cycle_complete() { return aux_adc_data_ready; }
    
    private:

        static Messaging* msg;
        _Msg_Motor motor_msgs;
        static time_interface* time;

        uint8_t channel_num;
        static uint8_t adc_resolution_bits;   // conversion resolution in bits
        static uint8_t phase_adc_result_left_shift; // number of bits to left shift ADC result to align with 16-bit full scale after oversampling
        static uint8_t phase_adc_sample_index;    // index of the current ADC sample in the sequence, shared between all channels since ADCs are synchronized
        static uint8_t num_of_channels; // number of motor channels
        static uint16_t phase_adc_trigger_offset; // number of timer ticks before the timer overflow event to trigger ADC conversion, shared between all channels since timers are synchronized
        static bool aux_adc_data_ready; // all aux adc data is present
        
        static uint32_t ns_per_pwm_cycle; // number of nanoseconds per PWM cycle, shared between all channels since timers are synchronized

        static uint8_t aux_adc_sample_index; // index of the current auxiliary ADC sample in the sequence, shared between all channels since ADCs are synchronized
        static uint8_t aux_adc_result_left_shift; // number of bits to left shift auxiliary ADC result to align with 16-bit full scale after oversampling

        static uint16_t phase_u_vref_offset;
        static uint16_t phase_v_vref_offset;
        static uint16_t phase_w_vref_offset;

        int16_t phase_u_adc_offset = 0;
        int16_t phase_v_adc_offset = 0;
        int16_t phase_w_adc_offset = 0;
        float phase_u_current = 0.0f;
        float phase_v_current = 0.0f;
        float phase_w_current = 0.0f;
        bool new_phase_adc_data_available = false;

        bool ipm_fault_detected = false;

        float max_phase_imbalance_amps = 1.0f;

        static float gate_supply_voltage;
        static float VBUS_voltage;
        static float IPM_IC_temp[3];
        static float IPM_IC_thermistor_temp[3];
        static bool new_aux_adc_data_available;

        static float min_vbus_voltage;

        bool use_discontinuous_pwm = false; // use discontinuous PWM mode to reduce switching losses
        bool use_basic_deadtime_compensation = false; // use basic deadtime compensation to provide more accurate phase voltage control
        bool use_advanced_deadtime_compensation = false; // use advanced deadtime compensation to provide more accurate phase voltage control



        void phase_adc_init();
        void vbus_sense_adc_init();
        void aux_adc_init();
        void ipm_fault_init();
        void sto_init();
        void enable_adc_clock(ADC_TypeDef* adc);
        void power_on_adc(ADC_TypeDef* adc);
        void configure_adc_watchdog(ADC_TypeDef* adc, float current, uint16_t center_offset, uint32_t channels);
        uint8_t configure_adc_oversampling(ADC_TypeDef* adc, uint8_t oversampling_ratio);
        void configure_adc_sample_time(ADC_TypeDef* adc, uint32_t channel, uint32_t sample_time_cycles);
        void timer_init();
        uint16_t timer_arr_from_frequency(float frequency_hz, float* resulting_frequency_hz = nullptr);
        uint16_t get_adc_reading_blocking(ADC_TypeDef* adc, uint32_t channel);
        static void trigger_vbus_sense_adc_reading();
        static float calculate_gate_supply_voltage(uint16_t adc_value);
        static float calculate_VBUS_voltage(uint16_t adc_value);
        static float calculate_IPM_IC_temp(uint16_t adc_value);
        static float calculate_IPM_IC_thermistor_temp(uint16_t adc_value);
        static float calculate_phase_current(uint16_t adc_value, int16_t adc_offset, uint16_t adc_vref_2_offset = 32768);
        static float calculate_adc_counts_from_current(float current_amps);
        static float calculate_svpwm_offset(float v_u, float v_v, float v_w);
        bool select_discontinuous_offset(float v_u, float v_v, float v_w, float v_half, float& offset);
        static int16_t voltage_to_raw(float voltage, float v_half);
        void apply_basic_deadtime_compensation(int16_t& raw_u, int16_t& raw_v, int16_t& raw_w);
        void enforce_min_pulse_width(int16_t& raw_u, int16_t& raw_v, int16_t& raw_w);
        static void reset_analog_watchdogs();
        static bool get_analog_watchdog_status();
        bool get_sto_ch1_fault_status();
        static bool get_sto_ch2_fault_status();
    };