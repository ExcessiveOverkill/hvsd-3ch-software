
#pragma once

#include <cstdint>
#include "motor_channel.h"
#include "aura.hpp"
#include "control_utils.h"

// runs actual logic and control that varies based on the mode of operation
class mode {
    public:
        mode(motor_channel* mtr_ch_, Messaging* msg_, time_interface* time_);
        void init();

        enum class states {
            IDLE,
            START,  // user logic runs here
            RUN,    // user logic runs here
            SOFT_STOP,  // user logic runs here
            HARD_STOP,
            FAULT,
            INVALID_STATE_TRANSITION
        };

        
        states get_current_state() const { return current_state; }
        states get_requested_state() const { return requested_state; }
        void set_requested_state(states new_state) { requested_state = new_state; }

        void loop();
        void flagged_timer_update();
        void flagged_all_adc_eoc();
        void flagged_systick();

        void reset();

        enum class function_results {
            CONTINUE,
            COMPLETE,
            ERROR
        };

        virtual void user_reset(){};

        virtual function_results user_start_loop(){ return function_results::COMPLETE; };
        virtual function_results user_start_flagged_timer_update(){ return function_results::COMPLETE; };
        virtual function_results user_start_flagged_all_adc_eoc(){ return function_results::COMPLETE; };
        virtual function_results user_start_flagged_systick(){ return function_results::COMPLETE; };

        virtual function_results user_run_loop(){ return function_results::COMPLETE; };
        virtual function_results user_run_flagged_timer_update(){ return function_results::COMPLETE; };
        virtual function_results user_run_flagged_all_adc_eoc(){ return function_results::COMPLETE; };
        virtual function_results user_run_flagged_systick(){ return function_results::COMPLETE; };

        virtual function_results user_soft_stop_loop(){ return function_results::COMPLETE; };
        virtual function_results user_soft_stop_flagged_timer_update(){ return function_results::COMPLETE; };
        virtual function_results user_soft_stop_flagged_all_adc_eoc(){ return function_results::COMPLETE; };
        virtual function_results user_soft_stop_flagged_systick(){ return function_results::COMPLETE; };


    private:
        motor_channel* mtr_ch;
        static Messaging* msg;
        static time_interface* time;

        states current_state = states::IDLE;
        states requested_state = states::IDLE;

        uint32_t pwm_cycle_counter = 0;
        uint32_t adc_cycle_counter = 0;
        uint32_t systick_counter = 0;

        enum class pwm_startup_states {
            IDLE,
            WAIT_FOR_AUX_ADC_READINGS,
            VERIFY_CHECKS,
            ENABLE_LOW_SIDE,
            WAIT_GATE_DRIVE_CHARGE,
            DONE
        } start_state = pwm_startup_states::IDLE;

        void next_state(function_results result);

        function_results idle_flagged_all_adc_eoc();
        function_results start_flagged_all_adc_eoc();
        function_results run_flagged_all_adc_eoc();
        function_results soft_stop_flagged_all_adc_eoc();
        function_results hard_stop_flagged_all_adc_eoc();
        function_results fault_flagged_all_adc_eoc();

        



};