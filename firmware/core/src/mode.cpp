#include "mode.h"

mode::mode(motor_channel* mtr_ch_){
    mtr_ch = mtr_ch_;
}

void mode::init() {
}

void mode::loop(){

}

void mode::flagged_timer_update(){

    pwm_cycle_counter++;
}

void mode::flagged_all_adc_eoc(){
    switch (current_state) {
        case states::IDLE:
            next_state(idle_flagged_all_adc_eoc());
            break;
        case states::START:
            next_state(start_flagged_all_adc_eoc());
            break;
        case states::RUN:
            next_state(run_flagged_all_adc_eoc());
            break;
        case states::SOFT_STOP:
            next_state(soft_stop_flagged_all_adc_eoc());
            break;
        case states::HARD_STOP:
            next_state(hard_stop_flagged_all_adc_eoc());
            break;
        case states::FAULT:
            next_state(fault_flagged_all_adc_eoc());
            break;
        default:
            next_state(function_results::ERROR);
    }
    adc_cycle_counter++;
}

void mode::flagged_systick(){

    systick_counter++;
}

mode::function_results mode::idle_flagged_all_adc_eoc() {
    // no default functionality
    return function_results::COMPLETE;
}

mode::function_results mode::start_flagged_all_adc_eoc() {
    // enable PWM at 0% duty cycle for all phases
    
    switch(start_state) {
        case pwm_startup_states::IDLE:
        {
            start_state = pwm_startup_states::VERIFY_CHECKS;
            break;
        }
        case pwm_startup_states::VERIFY_CHECKS:
        {
            bool safety_ok = true;
            safety_ok = mtr_ch->global_safety_checks();
            safety_ok = mtr_ch->channel_safety_checks() && safety_ok;
            if(!safety_ok) {
                start_state = pwm_startup_states::IDLE;
                return function_results::ERROR;
            }
            else{
                start_state = pwm_startup_states::ENABLE_LOW_SIDE;
            }
            break;
        }
        case pwm_startup_states::ENABLE_LOW_SIDE:
        {
            // enable low side switches
            mtr_ch->set_scaled_pwm_values(-32768, -32768, -32768); // set all phases to 0% duty cycle
            if(mtr_ch->main_output_enable() != 0){
                start_state = pwm_startup_states::IDLE;
                return function_results::ERROR;
            }
            adc_cycle_counter = 0;
            start_state = pwm_startup_states::WAIT_GATE_DRIVE_CHARGE;
            break;
        }
        case pwm_startup_states::WAIT_GATE_DRIVE_CHARGE:
        {
        // wait for gate drive to charge
            if(adc_cycle_counter >= 100) { // TODO: make this a configurable parameter
                start_state = pwm_startup_states::DONE;
            }
            break;
        }
        case pwm_startup_states::DONE:
        {
            return function_results::COMPLETE;
            break;
        }
    }

    return function_results::CONTINUE;
}

mode::function_results mode::run_flagged_all_adc_eoc() {
    // no default functionality
    return function_results::CONTINUE;
}

mode::function_results mode::soft_stop_flagged_all_adc_eoc() {
    // no default functionality
    return function_results::COMPLETE;
}

mode::function_results mode::hard_stop_flagged_all_adc_eoc() {
    // no default functionality
    return function_results::COMPLETE;
}

mode::function_results mode::fault_flagged_all_adc_eoc() {
    // no default functionality
    return function_results::COMPLETE;
}

void mode::reset() {
    current_state = states::IDLE;
    requested_state = states::IDLE;
    
    pwm_cycle_counter = 0;
    adc_cycle_counter = 0;
    systick_counter = 0;

    start_state = pwm_startup_states::IDLE;

    mtr_ch->reset_faults();

    user_reset();
}

void mode::next_state(mode::function_results result) {
    states next_state = current_state;

    if(result == function_results::COMPLETE) {
        switch(current_state){
            case states::IDLE:
                switch(requested_state){
                    case states::IDLE:
                        next_state = states::IDLE;
                        break;
                    case states::RUN:
                        next_state = states::START; // must go through START first
                        break;
                    default:
                        next_state = states::INVALID_STATE_TRANSITION;
                        break;
                }
                break;
            case states::START:
                switch(requested_state){
                    case states::RUN:
                        next_state = states::RUN;
                        break;
                    case states::IDLE:
                    case states::SOFT_STOP:
                    case states::HARD_STOP:
                        next_state = states::HARD_STOP;
                        break;
                    default:
                        next_state = states::INVALID_STATE_TRANSITION;
                        break;
                }
                break;
            case states::RUN:
                switch(requested_state){
                    case states::RUN:   // leave RUN state if it returns COMPLETE, user logic is done
                    case states::IDLE:
                    case states::SOFT_STOP:
                        next_state = states::SOFT_STOP;
                        break;
                    case states::HARD_STOP:
                        next_state = states::HARD_STOP;
                        break;
                    default:
                        next_state = states::INVALID_STATE_TRANSITION;
                        break;
                }
                break;
            case states::SOFT_STOP:
                next_state = states::IDLE;
                break;
            case states::HARD_STOP:
                next_state = states::IDLE;
                break;
            case states::FAULT:
                switch(requested_state){
                    case states::IDLE:
                        next_state = states::IDLE;
                        break;
                    default:
                        next_state = states::INVALID_STATE_TRANSITION;
                        break;
                }
                break;
        }
    }

    // run even if current function is not complete
    switch(current_state){
        case states::IDLE:
            switch(requested_state){
                case states::IDLE:
                    next_state = states::IDLE;
                    break;
            }
            break;
        case states::START:
            switch(requested_state){
                case states::IDLE:
                case states::HARD_STOP:
                case states::SOFT_STOP: // soft stop is same as hard stop when in START state
                    next_state = states::HARD_STOP;
                    break;
            }
            break;
        case states::RUN:
            switch(requested_state){
                case states::IDLE:
                case states::SOFT_STOP:
                    next_state = states::SOFT_STOP;
                    break;
                case states::HARD_STOP:
                    next_state = states::HARD_STOP;
                    break;
            }
            break;
    }

    if(next_state == states::INVALID_STATE_TRANSITION || result == function_results::ERROR) {
        current_state = states::FAULT;
    }
    else {
        current_state = next_state;
    }
}