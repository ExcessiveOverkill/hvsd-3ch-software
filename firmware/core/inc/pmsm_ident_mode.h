#pragma once

#include "mode.h"

/*
Identify PMSM parameters for an unknown motor.
*/

class pmsm_ident_mode : public mode {
    public:
        pmsm_ident_mode(motor_channel* mtr_ch_, Messaging* msg_, time_interface* time_);

    private:

        bool enable_resistance_measurement = false;
        bool enable_inductance_measurement = false;
        bool enable_commutation_offset_measurement = false;
        bool enable_back_emf_measurement = false;

        float measured_resistance[3] = {0.0f, 0.0f, 0.0f};  // ohms
        float measured_inductance[3] = {0.0f, 0.0f, 0.0f};  // henries
        float measured_commutation_offset = 0.0f;   // radians
        float measured_back_emf = 0.0f; // v / 1000 rpm

        float motor_calibration_current = 0.0f; // amps

        void user_reset() override;

        // function_results user_start_loop() override;
        // function_results user_start_flagged_timer_update() override;
        // function_results user_start_flagged_all_adc_eoc() override;
        // function_results user_start_flagged_systick() override;

        function_results user_run_loop() override;
        function_results user_run_flagged_timer_update() override;
        function_results user_run_flagged_all_adc_eoc() override;
        function_results user_run_flagged_systick() override;

        // function_results user_soft_stop_loop() override;
        // function_results user_soft_stop_flagged_timer_update() override;
        // function_results user_soft_stop_flagged_all_adc_eoc() override;
        // function_results user_soft_stop_flagged_systick() override;
        
};