#include "motor_channel.h"

#include "board_hw.h"
#include "gpio_ll.h"
#include "math.h"

motor_channel::motor_channel(uint8_t channel_num, const board_hw::motor_hw_config* config_, Messaging* msg_, time_interface* time_) {
    motor_channel::time = time_;
    motor_channel::msg = msg_;
    this->config = config_;
    this->channel_num = channel_num;
    motor_channel::motor_msgs = msgs.motor[channel_num];
    num_of_channels++;
}

void motor_channel::init() {
    sto_init();
    ipm_fault_init();
    phase_adc_init();
    timer_init();
}

void motor_channel::phase_adc_init() {
    
    ADC_TypeDef* adcs[] = {board_hw::phase_u_adc, board_hw::phase_v_adc, board_hw::phase_w_adc};

    if(channel_num == 0){
        // config for all channels once, since ADC instances are shared between channels

        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // enable SYSCFG clock for VREFBUF

        // enable VREF and vref half opamp
        if(board_hw::use_external_vref == false){
            // enable VREFBUF
            if(board_hw::internal_vref_voltage == 2.048f) {
                VREFBUF->CSR |= 0b00 << VREFBUF_CSR_VRS_Pos; // set to 2.048v
            } else if(board_hw::internal_vref_voltage == 2.5f) {
                VREFBUF->CSR |= 0b01 << VREFBUF_CSR_VRS_Pos; // set to 2.5v
            } else if(board_hw::internal_vref_voltage == 2.90f) {
                VREFBUF->CSR |= 0b10 << VREFBUF_CSR_VRS_Pos; // set to 2.90v
            } else {
                // TODO: handle error
                while(1){}
            }
            VREFBUF->CSR &= ~VREFBUF_CSR_HIZ; // disable high impedance mode to power on the buffer
            VREFBUF->CSR |= VREFBUF_CSR_ENVR; // enable VREFBUF output

            while(!(VREFBUF->CSR & VREFBUF_CSR_VRR)); // wait for VREFBUF to stabilize
            // TODO: add timeout to prevent infinite loop in case of hardware failure
        }

        enable_adc_clock(board_hw::phase_u_adc);
        enable_adc_clock(board_hw::phase_v_adc);
        enable_adc_clock(board_hw::phase_w_adc);

        gpio_ll::configure_analog({board_hw::vref_half_0_adc_port, board_hw::vref_half_0_adc_pin});
        gpio_ll::configure_analog({board_hw::vref_half_1_adc_port, board_hw::vref_half_1_adc_pin});
        gpio_ll::configure_analog({board_hw::vref_half_2_adc_port, board_hw::vref_half_2_adc_pin});

        // wait for vref/2 to stabilize
        uint64_t start_time = time->get_microseconds_update();
        while(time->get_microseconds_update() < start_time+10e3){} // wait 10ms for vref/2 to stabilize, based on board hardware testing


        // configure ADCs
        for (ADC_TypeDef* adc : adcs) {

            power_on_adc(adc);

            // set ADC regular conversion sequence to handle 3 channels
            adc->SQR1 = 2 << ADC_SQR1_L_Pos; // 3 conversions in the sequence
            
            // enable discontinuous mode to only do one channel per trigger
            adc->CFGR &= ~ADC_CFGR_CONT; // disable continuous mode
            adc->CFGR |= ADC_CFGR_DISCEN;   // enable discontinuous mode
            adc->CFGR &= ~ADC_CFGR_DISCNUM; // 1 conversion in discontinuous sequence

            adc->CFGR |= ADC_CFGR_OVRMOD; // enable overrun mode to overwrite old data if not read in time

            adc->CFGR2 |= ADC_CFGR2_ROVSE;  // enable oversampling
            // configure_adc_oversampling(adc, 1); // 2x minimum oversampling
            phase_adc_result_left_shift = configure_adc_oversampling(adc, 2);

            // phase current channels are configured per-motor below so all ranks get explicit sample times
            uint32_t cycles = 2;
            configure_adc_sample_time(adc, config->phase_u_adc_channel, cycles);
            configure_adc_sample_time(adc, config->phase_v_adc_channel, cycles);
            configure_adc_sample_time(adc, config->phase_w_adc_channel, cycles);

            cycles = 24;
            if(adc == board_hw::phase_u_adc){
                configure_adc_sample_time(adc, board_hw::vref_half_0_adc_channel, cycles);
            }
            else if(adc == board_hw::phase_v_adc){
                configure_adc_sample_time(adc, board_hw::vref_half_1_adc_channel, cycles);
            }
            else if(adc == board_hw::phase_w_adc){
                configure_adc_sample_time(adc, board_hw::vref_half_2_adc_channel, cycles);
            }


            // get vref half offset for each phase ADC
            uint16_t phase_offset = 0;
            enable_phase_adcs();
            if(adc == board_hw::phase_u_adc){
                    phase_u_vref_offset = get_adc_reading_blocking(adc, board_hw::vref_half_0_adc_channel);
                    phase_offset = phase_u_vref_offset;
            }
            else if(adc == board_hw::phase_v_adc){
                    phase_v_vref_offset = get_adc_reading_blocking(adc, board_hw::vref_half_1_adc_channel);
                    phase_offset = phase_v_vref_offset;
            }
            else if(adc == board_hw::phase_w_adc){
                    phase_w_vref_offset = get_adc_reading_blocking(adc, board_hw::vref_half_2_adc_channel);
                    phase_offset = phase_w_vref_offset;
            }
            disable_phase_adcs();


            // watchdog config
            uint32_t channels = (1u << config->phase_u_adc_channel) | (1u << config->phase_v_adc_channel) | (1u << config->phase_w_adc_channel);
            configure_adc_watchdog(adc, board_hw::ipms.max_phase_current_amps, phase_offset, channels);
        
            // enable interrupts
            adc->IER |= ADC_IER_EOCIE; // enable end of conversion interrupt
            IRQn_Type irq;
            if(adc == ADC1 || adc == ADC2){
                irq = ADC1_2_IRQn;
            }
            else if(adc == ADC3){
                irq = ADC3_IRQn;
            }
            else if(adc == ADC4){
                irq = ADC4_IRQn;
            }
            else if(adc == ADC5){
                irq = ADC5_IRQn;
            }
            NVIC_SetPriority(irq, 6); // medium priority
            NVIC_EnableIRQ(irq);
        }

        vbus_sense_adc_init();
        aux_adc_init();
    
    }

    // motor channel specific config

    // configure ADC GPIOs
    gpio_ll::configure_analog({config->phase_u_adc_port, config->phase_u_adc_pin});
    gpio_ll::configure_analog({config->phase_v_adc_port, config->phase_v_adc_pin});
    gpio_ll::configure_analog({config->phase_w_adc_port, config->phase_w_adc_pin});

    
    // configure channel sample sequence, always goes in the channel number order
    uint8_t sequence_offset = 0;
    uint32_t sequence_mask = 0;
    switch (channel_num) {
        case 0:
            sequence_offset = ADC_SQR1_SQ1_Pos;
            sequence_mask = ADC_SQR1_SQ1_Msk;
            break;
        case 1:
            sequence_offset = ADC_SQR1_SQ2_Pos;
            sequence_mask = ADC_SQR1_SQ2_Msk;
            break;
        case 2:
            sequence_offset = ADC_SQR1_SQ3_Pos;
            sequence_mask = ADC_SQR1_SQ3_Msk;
            break;
        default:
            // TODO: add fault handler
            while(1){}
    }

    board_hw::phase_u_adc->SQR1 = (board_hw::phase_u_adc->SQR1 & ~sequence_mask)
                                | (static_cast<uint32_t>(config->phase_u_adc_channel) << sequence_offset);
    board_hw::phase_v_adc->SQR1 = (board_hw::phase_v_adc->SQR1 & ~sequence_mask)
                                | (static_cast<uint32_t>(config->phase_v_adc_channel) << sequence_offset);
    board_hw::phase_w_adc->SQR1 = (board_hw::phase_w_adc->SQR1 & ~sequence_mask)
                                | (static_cast<uint32_t>(config->phase_w_adc_channel) << sequence_offset);

    // Ensure each used phase channel has an explicit sample time across all motor configs.
    uint32_t phase_cycles = 2;
    configure_adc_sample_time(board_hw::phase_u_adc, config->phase_u_adc_channel, phase_cycles);
    configure_adc_sample_time(board_hw::phase_v_adc, config->phase_v_adc_channel, phase_cycles);
    configure_adc_sample_time(board_hw::phase_w_adc, config->phase_w_adc_channel, phase_cycles);
}

void motor_channel::aux_adc_init() {
    // configure auxiliary ADC for gate supply voltage and IPM IC temperature
    // must be called after main ADC init
    ADC_TypeDef* adc = board_hw::aux_adc;

    enable_adc_clock(adc);
    power_on_adc(adc);

    // set ADC regular conversion sequence to handle 7 channels
    adc->SQR1 = ((7u - 1u) << ADC_SQR1_L_Pos)
              | (static_cast<uint32_t>(board_hw::gate_supply_sense_channel) << ADC_SQR1_SQ1_Pos)
              | (static_cast<uint32_t>(board_hw::inverter_a_therm_channel) << ADC_SQR1_SQ2_Pos)
              | (static_cast<uint32_t>(board_hw::inverter_b_therm_channel) << ADC_SQR1_SQ3_Pos)
              | (static_cast<uint32_t>(board_hw::inverter_c_therm_channel) << ADC_SQR1_SQ4_Pos);
    adc->SQR2 = (static_cast<uint32_t>(board_hw::inverter_a_temp_channel) << ADC_SQR2_SQ5_Pos)
              | (static_cast<uint32_t>(board_hw::inverter_b_temp_channel) << ADC_SQR2_SQ6_Pos)
              | (static_cast<uint32_t>(board_hw::inverter_c_temp_channel) << ADC_SQR2_SQ7_Pos);
    
    // enable discontinuous mode to only do one channel per trigger
    adc->CFGR &= ~ADC_CFGR_CONT; // disable continuous mode
    adc->CFGR |= ADC_CFGR_DISCEN;   // enable discontinuous mode
    adc->CFGR &= ~ADC_CFGR_DISCNUM; // 1 conversion in discontinuous sequence

    adc->CFGR |= ADC_CFGR_OVRMOD; // enable overrun mode to overwrite old data if not read in time

    // enable oversampling
    adc->CFGR2 |= ADC_CFGR2_ROVSE;
    aux_adc_result_left_shift = configure_adc_oversampling(adc, 8); // 256x oversampling for auxiliary ADCs to reduce switching noise since they are not synced with PWM timers

    // configure GPIOs for auxiliary ADCs
    gpio_ll::configure_analog({board_hw::gate_supply_sense_port, board_hw::gate_supply_sense_pin});
    gpio_ll::configure_analog({board_hw::inverter_a_therm_port, board_hw::inverter_a_therm_pin});
    gpio_ll::configure_analog({board_hw::inverter_b_therm_port, board_hw::inverter_b_therm_pin});
    gpio_ll::configure_analog({board_hw::inverter_c_therm_port, board_hw::inverter_c_therm_pin});
    gpio_ll::configure_analog({board_hw::inverter_a_temp_port, board_hw::inverter_a_temp_pin});
    gpio_ll::configure_analog({board_hw::inverter_b_temp_port, board_hw::inverter_b_temp_pin});
    gpio_ll::configure_analog({board_hw::inverter_c_temp_port, board_hw::inverter_c_temp_pin});

    // configure sample times for auxiliary ADC channels
    uint32_t cycles = 247;  // needs a long sample time due to high impedance
    configure_adc_sample_time(adc, board_hw::gate_supply_sense_channel, cycles);
    configure_adc_sample_time(adc, board_hw::inverter_a_therm_channel, cycles);
    configure_adc_sample_time(adc, board_hw::inverter_b_therm_channel, cycles);
    configure_adc_sample_time(adc, board_hw::inverter_c_therm_channel, cycles);
    configure_adc_sample_time(adc, board_hw::inverter_a_temp_channel, cycles);
    configure_adc_sample_time(adc, board_hw::inverter_b_temp_channel, cycles);
    configure_adc_sample_time(adc, board_hw::inverter_c_temp_channel, cycles);

    // enable interrupts
    adc->IER |= ADC_IER_EOCIE; // enable end of conversion interrupt
    IRQn_Type irq;
    if(adc == ADC1 || adc == ADC2){
        irq = ADC1_2_IRQn;
    }
    else if(adc == ADC3){
        irq = ADC3_IRQn;
    }
    else if(adc == ADC4){
        irq = ADC4_IRQn;
    }
    else if(adc == ADC5){
        irq = ADC5_IRQn;
    }
    NVIC_SetPriority(irq, 7); // medium priority
    NVIC_EnableIRQ(irq);

}

void motor_channel::sto_init() {
    // configure STO fault detection
    // must be called after main ADC init

    // STO ch 1 for timer break inputs
    gpio_ll::enable_port_clock(config->break_port);

    // STO ch 2 for IPM disable
    gpio_ll::enable_port_clock(board_hw::inverter_disable_port);
}

void motor_channel::vbus_sense_adc_init() {
    // configure VBUS sense ADC for voltage measurement
    // must be called after main ADC init
    ADC_TypeDef* adc = board_hw::vbus_sense_adc;

    // uses same ADC as phase sense, so we use auto injected conversion to read VBUS sense after phase sense conversion

    // adc->CFGR |= ADC_CFGR_JAUTO; // enable auto-injected conversion mode

    // adc->CFGR |= ADC_CFGR_JDISCEN;   // enable discontinuous mode
    adc->CFGR2 |= ADC_CFGR2_JOVSE;  // enable oversampling for injected conversions

    adc->JSQR |= board_hw::vbus_sense_channel << ADC_JSQR_JSQ1_Pos; // set injected channel to VBUS sense

    // ADC input is from an internal opamp, so we don't need to configure any GPIO

    // configure sample time
    uint32_t cycles = 12;
    configure_adc_sample_time(adc, board_hw::vbus_sense_channel, cycles);

    adc->IER |= ADC_IER_JEOCIE; // enable end of injected conversion interrupt

    // irq is already enabled in phase_adc_init() since it uses the same ADC instance
}

void motor_channel::trigger_vbus_sense_adc_reading() {
    // trigger an injected conversion for VBUS sense ADC
    ADC_TypeDef* adc = board_hw::vbus_sense_adc;
    adc->CR |= ADC_CR_JADSTART; // start injected conversion
}

uint16_t motor_channel::get_adc_reading_blocking(ADC_TypeDef* adc, uint32_t channel){
    
    // configure the ADC to read the specified channel and wait for the conversion to complete
    // this is a blocking function and should be used sparingly

    // set the channel in the regular sequence
    adc->SQR1 = (adc->SQR1 & ~ADC_SQR1_SQ1_Msk) | (channel << ADC_SQR1_SQ1_Pos);

    // start conversion
    adc->CR |= ADC_CR_ADSTART;

    // wait for conversion to complete
    while(!(adc->ISR & ADC_ISR_EOC)) {}

    // read the result
    uint16_t result = adc->DR << phase_adc_result_left_shift; // left shift to align with 16-bit full scale after oversampling

    // clear the EOC flag
    adc->ISR = ADC_ISR_EOC;

    return result;
}

void motor_channel::reset_analog_watchdogs() {
    // reset the analog watchdog status flags for phase current ADCs
    board_hw::phase_u_adc->ISR |= ADC_ISR_AWD2;
    board_hw::phase_v_adc->ISR |= ADC_ISR_AWD2;
    board_hw::phase_w_adc->ISR |= ADC_ISR_AWD2;
}

bool motor_channel::get_analog_watchdog_status() {
    // check if any of the phase current ADCs have triggered the analog watchdog
    return (board_hw::phase_u_adc->ISR & ADC_ISR_AWD2) || (board_hw::phase_v_adc->ISR & ADC_ISR_AWD2) || (board_hw::phase_w_adc->ISR & ADC_ISR_AWD2);
}

void motor_channel::enable_adc_clock(ADC_TypeDef* adc) {
    uint8_t ckmode = 0;
    switch(board_hw::adc_clk_div) {
        case 1:
            ckmode = 0b01; // no division
            break;
        case 2:
            ckmode = 0b10; // divide by 2
            break;
        case 4:
            ckmode = 0b11; // divide by 4
            break;
        default:
            // unsupported clock division, TODO: handle error
            while(1){}
    }
    if(adc == ADC1 || adc == ADC2){
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
        ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~ADC_CCR_CKMODE)
                          | (static_cast<uint32_t>(ckmode) << ADC_CCR_CKMODE_Pos);
    }
    else if(adc == ADC3 || adc == ADC4 || adc == ADC5){
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
        ADC345_COMMON->CCR = (ADC345_COMMON->CCR & ~ADC_CCR_CKMODE)
                           | (static_cast<uint32_t>(ckmode) << ADC_CCR_CKMODE_Pos);
    }
    else{
        // unsupported ADC, TODO: handle error
        while(1){}
    }
}

void motor_channel::power_on_adc(ADC_TypeDef* adc) {
    // power on ADC and wait for it to be ready
    adc->CR &= ~ADC_CR_DEEPPWD; // exit deep power down
    adc->CR |= ADC_CR_ADVREGEN;  // enable ADC voltage regulator

    // wait for ADC voltage regulator to stabilize, 20us according to  DS12712 table 61
    // delay based on systick counter since we don't have any other timers running at this point
    uint32_t start_tick = SysTick->VAL;
    uint32_t target_delay = board_hw::systick_clk_source_hz / 1000000 * 20;
    while(true) {
        uint32_t current_tick = SysTick->VAL;
        uint32_t elapsed = (start_tick >= current_tick) ? (start_tick - current_tick) : (start_tick + (SysTick->LOAD - current_tick));
        if(elapsed >= target_delay) break;
    }

    // calibrate (ADC must be disabled)
    if(adc->CR & ADC_CR_ADEN) {
        adc->CR |= ADC_CR_ADDIS;
        while(adc->CR & ADC_CR_ADEN) {}
    }
    adc->CR &= ~ADC_CR_ADCALDIF; // single-ended calibration
    adc->CR |= ADC_CR_ADCAL; // start calibration
    while (adc->CR & ADC_CR_ADCAL); // wait for calibration to complete
}

void motor_channel::configure_adc_watchdog(ADC_TypeDef* adc, float current, uint16_t center_offset, uint32_t channels) {
    // configure ADC analog watchdog to monitor specified channels

    // AWD hardware thresholds are compared to the ADC data register, after oversampling and right shifting (not left shifting)

    center_offset >>= phase_adc_result_left_shift; // right shift to switch back to raw ADC counts

    uint16_t upper_threshold = static_cast<uint16_t>(calculate_adc_counts_from_current(current) + center_offset);
    uint16_t lower_threshold = static_cast<uint16_t>(center_offset - calculate_adc_counts_from_current(current));
    
    adc->AWD2CR = channels & 0x3FFFF; // monitor specified channels

    // watchdog only uses upper 8 bits of data, so shift threshold to match ADC resolution
    // note that this reduces the effective resolution

    upper_threshold >>= 8;
    lower_threshold >>= 8;
    adc->TR2 = (upper_threshold << ADC_TR2_HT2_Pos) | (lower_threshold << ADC_TR2_LT2_Pos); // set both low and high thresholds to the same value to trigger on a single threshold crossing
}

uint8_t motor_channel::configure_adc_oversampling(ADC_TypeDef* adc, uint8_t oversampling_ratio) {
    // configure ADC oversampling with specified ratio (e.g. 2, 4, 8, 16, up to 256.)
    // oversampling can improve effective resolution and SNR at the cost of slower sampling rate
    // actual decimation factor is 2^oversampling_ratio, so valid input values are 1-8 for ratios of 2-256

    if(adc->CR & ADC_CR_ADSTART) {
        // ADC is currently converting, cannot change oversampling ratio
        // TODO: handle error
        while(1){}
    }

    if (oversampling_ratio < 1 || oversampling_ratio > 8) {
        // invalid oversampling ratio, TODO: handle error
        return 0; // return 0 as default shift for invalid oversampling ratio
    }

    // we always want 16 bit in the output register, so we need to calculate the appropriate shift based on the oversampling ratio and ADC resolution
    int8_t shift = 0;
    shift = oversampling_ratio + (adc_resolution_bits - 16);

    if(shift < 0) {
        // oversampling ratio is not high enough to achieve 16 bit resolution, so we will have to left shift the result manually after reading from the data register
        adc->CFGR2 &= ~ADC_CFGR2_OVSS; // clear oversampling shift
        adc->CFGR2 |= (0) << ADC_CFGR2_OVSS_Pos; // set shift to 0, we will handle it in software after reading the result
        shift = -shift; // store number of bits to left shift result for later
    } else {
        // if shift is positive, we can right shift the result in hardware
        adc->CFGR2 &= ~ADC_CFGR2_OVSS; // clear oversampling shift
        adc->CFGR2 |= (shift) << ADC_CFGR2_OVSS_Pos; // set shift
        shift = 0; // no need for software shift
    }

    adc->CFGR2 &= ~ADC_CFGR2_OVSR; // clear oversampling ratio
    adc->CFGR2 |= (oversampling_ratio - 1) << ADC_CFGR2_OVSR_Pos; // set oversampling ratio

    return shift; // return number of bits to left shift result in software if needed

}

void motor_channel::configure_adc_sample_time(ADC_TypeDef* adc, uint32_t channel, uint32_t sample_time_cycles) {
    // configure ADC sample time for specified channel
    // sample time is in ADC clock cycles, and affects the input impedance and noise performance of the measurement
    // longer sample times can improve accuracy for high impedance sources but reduce maximum sampling rate
    // valid sample times are 2.5, 6.5, 12.5, 24.5, 47.5, 92.5, 247.5, and 640.5 cycles

    uint32_t smpr_bits = 0;
    switch (sample_time_cycles) {
        case 2:
            smpr_bits = 0b000;
            break;
        case 6:
            smpr_bits = 0b001;
            break;
        case 12:
            smpr_bits = 0b010;
            break;
        case 24:
            smpr_bits = 0b011;
            break;
        case 47:
            smpr_bits = 0b100;
            break;
        case 92:
            smpr_bits = 0b101;
            break;
        case 247:
            smpr_bits = 0b110;
            break;
        case 640:
            smpr_bits = 0b111;
            break;
        default:
            // invalid sample time, TODO: handle error
            return;
    }

    if (channel <= 9) {
        adc->SMPR1 &= ~(0b111 << (channel * 3)); // clear existing sample time bits for the channel
        adc->SMPR1 |= smpr_bits << (channel * 3); // set new sample time bits
    } else if (channel <= 18) {
        uint8_t ch = channel - 10; // channels above 9 are in SMPR2 with an offset of -10
        adc->SMPR2 &= ~(0b111 << (ch * 3)); // clear existing sample time bits for the channel
        adc->SMPR2 |= smpr_bits << (ch * 3); // set new sample time bits
    } else {
        // TODO: handle error

        while (1){} // invalid channel number
        
    }
}

void motor_channel::enable_phase_adc_timer_trigger(){
    // enable hardware trigger from timer for ADC conversions to synchronize with PWM
    // only one timer can be selected at a time, so this must be called after each ADC sample to move to the next motor
    board_hw::phase_u_adc->CFGR = (board_hw::phase_u_adc->CFGR & ~ADC_CFGR_EXTSEL) | (config->phase_u_adc_timer_reg_extsel << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_EXTEN_0; // rising edge trigger
    board_hw::phase_v_adc->CFGR = (board_hw::phase_v_adc->CFGR & ~ADC_CFGR_EXTSEL) | (config->phase_v_adc_timer_reg_extsel << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_EXTEN_0; // rising edge trigger
    board_hw::phase_w_adc->CFGR = (board_hw::phase_w_adc->CFGR & ~ADC_CFGR_EXTSEL) | (config->phase_w_adc_timer_reg_extsel << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_EXTEN_0; // rising edge trigger

    manual_phase_adc_trigger();

}

void motor_channel::disable_phase_adc_timer_trigger(){
    // disable hardware trigger from timer for ADC conversions, switching back to software trigger mode

    if(board_hw::phase_u_adc->CR & ADC_CR_ADSTART) {
        board_hw::phase_u_adc->CR |= ADC_CR_ADSTP;
        while(board_hw::phase_u_adc->CR & ADC_CR_ADSTART) {}
    }
    if(board_hw::phase_v_adc->CR & ADC_CR_ADSTART) {
        board_hw::phase_v_adc->CR |= ADC_CR_ADSTP;
        while(board_hw::phase_v_adc->CR & ADC_CR_ADSTART) {}
    }
    if(board_hw::phase_w_adc->CR & ADC_CR_ADSTART) {
        board_hw::phase_w_adc->CR |= ADC_CR_ADSTP;
        while(board_hw::phase_w_adc->CR & ADC_CR_ADSTART) {}
    }
    
    board_hw::phase_u_adc->CFGR &= ~ADC_CFGR_EXTEN; // disable external trigger
    board_hw::phase_v_adc->CFGR &= ~ADC_CFGR_EXTEN; // disable external trigger
    board_hw::phase_w_adc->CFGR &= ~ADC_CFGR_EXTEN; // disable external trigger
}

void motor_channel::manual_phase_adc_trigger() {
    // trigger ADC conversion manually in software, used when timer trigger is disabled
    if(!(board_hw::phase_u_adc->CR & ADC_CR_ADSTART)) {
        board_hw::phase_u_adc->CR |= ADC_CR_ADSTART; // start conversion
    }
    if(!(board_hw::phase_v_adc->CR & ADC_CR_ADSTART)) {
        board_hw::phase_v_adc->CR |= ADC_CR_ADSTART; // start conversion
    }
    if(!(board_hw::phase_w_adc->CR & ADC_CR_ADSTART)) {
        board_hw::phase_w_adc->CR |= ADC_CR_ADSTART; // start conversion
    }
}

void motor_channel::manual_aux_adc_trigger() {
    // trigger auxiliary ADC conversion manually in software
    board_hw::aux_adc->CR |= ADC_CR_ADSTART; // start conversion
}

void motor_channel::timer_init() {
    // configure timer for PWM generation on the specified channels for this motor channel
    // timer and channel mapping is defined in the hw_config struct

    // init gpio
    // U
    gpio_ll::configure_alternate({config->phase_u_low_port, config->phase_u_low_pin}, config->phase_u_low_pin_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::medium);
    gpio_ll::configure_alternate({config->phase_u_high_port, config->phase_u_high_pin}, config->phase_u_high_pin_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::medium);
    // V
    gpio_ll::configure_alternate({config->phase_v_low_port, config->phase_v_low_pin}, config->phase_v_low_pin_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::medium);
    gpio_ll::configure_alternate({config->phase_v_high_port, config->phase_v_high_pin}, config->phase_v_high_pin_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::medium);
    // W
    gpio_ll::configure_alternate({config->phase_w_low_port, config->phase_w_low_pin}, config->phase_w_low_pin_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::medium);
    gpio_ll::configure_alternate({config->phase_w_high_port, config->phase_w_high_pin}, config->phase_w_high_pin_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::medium);
    // Break
    gpio_ll::configure_alternate({config->break_port, config->break_pin}, config->break_pin_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::medium);
    gpio_ll::configure_alternate({config->break_2_port, config->break_2_pin}, config->break_2_pin_alternate_function, gpio_ll::Pull::none, gpio_ll::Speed::medium);

    // enable timer clock
    if(config->timer == TIM1){
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
    else if(config->timer == TIM8){
        RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    }
    else if(config->timer == TIM20){
        RCC->APB2ENR |= RCC_APB2ENR_TIM20EN;
    }
    else{
        // not a motor control timer, TODO: handle error
        while(1){}
    }


    // configure timer
    config->timer->CR1 |= 0b11 << TIM_CR1_CMS_Pos; // center-aligned mode 2 (ouput compare flags on up and down counting)
    config->timer->CR1 |= TIM_CR1_ARPE; // enable auto-reload preload

    config->timer->CR2 |= 0b0111 << TIM_CR2_MMS_Pos; // set master mode to compare tim_oc4refc (trgo) for ADC triggering
    config->timer->SMCR |= ((config->timer_itr & 0b111) << TIM_SMCR_TS_Pos) | (((config->timer_itr >> 3) & 0b11) << 20); // set internal trigger from sync timer (timer_itr) for syncing multiple timers together
    config->timer->SMCR |= 0b0110 << TIM_SMCR_SMS_Pos; // set slave mode to trigger mode

    // setup update interrupt
    config->timer->DIER |= TIM_DIER_UIE; // enable update interrupt
    IRQn_Type irq_num;
    if(config->timer == TIM1){
        irq_num = TIM1_UP_TIM16_IRQn;
    }
    else if(config->timer == TIM8){
        irq_num = TIM8_UP_IRQn;
    }
    else if(config->timer == TIM20){
        irq_num = TIM20_UP_IRQn;
    }
    else{
        // not a motor control timer, TODO: handle error
        while(1){}
    }
    NVIC_SetPriority(irq_num, 7u); // medium priority
    NVIC_EnableIRQ(irq_num);

    // TODO: assuming channels 1, 2, and 3 are used for motor phases, might want to make this use the config in the future
    config->timer->CCMR1 |= 0b1 << TIM_CCMR1_OC1PE_Pos; // enable output compare 1 preload
    config->timer->CCMR1 |= 0b0111 << TIM_CCMR1_OC1M_Pos; // set output compare 1 mode to PWM mode 2
    config->timer->CCMR1 |= 0b1 << TIM_CCMR1_OC2PE_Pos; // enable output compare 2 preload
    config->timer->CCMR1 |= 0b0111 << TIM_CCMR1_OC2M_Pos; // set output compare 2 mode to PWM mode 2
    config->timer->CCMR2 |= 0b1 << TIM_CCMR2_OC3PE_Pos; // enable output compare 3 preload
    config->timer->CCMR2 |= 0b0111 << TIM_CCMR2_OC3M_Pos; // set output compare 3 mode to PWM mode 2

    config->timer->CCMR2 |= 0b1 << TIM_CCMR2_OC4PE_Pos; // enable output compare 4 preload (used for triggering ADC)
    config->timer->CCMR2 |= 0b0111 << TIM_CCMR2_OC4M_Pos; // set output compare 4 mode to PWM mode 2

    config->timer->CCER |= 0b0 << TIM_CCER_CC1P_Pos; // set output compare 1 polarity to active high
    config->timer->CCER |= 0b0 << TIM_CCER_CC2P_Pos; // set output compare 2 polarity to active high
    config->timer->CCER |= 0b0 << TIM_CCER_CC3P_Pos; // set output compare 3 polarity to active high
    // config->timer->CCER |= 0b1 << TIM_CCER_CC1NP_Pos; // set output compare 1 complementary polarity to active low
    // config->timer->CCER |= 0b1 << TIM_CCER_CC2NP_Pos; // set output compare 2 complementary polarity to active low
    // config->timer->CCER |= 0b1 << TIM_CCER_CC3NP_Pos; // set output compare 3 complementary polarity to active low

    config->timer->CCER |= TIM_CCER_CC1E; // enable output compare 1
    config->timer->CCER |= TIM_CCER_CC2E; // enable output compare 2
    config->timer->CCER |= TIM_CCER_CC3E; // enable output compare 3
    config->timer->CCER |= TIM_CCER_CC1NE; // enable output compare 1 complementary
    config->timer->CCER |= TIM_CCER_CC2NE; // enable output compare 2 complementary
    config->timer->CCER |= TIM_CCER_CC3NE; // enable output compare 3 complementary

    config->timer->PSC = board_hw::phase_pwm_timer_prescaler - 1; // set prescaler

    set_pwm_frequency(board_hw::phase_min_pwm_frequency_hz); // set initial PWM frequency to minimum

    config->timer->BDTR |= 0b1 << TIM_BDTR_OSSI_Pos; // enable off-state selection for idle mode
    config->timer->BDTR |= 0b1 << TIM_BDTR_OSSR_Pos; // enable off-state selection for run mode
    config->timer->BDTR |= 0b1000 << TIM_BDTR_BKF_Pos; // set break filter to Fdts/8, 8 cycles
    config->timer->BDTR |= TIM_BDTR_BKP;    // set break polarity to active high (fault when high)
    // config->timer->BDTR |= 0b1000 << TIM_BDTR_BK2F_Pos; // set break filter to Fdts/8, 8 cycles
    config->timer->BDTR |= 0b1 << TIM_BDTR_BKE_Pos; // enable break input

    uint16_t dtg = 0;
    if(board_hw::phase_pwm_timer_deadtime_ticks < 128){
        dtg = board_hw::phase_pwm_timer_deadtime_ticks;
    }
    else if(board_hw::phase_pwm_timer_deadtime_ticks < 256){
        dtg = 0b10000000 | ((board_hw::phase_pwm_timer_deadtime_ticks/2 - 64));
    }
    else if(board_hw::phase_pwm_timer_deadtime_ticks < 512){
        dtg = 0b11000000 | ((board_hw::phase_pwm_timer_deadtime_ticks/8 - 32));
    }
    else if(board_hw::phase_pwm_timer_deadtime_ticks < 1024){
        dtg = 0b11100000 | ((board_hw::phase_pwm_timer_deadtime_ticks/16 - 32));
    }
    else{
        // TODO: handle error, deadtime too long
        while(1){}
    }
    config->timer->BDTR |= dtg << TIM_BDTR_DTG_Pos; // set deadtime

    config->timer->BDTR |= 0b11 << TIM_BDTR_LOCK_Pos; // set lock level 3, prevents changes to the timer configuration until a reset

}

void motor_channel::ipm_fault_init(){
    // configure GPIO for IPM fault input (fault when low)
    gpio_ll::configure_input({config->fault_port, config->fault_pin}, gpio_ll::Pull::up);

    // configure EXTI
    uint32_t exti_line = 1 << config->fault_pin; // EXTI line corresponding to the fault pin

    gpio_ll::configure_exti_line({config->fault_port, config->fault_pin}); // route EXTI line to the fault pin's port (defaults to GPIOA otherwise)

    EXTI->IMR1 |= exti_line; // unmask interrupt
    EXTI->FTSR1 |= exti_line; // trigger on falling edge

    // configure NVIC
    IRQn_Type irq;
    if(config->fault_pin <= 4){
        irq = static_cast<IRQn_Type>(EXTI0_IRQn + config->fault_pin);
    }
    else if(config->fault_pin <= 9){
        irq = EXTI9_5_IRQn;
    }
    else{
        irq = EXTI15_10_IRQn;
    }
    NVIC_SetPriority(irq, 2); // set priority to high
    NVIC_EnableIRQ(irq); // enable the interrupt in NVIC
}

void motor_channel::ipm_exti_irq_handler(){
    // handle IPM fault interrupt

    if(!(EXTI->PR1 & (1 << config->fault_pin))) {
        // not from this channel's fault pin
        return;
    }

    // clear EXTI pending bit
    uint32_t exti_line = 1 << config->fault_pin;
    EXTI->PR1 = exti_line;

    // disable PWM outputs
    main_output_disable();

    // set fault flag
    ipm_fault_detected = true;
    msg->add(motor_msgs.ipm_fault, 0);
}

uint16_t motor_channel::timer_arr_from_frequency(float frequency_hz, float* resulting_frequency_hz) {
    // calculate the auto-reload value for the timer based on the desired PWM frequency
    // note that the timer is in center-aligned mode, so the update event occurs at twice the set frequency (up and down counting)

    if(frequency_hz < board_hw::phase_min_pwm_frequency_hz) {
        frequency_hz = board_hw::phase_min_pwm_frequency_hz;
    }
    else if(frequency_hz > board_hw::phase_max_pwm_frequency_hz) {
        frequency_hz = board_hw::phase_max_pwm_frequency_hz;
    }

    uint32_t timer_clock = board_hw::TIM1_ker_clk; // assuming all motor channels use the same timer clock, might want to make this configurable in the future
    uint16_t arr = ((float)timer_clock / (frequency_hz * 2.0f * (float)board_hw::phase_pwm_timer_prescaler)) - 1;
    if (resulting_frequency_hz) {
        *resulting_frequency_hz = (float)timer_clock / ((float)board_hw::phase_pwm_timer_prescaler * (arr + 1) * 2.0f); // calculate the actual frequency based on the calculated ARR value
    }
    return arr;
}

void motor_channel::main_output_disable() {
    // disable PWM outputs
    config->timer->BDTR &= ~TIM_BDTR_MOE;
}

uint8_t motor_channel::main_output_enable() {
    // enable PWM outputs

    if(!channel_safety_checks()) {
        // safety checks failed, do not enable outputs
        return 4; // return 4 to indicate failure
    }

    if(!global_safety_checks()) {
        // safety checks failed, do not enable outputs
        return 3; // return 3 to indicate failure
    }

    if(!(config->timer->CR1 & TIM_CR1_CEN)) {
        // timer is not running, cannot enable outputs
        return 2; // return 2 to indicate failure
    }

    config->timer->SR &= ~TIM_SR_BIF; // clear break interrupt flag

    if(config->timer->SR & TIM_SR_BIF) {
        // break interrupt flag is still set, indicating a fault condition
        // do not enable outputs (hardware prevents it anyway)
        msg->add(motor_msgs.pwm_break_input_active, 0);
        return 1; // return 1 to indicate failure
    }

    config->timer->BDTR |= TIM_BDTR_MOE;
    return 0; // return 0 to indicate success
}

void motor_channel::stop_timer() {
    // stop the timer, halting PWM generation
    main_output_disable();
    config->timer->CR1 &= ~TIM_CR1_CEN;
}

void motor_channel::start_timer_unsynced() {
    // start the timer, outputs not enabled yet
    // not synchronized with other timers
    stop_timer();
    config->timer->SR &= ~TIM_SR_TIF; // clear trigger interrupt flag
    config->timer->CR1 |= TIM_CR1_CEN;   // manually start the timer
}

void motor_channel::start_timer_synced() {
    // start the timer, outputs not enabled yet
    // synchronized with other timers, offset according to motor channel number
    // ensure triggering timer is running and configured to trigger this timer, but not yet counting
    // this timer will start counting on the next trigger from the triggering timer
    stop_timer();

    uint32_t timer_cnt_offset = 0;
    uint8_t timer_dir = 0;
    switch(channel_num) {
        case 0:
            timer_cnt_offset = 0;
            break;
        case 1:
            timer_cnt_offset = (config->timer->ARR * 2) / 3;
            break;
        case 2:
            timer_cnt_offset = (config->timer->ARR * 4) / 3;
            timer_cnt_offset -= config->timer->ARR;
            timer_dir = 1;
            break;
        default:
            // invalid channel number, TODO: handle error
            while(1){}
    }

    // timer_cnt_offset = 0;   // for testing sync offsets, remove later

    config->timer->EGR |= TIM_EGR_UG; // generate an update event to update preloaded registers

    config->timer->SR &= ~TIM_SR_TIF; // clear trigger interrupt flag
    if(config->timer->SR & TIM_SR_TIF) {
        // trigger is still set, triggering timer is not configured correctly, TODO: handle error
        while(1){}
    }

    config->timer->CNT = timer_cnt_offset;
    if(timer_dir) {
        config->timer->CR1 |= TIM_CR1_DIR; // set timer direction to down
    }

}

void motor_channel::set_scaled_pwm_values(float phase_u, float phase_v, float phase_w) {
    // won't take effect until the next update event
    // inputs are duty cycle scaled from -1.0 (0%) to 1.0 (100%)

    uint16_t arr = config->timer->ARR;

    float u = phase_u < -1.0f ? -1.0f : (phase_u > 1.0f ? 1.0f : phase_u);
    float v = phase_v < -1.0f ? -1.0f : (phase_v > 1.0f ? 1.0f : phase_v);
    float w = phase_w < -1.0f ? -1.0f : (phase_w > 1.0f ? 1.0f : phase_w);

    uint16_t compare_u = static_cast<uint16_t>((-u * 0.5f + 0.5f) * static_cast<float>(arr));
    uint16_t compare_v = static_cast<uint16_t>((-v * 0.5f + 0.5f) * static_cast<float>(arr));
    uint16_t compare_w = static_cast<uint16_t>((-w * 0.5f + 0.5f) * static_cast<float>(arr));

    *((volatile uint16_t*)(&config->timer->CCR1 + config->phase_u_ch-1)) = compare_u;
    *((volatile uint16_t*)(&config->timer->CCR1 + config->phase_v_ch-1)) = compare_v;
    *((volatile uint16_t*)(&config->timer->CCR1 + config->phase_w_ch-1)) = compare_w;
}

void motor_channel::set_pwm_frequency(float frequency_hz, float* resulting_frequency_hz, uint16_t* resulting_arr) {
    // all timers must be stopped before changing the frequency!
    uint16_t calc_arr = 0;
    if(resulting_frequency_hz == nullptr) {
        calc_arr = timer_arr_from_frequency(frequency_hz, &pwm_frequency_hz);
    } else {
        calc_arr = timer_arr_from_frequency(frequency_hz, &pwm_frequency_hz);
        *resulting_frequency_hz = pwm_frequency_hz;
    }
    config->timer->ARR = calc_arr;
    if(resulting_arr) {
        *resulting_arr = calc_arr;
    }

    ns_per_pwm_cycle = 1e9f / pwm_frequency_hz;

    // update ADC trigger compare value to match new frequency
    // ADC should trigger the same number of ticks before the up->down? timer overflow event.

    phase_adc_trigger_offset = 10;    // TODO: calc based on ADC sample/conversion time and oversampling

    config->timer->CCR4 = calc_arr - phase_adc_trigger_offset; // ensure offset is within the new ARR range
}

void motor_channel::enable_phase_adcs() {
    // enable ADCs for this motor channel

    // Clear stale ADRDY before enabling so waits below are meaningful.
    board_hw::phase_u_adc->ISR = ADC_ISR_ADRDY;
    board_hw::phase_v_adc->ISR = ADC_ISR_ADRDY;
    board_hw::phase_w_adc->ISR = ADC_ISR_ADRDY;

    board_hw::phase_u_adc->CR |= ADC_CR_ADEN;
    board_hw::phase_v_adc->CR |= ADC_CR_ADEN;
    board_hw::phase_w_adc->CR |= ADC_CR_ADEN;
    while(!(board_hw::phase_u_adc->ISR & ADC_ISR_ADRDY)){} // wait for ADC to be ready
    while(!(board_hw::phase_v_adc->ISR & ADC_ISR_ADRDY)){}
    while(!(board_hw::phase_w_adc->ISR & ADC_ISR_ADRDY)){}
}

void motor_channel::disable_phase_adcs() {
    // disable ADCs for this motor channel
    if(board_hw::phase_u_adc->CR & ADC_CR_ADEN) {
        board_hw::phase_u_adc->CR |= ADC_CR_ADDIS;
        while(board_hw::phase_u_adc->CR & ADC_CR_ADEN) {}
    }
    if(board_hw::phase_v_adc->CR & ADC_CR_ADEN) {
        board_hw::phase_v_adc->CR |= ADC_CR_ADDIS;
        while(board_hw::phase_v_adc->CR & ADC_CR_ADEN) {}
    }
    if(board_hw::phase_w_adc->CR & ADC_CR_ADEN) {
        board_hw::phase_w_adc->CR |= ADC_CR_ADDIS;
        while(board_hw::phase_w_adc->CR & ADC_CR_ADEN) {}
    }
    phase_adc_sample_index = 0;
}

void motor_channel::enable_aux_adc() {
    // enable auxiliary ADC

    // Clear stale ADRDY before enabling so wait below is meaningful.
    board_hw::aux_adc->ISR = ADC_ISR_ADRDY;

    board_hw::aux_adc->CR |= ADC_CR_ADEN;
    while(!(board_hw::aux_adc->ISR & ADC_ISR_ADRDY)){} // wait for ADC to be ready
    
}

void motor_channel::disable_aux_adc() {
    // disable auxiliary ADC
    if(board_hw::aux_adc->CR & ADC_CR_ADEN) {
        board_hw::aux_adc->CR |= ADC_CR_ADDIS;
        while(board_hw::aux_adc->CR & ADC_CR_ADEN) {}
    }
}

bool motor_channel::aux_adc_enabled() {
    return (board_hw::aux_adc->CR & ADC_CR_ADEN);
}

void motor_channel::all_phase_adc_eoc_flagged_handler() {

    // read vbus
    trigger_vbus_sense_adc_reading();

    // read ADC results
    phase_u_current = calculate_phase_current(board_hw::phase_u_adc->DR << phase_adc_result_left_shift, phase_u_adc_offset, phase_u_vref_offset);
    phase_v_current = calculate_phase_current(board_hw::phase_v_adc->DR << phase_adc_result_left_shift, phase_v_adc_offset, phase_v_vref_offset);
    phase_w_current = calculate_phase_current(board_hw::phase_w_adc->DR << phase_adc_result_left_shift, phase_w_adc_offset, phase_w_vref_offset);

    phase_adc_sample_index = (phase_adc_sample_index + 1) % num_of_channels;

    // verify sequences are in sync and match adc_sample_index
    uint8_t end_of_sequence_flags = 0;
    end_of_sequence_flags |= (board_hw::phase_u_adc->ISR & ADC_ISR_EOS) ? (1 << 0) : 0;
    end_of_sequence_flags |= (board_hw::phase_v_adc->ISR & ADC_ISR_EOS) ? (1 << 1) : 0;
    end_of_sequence_flags |= (board_hw::phase_w_adc->ISR & ADC_ISR_EOS) ? (1 << 2) : 0;

    if((end_of_sequence_flags != 0) && (end_of_sequence_flags != 0b111)) {
        // sequences are not in sync, TODO: handle error
        while(1){}
    }

    if(end_of_sequence_flags && (phase_adc_sample_index != 0)) {
        // end of sequence flag is set but adc_sample_index is not 0, TODO: handle error
        // while(1){}
        phase_adc_sample_index = 0; // reset to 0 to avoid getting stuck in an error state
    }

    // clear end of sequence flags
    board_hw::phase_u_adc->ISR = ADC_ISR_EOS;
    board_hw::phase_v_adc->ISR = ADC_ISR_EOS;
    board_hw::phase_w_adc->ISR = ADC_ISR_EOS;

    new_phase_adc_data_available = true;
}

void motor_channel::aux_adc_eoc_flagged_handler() {
    // read ADC result
    uint16_t data = board_hw::aux_adc->DR << aux_adc_result_left_shift;

    switch(aux_adc_sample_index) {
        case 0:
            gate_supply_voltage = calculate_gate_supply_voltage(data);
            break;
        case 1:
            IPM_IC_thermistor_temp[0] = calculate_IPM_IC_thermistor_temp(data);
            break;
        case 2:
            IPM_IC_thermistor_temp[1] = calculate_IPM_IC_thermistor_temp(data);
            break;
        case 3:
            IPM_IC_thermistor_temp[2] = calculate_IPM_IC_thermistor_temp(data);
            break;
        case 4:
            IPM_IC_temp[0] = calculate_IPM_IC_temp(data);
            break;
        case 5:
            IPM_IC_temp[1] = calculate_IPM_IC_temp(data);
            break;
        case 6:
            IPM_IC_temp[2] = calculate_IPM_IC_temp(data);
            aux_adc_data_ready = true;
            break;
        default:
            // invalid sample index, TODO: handle error
            while(1){}
    }
    
    aux_adc_sample_index = (aux_adc_sample_index + 1) % 7; // 7 channels in the sequence

    if(aux_adc_sample_index == 0 && !(board_hw::aux_adc->ISR & ADC_ISR_EOS)) {
        // end of sequence flag is not set but aux_adc_sample_index is 0, TODO: handle error
        while(1){}
    }

    // clear end of sequence flag
    board_hw::aux_adc->ISR = ADC_ISR_EOS;

    // trigger next conversion
    manual_aux_adc_trigger();

    new_aux_adc_data_available = true;
}

void motor_channel::phase_adc2_injected_eoc_flagged_handler(){
    // read ADC result
    uint16_t data = ADC2->JDR1 << phase_adc_result_left_shift;

    VBUS_voltage = calculate_VBUS_voltage(data);

}

bool motor_channel::is_timer_running() {
    return (config->timer->CR1 & TIM_CR1_CEN) != 0;
}

bool motor_channel::phase_adcs_enabled() {
    return (board_hw::phase_u_adc->CR & ADC_CR_ADEN) && (board_hw::phase_v_adc->CR & ADC_CR_ADEN) && (board_hw::phase_w_adc->CR & ADC_CR_ADEN);
}

bool motor_channel::global_safety_checks() {
    // check global safety conditions that apply to all motor channels
    bool safety_ok = true;

    // gate undervoltage
    if(gate_supply_voltage < board_hw::ipms.min_gate_supply_voltage) {
        // gate supply undervoltage detected, TODO: handle error
        msg->add(msgs.motor_all.gate_supply_undervoltage, static_cast<uint32_t>(gate_supply_voltage*1000.0f)); // convert to mV for reporting
        safety_ok = false;

    }

    // gate overvoltage
    if(gate_supply_voltage > board_hw::ipms.max_gate_supply_voltage) {
        // gate supply overvoltage detected, TODO: handle error
        msg->add(msgs.motor_all.gate_supply_overvoltage, static_cast<uint32_t>(gate_supply_voltage*1000.0f)); // convert to mV for reporting
        safety_ok = false;
    }

    // VBUS undervoltage
    if(VBUS_voltage < min_vbus_voltage) {
        // VBUS undervoltage detected, TODO: handle error
        msg->add(msgs.motor_all.vbus_undervoltage, static_cast<uint32_t>(VBUS_voltage*1000.0f)); // convert to mV for reporting
        safety_ok = false;
    }

    // VBUS overvoltage
    if(VBUS_voltage > board_hw::ipms.max_vbus_voltage) {
        // VBUS overvoltage detected, TODO: handle error
        msg->add(msgs.motor_all.vbus_overvoltage, static_cast<uint32_t>(VBUS_voltage*1000.0f)); // convert to mV for reporting
        safety_ok = false;
    }

    // Analog watchdog for phase currents
    if(get_analog_watchdog_status()) {
        // analog watchdog triggered, TODO: handle error
        msg->add(msgs.motor_all.analog_phase_current_watchdog_triggered, 0);
        safety_ok = false;
    }

    // STO IPM disable input fault detection
    if(get_sto_ch2_fault_status()) {
        // STO channel 2 fault detected, TODO: handle error
        msg->add(msgs.motor_all.sto_ch2_fault, 0);
        safety_ok = false;
    }

    return safety_ok;
}

bool motor_channel::channel_safety_checks() {

    bool safety_ok = true;
    
    // TODO: make messaging system handle different payload types

    // phase overcurrent
    if(fabs(phase_u_current) > board_hw::ipms.max_phase_current_amps){
        msg->add(motor_msgs.overcurrent_U, static_cast<uint32_t>(phase_u_current*1000.0f)); // convert to mA for reporting
        safety_ok = false; // overcurrent detected, TODO: handle error
    }
    if(fabs(phase_v_current) > board_hw::ipms.max_phase_current_amps){
        msg->add(motor_msgs.overcurrent_V, static_cast<uint32_t>(phase_v_current*1000.0f)); // convert to mA for reporting
        safety_ok = false; // overcurrent detected, TODO: handle error
    }
    if(fabs(phase_w_current) > board_hw::ipms.max_phase_current_amps){
        msg->add(motor_msgs.overcurrent_W, static_cast<uint32_t>(phase_w_current*1000.0f)); // convert to mA for reporting
        safety_ok = false; // overcurrent detected, TODO: handle error
    }

    // phase imbalance
    float imbalance = phase_u_current + phase_v_current + phase_w_current;
    if(fabs(imbalance) > max_phase_imbalance_amps){
        msg->add(motor_msgs.phase_imbalance, static_cast<uint32_t>(imbalance*1000.0f)); // convert to mA for reporting
        safety_ok = false; // phase imbalance detected, TODO: handle error
    }

    // IPM fault flag
    if(ipm_fault_detected) {
        // IPM fault detected, TODO: handle error
        msg->add(motor_msgs.ipm_fault, 0);
        safety_ok = false;
    }

    // also check the IPM pin itself, in case the fault flag was missed
    if(!(gpio_ll::read({config->fault_port, config->fault_pin}))) {
        // IPM fault detected, TODO: handle error
        ipm_fault_detected = true; // set the fault flag in case it was missed
        msg->add(motor_msgs.ipm_fault, 0);
        safety_ok = false;
    }

    // STO break input
    if(get_sto_ch1_fault_status()) {
        // STO channel 1 fault detected, TODO: handle error
        msg->add(msgs.motor_all.sto_ch1_fault, 0);
        safety_ok = false;
    }

    if(!safety_ok) {
        // disable PWM outputs if any safety check failed
        main_output_disable();
    }

    return safety_ok;
}

bool motor_channel::get_sto_ch1_fault_status() {
    // check if STO signal is invalid
    bool sto_fault = false;
    if(gpio_ll::read({config->break_port, config->break_pin}) == 1) {
        sto_fault = true;
    }
    return sto_fault;
}

bool motor_channel::get_sto_ch2_fault_status() {
    // check if STO signal is invalid
    bool sto_fault = false;
    if(gpio_ll::read({board_hw::inverter_disable_port, board_hw::inverter_disable_pin}) == 1) {
        sto_fault = true;
    }
    return sto_fault;
}

void motor_channel::reset_faults(){
    if(gpio_ll::read({config->fault_port, config->fault_pin})){
        // IPM fault pin is high, clear the fault flag
        ipm_fault_detected = false;
    }
}

float motor_channel::calculate_gate_supply_voltage(uint16_t adc_value) {
    // convert ADC value to gate supply voltage in volts
    constexpr float sense_ratio = (board_hw::vref_voltage * ((board_hw::gate_supply_sense_resistor_top + board_hw::gate_supply_sense_resistor_bottom) / board_hw::gate_supply_sense_resistor_bottom)) / 0xFFFF;
    float voltage = static_cast<float>(adc_value) * sense_ratio;
    return voltage;
}

float motor_channel::calculate_VBUS_voltage(uint16_t adc_value) {
    // convert ADC value to VBUS voltage in volts
    constexpr float sense_ratio = (board_hw::vref_voltage * ((board_hw::vbus_sense_resistor_top + board_hw::vbus_sense_resistor_bottom) / board_hw::vbus_sense_resistor_bottom)) / 0xFFFF;
    float voltage = static_cast<float>(adc_value) * sense_ratio;
    return voltage;
}

float motor_channel::calculate_IPM_IC_thermistor_temp(uint16_t adc_value) {
    // convert ADC value to temperature in degrees Celsius using a thermistor lookup table or formula
    // TODO: finish this
    return static_cast<float>(adc_value);
}

float motor_channel::calculate_IPM_IC_temp(uint16_t adc_value) {
    // convert ADC value to temperature in degrees Celsius using a thermistor lookup table or formula
    // TODO: finish this
    return static_cast<float>(adc_value);
}

void motor_channel::zero_phase_adcs() {
    phase_u_adc_offset = static_cast<int32_t>(board_hw::phase_u_adc->DR << phase_adc_result_left_shift) - static_cast<int32_t>(phase_u_vref_offset);
    phase_v_adc_offset = static_cast<int32_t>(board_hw::phase_v_adc->DR << phase_adc_result_left_shift) - static_cast<int32_t>(phase_v_vref_offset);
    phase_w_adc_offset = static_cast<int32_t>(board_hw::phase_w_adc->DR << phase_adc_result_left_shift) - static_cast<int32_t>(phase_w_vref_offset);
}

float motor_channel::calculate_phase_current(uint16_t adc_value, int16_t adc_offset, uint16_t adc_vref_2_offset) {
    // convert ADC value to phase current in amps from sense shunts
    constexpr float adc_lsb_volts = board_hw::vref_voltage / 65535.0f;
    constexpr float volts_to_amps = 1.0f / (board_hw::current_sense_resistance * board_hw::current_sense_gain);
    constexpr float counts_to_amps = adc_lsb_volts * volts_to_amps;

    // Current sense is biased at VREF/2, so subtract midpoint to zero raw ADC counts.
    int32_t centered_counts = static_cast<int32_t>(adc_value) - static_cast<int32_t>(adc_vref_2_offset) - static_cast<int32_t>(adc_offset);

    float current = static_cast<float>(centered_counts) * counts_to_amps;
    return current;
}

float motor_channel::calculate_adc_counts_from_current(float current_amps){
    // convert current in amps to ADC counts for phase current sense
    float adc_lsb_volts = board_hw::vref_voltage / static_cast<float>(65535u >> phase_adc_result_left_shift);
    constexpr float volts_to_amps = 1.0f / (board_hw::current_sense_resistance * board_hw::current_sense_gain);
    float amps_to_counts = 1.0f / (adc_lsb_volts * volts_to_amps);

    int32_t counts = static_cast<int32_t>(current_amps * amps_to_counts);
    return static_cast<float>(counts);
}

uint32_t motor_channel::calculate_pwm_cycles_from_us(uint32_t microseconds) {
    // convert microseconds to PWM cycles based on the current timer configuration
    // result is rounded UP to the nearest whole number of cycles
    float cycles = (static_cast<float>(microseconds) * 1e-6f) * pwm_frequency_hz;
    return static_cast<uint32_t>(ceilf(cycles));
}

float motor_channel::calculate_svpwm_offset(float v_u, float v_v, float v_w) {
    // standard min-max (third-harmonic-equivalent) injection: centers the desired phase
    // voltages within the available range, maximizing VBUS utilization. Produces the same
    // output as conventional sector-based space vector PWM.
    float v_min = v_u;
    if(v_v < v_min) v_min = v_v;
    if(v_w < v_min) v_min = v_w;

    float v_max = v_u;
    if(v_v > v_max) v_max = v_v;
    if(v_w > v_max) v_max = v_w;

    return -(v_max + v_min) * 0.5f;
}

bool motor_channel::select_discontinuous_offset(float v_u, float v_v, float v_w, float v_half, float& offset) {
    // try to hold the phase with the highest current at 0% duty (bottom rail) to reduce
    // switching losses on the phase that would otherwise dissipate the most. Falls back to the
    // next highest current phase if VBUS can't support clamping the first choice.
    float voltages[3] = {v_u, v_v, v_w};
    float current_mag[3] = {fabsf(phase_u_current), fabsf(phase_v_current), fabsf(phase_w_current)};

    uint8_t order[3] = {0, 1, 2};
    for(uint8_t i = 0; i < 2; i++){
        for(uint8_t j = i + 1; j < 3; j++){
            if(current_mag[order[j]] > current_mag[order[i]]){
                uint8_t temp = order[i];
                order[i] = order[j];
                order[j] = temp;
            }
        }
    }

    for(uint8_t i = 0; i < 3; i++){
        uint8_t clamp_idx = order[i];
        float candidate_offset = -v_half - voltages[clamp_idx];

        bool feasible = true;
        for(uint8_t k = 0; k < 3; k++){
            if(k == clamp_idx) continue;
            float v = voltages[k] + candidate_offset;
            if(v < -v_half || v > v_half){
                feasible = false;
                break;
            }
        }

        if(feasible){
            offset = candidate_offset;
            return true;
        }
    }

    return false;
}

int16_t motor_channel::voltage_to_raw(float voltage, float v_half) {
    if(v_half <= 0.0f) return 0; // VBUS not valid yet, default to safe centered output

    float raw_f = (voltage / v_half) * 32767.0f;
    if(raw_f > 32767.0f) raw_f = 32767.0f;
    if(raw_f < -32768.0f) raw_f = -32768.0f;
    return static_cast<int16_t>(raw_f);
}

void motor_channel::apply_basic_deadtime_compensation(int16_t& raw_u, int16_t& raw_v, int16_t& raw_w) {
    // compensate for the fixed hardware deadtime by shifting the commanded duty cycle by the
    // volt-time error it introduces. Sign follows current direction: assumes positive phase
    // current flows out of the inverter leg into the motor winding, which means the low-side
    // body diode conducts during the dead interval, pulling the phase low and reducing the
    // effective output voltage below the commanded value - so duty is increased to compensate.
    // Flip this sign if bench testing shows it's backwards for this hardware.
    constexpr float current_deadband_amps = 0.2f; // avoid chattering right at the current zero-crossing

    float deadtime_fraction = static_cast<float>(board_hw::ipms.deadtime_ns) / static_cast<float>(ns_per_pwm_cycle);
    float raw_correction = deadtime_fraction * 65536.0f;

    int16_t* raws[3] = {&raw_u, &raw_v, &raw_w};
    float currents[3] = {phase_u_current, phase_v_current, phase_w_current};

    for(uint8_t i = 0; i < 3; i++){
        float corrected = static_cast<float>(*raws[i]);
        if(currents[i] > current_deadband_amps){
            corrected += raw_correction;
        } else if(currents[i] < -current_deadband_amps){
            corrected -= raw_correction;
        } else {
            continue;
        }

        if(corrected > 32767.0f) corrected = 32767.0f;
        if(corrected < -32768.0f) corrected = -32768.0f;
        *raws[i] = static_cast<int16_t>(corrected);
    }
}

void motor_channel::enforce_min_pulse_width(int16_t& raw_u, int16_t& raw_v, int16_t& raw_w) {
    // the IPM can't reliably switch a pulse shorter than min_pulse_width_ns. If a phase's raw
    // command is close to, but not exactly at, a rail, snap it to the rail rather than
    // commanding an unreliably-short pulse on the opposite switch. A true 0%/100% command is
    // fine, the hardware can hold either rail indefinitely.
    float min_pulse_raw = (static_cast<float>(board_hw::ipms.min_pulse_width_ns) / static_cast<float>(ns_per_pwm_cycle)) * 65536.0f;

    int16_t* raws[3] = {&raw_u, &raw_v, &raw_w};
    for(uint8_t i = 0; i < 3; i++){
        float r = static_cast<float>(*raws[i]);
        if(r < (-32768.0f + min_pulse_raw)){
            *raws[i] = -32768;
        } else if(r > (32767.0f - min_pulse_raw)){
            *raws[i] = 32767;
        }
    }
}

bool motor_channel::set_phase_voltage(float phase_u_voltage, float phase_v_voltage, float phase_w_voltage) {
    // set the phase voltages in volts, relative to an arbitrary common reference - only the
    // relative (line-to-line) voltages matter, a common offset across all 3 phases is fine and
    // is exactly what the SVPWM/discontinuous PWM offset selection below relies on.

    float vbus = get_VBUS_voltage();
    if(vbus <= 0.0f){
        set_scaled_pwm_values(0.0f, 0.0f, 0.0f); // VBUS not valid yet, hold a safe centered output
        return false;
    }
    float v_half = vbus * 0.5f;

    float v_u = phase_u_voltage;
    float v_v = phase_v_voltage;
    float v_w = phase_w_voltage;

    float offset = 0.0f;
    bool clamped = use_discontinuous_pwm && select_discontinuous_offset(v_u, v_v, v_w, v_half, offset);
    if(!clamped){
        // either discontinuous PWM is disabled, or no single phase could be held at 0% within
        // VBUS limits - use standard SVPWM to make full use of the available VBUS
        offset = calculate_svpwm_offset(v_u, v_v, v_w);
    }

    v_u += offset;
    v_v += offset;
    v_w += offset;

    int16_t raw_u = voltage_to_raw(v_u, v_half);
    int16_t raw_v = voltage_to_raw(v_v, v_half);
    int16_t raw_w = voltage_to_raw(v_w, v_half);

    if(use_basic_deadtime_compensation){
        apply_basic_deadtime_compensation(raw_u, raw_v, raw_w);
    }
    if(use_advanced_deadtime_compensation){
        // TODO: needs a characterization LUT (ipm temp x VBUS x phase current -> switching
        // delay) built from bench test data, which doesn't exist yet.
    }

    enforce_min_pulse_width(raw_u, raw_v, raw_w);

    set_scaled_pwm_values(static_cast<float>(raw_u) / 32768.0f, static_cast<float>(raw_v) / 32768.0f, static_cast<float>(raw_w) / 32768.0f);

    return true;
}


uint8_t motor_channel::adc_resolution_bits = 12;
uint8_t motor_channel::phase_adc_result_left_shift = 0;
uint8_t motor_channel::phase_adc_sample_index = 0;
uint8_t motor_channel::num_of_channels = 0;
uint16_t motor_channel::phase_adc_trigger_offset = 0;

uint32_t motor_channel::ns_per_pwm_cycle = 1e9f / board_hw::phase_min_pwm_frequency_hz; // default to minimum frequency
float motor_channel::pwm_frequency_hz = board_hw::phase_min_pwm_frequency_hz; // default to minimum frequency
uint8_t motor_channel::aux_adc_sample_index = 0;
uint8_t motor_channel::aux_adc_result_left_shift = 0;

float motor_channel::gate_supply_voltage = 0.0f;
float motor_channel::VBUS_voltage = 0.0f;
float motor_channel::IPM_IC_temp[3] = {0.0f, 0.0f, 0.0f};
float motor_channel::IPM_IC_thermistor_temp[3] = {0.0f, 0.0f, 0.0f};
bool motor_channel::new_aux_adc_data_available = false;
Messaging* motor_channel::msg = nullptr;
time_interface* motor_channel::time = nullptr;
float motor_channel::min_vbus_voltage = 0.0f;
bool motor_channel::aux_adc_data_ready = false;

uint16_t motor_channel::phase_u_vref_offset = 0;
uint16_t motor_channel::phase_v_vref_offset = 0;
uint16_t motor_channel::phase_w_vref_offset = 0;