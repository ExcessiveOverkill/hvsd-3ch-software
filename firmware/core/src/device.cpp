#include "device.h"

#include "system_stm32g4xx.h"

// ---- Static members -------------------------------------------------------
device::IRQ device::missed_irq = device::IRQ::NONE;
device* device::active_instance = nullptr;
Messaging device::msg;
time_interface device::time;
cordic device::cordic_drv;

motor_channel device::motor_channels[3] = {
    motor_channel(0, &board_hw::motor_configs[0], &msg, &time),
    motor_channel(1, &board_hw::motor_configs[1], &msg, &time),
    motor_channel(2, &board_hw::motor_configs[2], &msg, &time)
};

device::mode_options device::channel_modes[3] = {
        { .none = mode(&device::motor_channels[0], &msg, &time),
          .pmsm_ident = pmsm_ident_mode(&device::motor_channels[0], &msg, &time)
        },
        { .none = mode(&device::motor_channels[1], &msg, &time),
          .pmsm_ident = pmsm_ident_mode(&device::motor_channels[1], &msg, &time)
        },
        { .none = mode(&device::motor_channels[2], &msg, &time),
          .pmsm_ident = pmsm_ident_mode(&device::motor_channels[2], &msg, &time)
        }
};

void device::missed_irq_handler(IRQ irq) {
    missed_irq = irq;
}

// ---- cpu_init -------------------------------------------------------------
// Configure PLL from HSI16 to 170 MHz.
// HSI16 / PLLM(4) * PLLN(85) / PLLR(2) = 170 MHz
void device::cpu_init() {
    // Enable Range 1 boost mode (required for 170 MHz operation)
    PWR->CR5 &= ~PWR_CR5_R1MODE;

    // Set Flash latency to 4 WS before increasing clock speed
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk)
               | FLASH_ACR_LATENCY_4WS
               | FLASH_ACR_ICEN
               | FLASH_ACR_DCEN
               | FLASH_ACR_PRFTEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_4WS) {}

    // TODO: make cpu clock change have a midpoint step according to RM0440 7.2.7

    // HSI16 is already on at reset — configure PLL to use it
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_1          // source = HSI16 (0b10)
                 | (3u  << RCC_PLLCFGR_PLLM_Pos)  // PLLM = /4  (bits = 3, divider = bits+1)
                 | (85u << RCC_PLLCFGR_PLLN_Pos)  // PLLN = x85 → VCO = 340 MHz
                 | (0u  << RCC_PLLCFGR_PLLR_Pos)  // PLLR = /2  (00 = /2)
                 | RCC_PLLCFGR_PLLREN;             // enable PLLR output

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    // Switch SYSCLK to PLL
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) {}

}

// ---- safety_init ----------------------------------------------------------
void device::safety_init() {

    // configure system for proper fault handling

    // Configure option bytes in flash
    // see RM0440 pg. 111-113 for register details
    const uint32_t FLASH_OPTR_DATA = // flash data that will be loaded into FLASH_OPTR after reset
        (1 << 31) | // reserved: default 1
        (1 << 30) | // IRH_EN, Internal reset holder on NRST pin: enabled (1), default 1
        (0b11 << 28) | // NRST_MODE[1:0], PG10 pad mode: Reset Input/Output (0b11), default 0b11
        (1 << 27) | // nBOOT1, nBOOT0 option bit: default 1
        (1 << 26) | // nSWBOOT0, Software BOOT0: BOOT0 taken from PB8/BOOT0 pin (1), default 1
        (1 << 25) | // CCMSRAM_RST, CCM SRAM erase when system reset: CCM SRAM is not erased when a system reset occurs (1), default 1
        (0 << 24) | // SRAM_PE, SRAM1 and CCM SRAM parity check enable: enabled (0), default 1
        (1 << 23) | // nBOOT1, Boot configuration: default 1
        (1 << 22) | // DBANK, Dual bank mode: dual bank (1), default 1
        (1 << 21) | // reserved: default 1
        (0 << 20) | // BFB2, Dual-bank boot: disabled (0), default 0
        (1 << 19) | // WWDG_SW, Window watchdog selection: software (1), default 1
        (1 << 18) | // IWDG_STDBY, Independent watchdog counter freeze in Standby mode: independent watchdog counter is running in Standby mode (1), default 1
        (1 << 17) | // IWDG_STOP, Independent watchdog counter freeze in Stop mode: independent watchdog counter is running in Stop mode (1), default 1
        (1 << 16) | // IWDG_SW, Independent watchdog selection: software (1), default 1
        (1 << 15) | // reserved: default 1
        (1 << 14) | // nRST_SHDW: No reset generated when entering the Shutdown mode (1), default 1
        (1 << 13) | // nRST_STDBY: No reset generated when entering the Standby mode (1), default 1
        (1 << 12) | // nRST_STOP: No reset generated when entering the Stop mode (1), default 1
        (1 << 11) | // reserved: default 1
        (0b100 << 8) | // BOR_LEV, BOR reset level: level 4 2.8v (0b100), default 0b000 level 0 1.7v
        (0xAA << 0);   // RDP, Read protection level: read protection not active (0xAA), default 0xAA (read protection not active)



    // check if option bytes need programmed
    if(FLASH->OPTR != FLASH_OPTR_DATA) {
        // option bytes are not yet programmed with the desired configuration, program them now
        // Unlock option bytes
        while((FLASH->SR & FLASH_SR_BSY));  // wait for flash to be not busy
        FLASH->KEYR = 0x45670123;           // flash unlock sequence part 1
        FLASH->KEYR = 0xCDEF89AB;           // flash unlock sequence part 2
        FLASH->OPTKEYR = 0x08192A3B;        // opt unlock sequence part 1
        FLASH->OPTKEYR = 0x4C5D6E7F;        // opt unlock sequence part 2

        // write option bytes
        FLASH->OPTR = FLASH_OPTR_DATA;
        FLASH->CR |= FLASH_CR_OPTSTRT; // start option byte programming
        while((FLASH->SR & FLASH_SR_BSY));  // wait for flash to be not busy
        FLASH->CR |= FLASH_CR_OBL_LAUNCH; // launch option byte loading to use the new configuration

        // lock flash
        FLASH->CR |= FLASH_CR_LOCK;
    }


    // Flash ECC config
    FLASH->ECCR |= FLASH_ECCR_ECCIE;  // enable flash ECC correction interrupt
    NVIC_SetPriority(FLASH_IRQn, 15u); // priority for flash ECC correction (not errors, which cause hard faults)

    // PVD config (Programmable Voltage Detector for brownout detection)
    PWR->CR2 = PWR_CR2_PVDE | PWR_CR2_PLS_LEV6;          // enable PVD and set threshold to 2.9v (LEV6), see DS12712 table 19. for threshold levels

    SYSCFG->CFGR2 |= SYSCFG_CFGR2_CLL;  // lock hardfaults to trigger timer break inputs
    SYSCFG->CFGR2 |= SYSCFG_CFGR2_SPL;  // lock SRAM1 and CCM SRAM parity errors to trigger timer break inputs
    SYSCFG->CFGR2 |= SYSCFG_CFGR2_PVDL; // lock PVD output to trigger timer break inputs
    SYSCFG->CFGR2 |= SYSCFG_CFGR2_ECCL; // lock flash ECC errors to trigger timer break inputs

}

// ---- systick_init ---------------------------------------------------------
void device::systick_init() {

    // see PM0214 section 4.5

    SysTick->LOAD = board_hw::systick_reload_value;
    SysTick->VAL  = 0u;
    SysTick->CTRL = (board_hw::systick_clk_div == 1u ? SysTick_CTRL_CLKSOURCE_Msk : 0)  // processor clock or div 8
                  | SysTick_CTRL_TICKINT_Msk     // enable interrupt
                  | SysTick_CTRL_ENABLE_Msk;     // enable counter
    NVIC_SetPriority(SysTick_IRQn, 15u);         // lowest priority
}

void device::opamps_init() {
    // TODO: make this more hardware agnostic, currently hardcoded for this specific board

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // enable SYSCFG clock

    // configure VREF opamp
    OPAMP5->CSR |= 0b11 << OPAMP_CSR_VMSEL_Pos; // follower mode
    OPAMP5->CSR |= 0b10 << OPAMP_CSR_VPSEL_Pos; // non-inverting input = PC3 (VINP2)
    OPAMP5->CSR |= OPAMP_CSR_OPAMPxEN; // enable OPAMP5

    // configure VBUS opamp
    OPAMP2->CSR |= 0b11 << OPAMP_CSR_VMSEL_Pos; // follower mode
    OPAMP2->CSR |= 0b10 << OPAMP_CSR_VPSEL_Pos; // non-inverting input = PB0 (VINP2)
    OPAMP2->CSR |= OPAMP_CSR_OPAMPINTEN; // enable OPAMP2 internal connection to ADC2 ch16
    OPAMP2->CSR |= OPAMP_CSR_OPAMPxEN; // enable OPAMP2

}

// ---- init -----------------------------------------------------------------
void device::init() {
    active_instance = this;

    safety_init();
    cpu_init();
    systick_init();
    time.init();
    opamps_init();
    cordic_drv.init();

    fan.init();
    fan.set_speed_percent(0);

    leds.init(&time);

    comm.init();

    msg.init();
    shell_init();

    for(uint8_t i = 0; i < 3; i++) {

        // init all modes
        channel_modes[i].none.init();
        channel_modes[i].pmsm_ident.init();
        // other modes can be added here in the future


        // channel_current_mode_instance[i] = &channel_modes[i].none;  // set to default mode
        
        channel_current_mode_instance[i] = &channel_modes[i].pmsm_ident;  // for testing
        
        motor_channels[i].init();

    }

    // force_restart_pwm_timers(board_hw::phase_min_pwm_frequency_hz); // start PWM timers at minimum frequency

    force_restart_pwm_timers(5000.0f);

    
    channel_current_mode_instance[0]->set_requested_state(mode::states::RUN);

}

void device::start_pwm_timers_at_sync_cnt(uint32_t cnt) {
    // sets sync timer to output TRGO at the specified count, which will trigger the main PWM timers to start

    board_hw::comm_uart_rx_sync_timer->CCR2 = cnt;
    board_hw::comm_uart_rx_sync_timer->CCMR1 &= ~(0b1111 << TIM_CCMR1_OC2M_Pos); // clear output compare 2 mode bits
    board_hw::comm_uart_rx_sync_timer->CCMR1 |= 0b0100 << TIM_CCMR1_OC2M_Pos; // force off
    board_hw::comm_uart_rx_sync_timer->CCMR1 &= ~(0b1111 << TIM_CCMR1_OC2M_Pos); // clear
    board_hw::comm_uart_rx_sync_timer->CCMR1 |= 0b0001 << TIM_CCMR1_OC2M_Pos; // active on match
}

void device::start_pwm_timers_now() {
    // sets sync timer to output TRGO immediately, which will trigger the main PWM timers to start
    board_hw::comm_uart_rx_sync_timer->CCMR1 &= ~(0b1111 << TIM_CCMR1_OC2M_Pos); // clear
    board_hw::comm_uart_rx_sync_timer->CCMR1 |= 0b0101 << TIM_CCMR1_OC2M_Pos; // force on
}

void device::shell_init() {
    // TODO: update generated shell config callback to accept context
    RegShellConfig cfg = make_shell_config(&device::shell_putc, "my_device> ");
    shell.init(cfg);
}

void device::shell_putc(char c) {
    if (active_instance) {
        active_instance->comm.send_byte_blocking(static_cast<uint8_t>(c));
    }
}

// ---- run ------------------------------------------------------------------
void device::run() {
    tim1_update_flag   = false;
    tim1_update_missed = false;

    while(1){
        if(systick_flag){
            flagged_systick();
        }
        if(tim1_update_flag){
            flagged_tim1_update();
        }
        if(tim8_update_flag){
            flagged_tim8_update();
        }
        if(tim20_update_flag){
            flagged_tim20_update();
        }
        if(adc4_eoc_flag) {
            flagged_adc4_eoc();
        }
        if(adc2_injected_eoc_flag) {
            flagged_adc2_injected_eoc();
        }

        
        if(adc1_eoc_flag && adc2_eoc_flag && adc3_eoc_flag) {
            // all ADCs have completed a conversion, read the results and clear the flags
            
            uint8_t phase_adc_sample_index = motor_channel::get_phase_adc_sample_index();
            channel_current_mode_instance[phase_adc_sample_index]->flagged_all_adc_eoc();
            motor_channels[phase_adc_sample_index].all_phase_adc_eoc_flagged_handler();

            phase_adc_sample_index = motor_channel::get_phase_adc_sample_index();
            motor_channels[phase_adc_sample_index].disable_phase_adc_timer_trigger();
            motor_channels[phase_adc_sample_index].enable_phase_adc_timer_trigger();

            adc1_eoc_flag = false;
            adc2_eoc_flag = false;
            adc3_eoc_flag = false;
        }

        // comm testing
        // uint8_t byte = 0;
        // while (comm.receive_byte_nonblocking(byte)) {
        //     // comm.send_byte_blocking(byte);
        //     shell.feed(static_cast<char>(byte));
        // }

        // PWM on led
        bool pwm_on = 0;
        pwm_on |= motor_channels[0].main_output_enabled();
        pwm_on |= motor_channels[1].main_output_enabled();
        pwm_on |= motor_channels[2].main_output_enabled();

        if(pwm_on){
            leds.set_state(led_interface::led_id::BOARD_LED_0, led_interface::led_state::BLINK_FAST);
        }
        else{
            leds.set_state(led_interface::led_id::BOARD_LED_0, led_interface::led_state::OFF);
        }

        time.get_microseconds_update(); // update time interface's microseconds value
    
    }
}

bool device::change_pwm_frequency(float frequency_hz) {
    // change PWM frequency for all channels
    // returns true if successful, false if frequency is out of range or timers or adcs are running
    
    if(motor_channels[0].is_timer_running() || motor_channels[1].is_timer_running() || motor_channels[2].is_timer_running()) {
        return false; // timers are running, cannot change frequency
    }

    if(motor_channels[0].phase_adcs_enabled() || motor_channels[1].phase_adcs_enabled() || motor_channels[2].phase_adcs_enabled()) {
        return false; // adcs are running, cannot change frequency
    }

    if (frequency_hz < board_hw::phase_min_pwm_frequency_hz || frequency_hz > board_hw::phase_max_pwm_frequency_hz) {
        return false; // out of range
    }

    uint16_t resulting_arr = 0;
    motor_channels[0].set_pwm_frequency(frequency_hz, &pwm_freq.hz_float, &resulting_arr);
    motor_channels[1].set_pwm_frequency(frequency_hz);
    motor_channels[2].set_pwm_frequency(frequency_hz);

    pwm_freq.hz_int = static_cast<uint32_t>(frequency_hz);
    pwm_freq.period_s = 1.0f / frequency_hz;
    pwm_freq.period_timer_ticks = (resulting_arr + 1) * 2; // timer counts up and down, so period is 2x the ARR value
    pwm_freq.period_us = static_cast<uint32_t>(pwm_freq.period_s * 1e6f);

    return true;
}

bool device::force_restart_pwm_timers(float frequency_hz) {
    // force restart of PWM timers at the specified frequency
    // stops timers and adcs, changes frequency, and restarts timers and adcs

    // stop all timers
    motor_channels[0].stop_timer();
    motor_channels[1].stop_timer();
    motor_channels[2].stop_timer();

    // disable ADCs
    motor_channel::disable_phase_adcs();
    motor_channel::disable_aux_adc();

    // wait for adcs to finish any conversions
    while(1){
        // TODO: add timeout to this loop in case ADCs get stuck
        for(uint8_t i = 0; i < 3; i++) {
            if(motor_channels[i].phase_adcs_enabled()) {
                continue; // wait for this channel's ADCs to finish
            }
            if(motor_channels[i].aux_adc_enabled()) {
                continue; // wait for this channel's auxiliary ADC to finish
            }
        }
        break; // exit the while loop once all ADCs are finished
    }

    if(!change_pwm_frequency(frequency_hz)) {
        return false;
    }

    // start all timers
    motor_channels[0].start_timer_synced();
    motor_channels[1].start_timer_synced();
    motor_channels[2].start_timer_synced();

    motor_channel::enable_phase_adcs();
    motor_channel::enable_aux_adc();

    motor_channels[0].enable_phase_adc_timer_trigger();   // channel 0 will trigger ADCs first, IRQ handler will switch to the next channel's ADC trigger after all 3 ADCs have completed a conversion
    motor_channel::manual_aux_adc_trigger(); // trigger auxiliary ADC to start sampling

    // TODO: optional to start timers at a specific sync timer count, for now just start immediately
    start_pwm_timers_now();

    channel_current_mode_instance[0]->reset();
    channel_current_mode_instance[1]->reset();
    channel_current_mode_instance[2]->reset();

    return true;
}

// ---- flagged handlers -----------------------------------------------------
void device::flagged_systick() {
    systick_flag = false;
    leds.update();
    fan.systick_flagged_handler();
    fan.set_speed_percent(regs::get_fan_pwm_cmd());
    regs::set_fan_rpm_fbk(fan.get_rpm());
}

void device::flagged_tim1_update() {
    channel_current_mode_instance[0]->flagged_timer_update();
    tim1_update_flag = false;
}

void device::flagged_tim8_update() {
    channel_current_mode_instance[1]->flagged_timer_update();
    tim8_update_flag = false;
}

void device::flagged_tim20_update() {
    channel_current_mode_instance[2]->flagged_timer_update();
    tim20_update_flag = false;
}

void device::flagged_adc1_eoc() {
    adc1_eoc_flag = false;
}

void device::flagged_adc2_eoc() {
    adc2_eoc_flag = false;
}

void device::flagged_adc2_injected_eoc() {
    adc2_injected_eoc_flag = false;
    motor_channel::phase_adc2_injected_eoc_flagged_handler();
}

void device::flagged_adc3_eoc() {
    adc3_eoc_flag = false;
}

void device::flagged_adc4_eoc() {
    adc4_eoc_flag = false;
    motor_channel::aux_adc_eoc_flagged_handler();
}

// interrupt callbacks
void device::SysTick_Handler() {
    channel_current_mode_instance[0]->flagged_systick();
    channel_current_mode_instance[1]->flagged_systick();
    channel_current_mode_instance[2]->flagged_systick();

    systick_flag = true;
}

void device::TIM1_UP_TIM16_IRQHandler() {
    if(TIM1->SR & TIM_SR_UIF){
        TIM1->SR &= ~TIM_SR_UIF;
        tim1_update_missed |= tim1_update_flag;
        tim1_update_flag = true;
    }
}

void device::TIM8_UP_IRQHandler() {
    if(TIM8->SR & TIM_SR_UIF){
        TIM8->SR &= ~TIM_SR_UIF;
        tim8_update_missed |= tim8_update_flag;
        tim8_update_flag = true;
    }
}

void device::TIM20_UP_IRQHandler() {
    if(TIM20->SR & TIM_SR_UIF){
        TIM20->SR &= ~TIM_SR_UIF;
        tim20_update_missed |= tim20_update_flag;
        tim20_update_flag = true;
    }
}

void device::ADC1_2_IRQHandler() {
    if(ADC1->ISR & ADC_ISR_EOC) {
           ADC1->ISR = ADC_ISR_EOC; // clear EOC flag
        adc1_eoc_missed |= adc1_eoc_flag;
        adc1_eoc_flag = true;
    }
    if(ADC2->ISR & ADC_ISR_EOC) {
           ADC2->ISR = ADC_ISR_EOC; // clear EOC flag
        adc2_eoc_missed |= adc2_eoc_flag;
        adc2_eoc_flag = true;
    }
    if(ADC2->ISR & ADC_ISR_JEOC) {
        ADC2->ISR = ADC_ISR_JEOC; // clear injected EOC flag
        adc2_injected_eoc_missed |= adc2_injected_eoc_flag;
        adc2_injected_eoc_flag = true;
    }
}

void device::ADC3_IRQHandler() {
    if(ADC3->ISR & ADC_ISR_EOC) {
        ADC3->ISR = ADC_ISR_EOC; // clear EOC flag
        adc3_eoc_missed |= adc3_eoc_flag;
        adc3_eoc_flag = true;
    }
}

void device::ADC4_IRQHandler() {
    if(ADC4->ISR & ADC_ISR_EOC) {
        ADC4->ISR = ADC_ISR_EOC; // clear EOC flag
        adc4_eoc_missed |= adc4_eoc_flag;
        adc4_eoc_flag = true;
    }
}

void device::EXTI15_10_IRQHandler() {
    // handle IPM fault interrupts for all channels
    // no flagged handlers since this is a critical safety event
    for(uint8_t i = 0; i < 3; i++) {
        motor_channels[i].ipm_exti_irq_handler();
    }
}

