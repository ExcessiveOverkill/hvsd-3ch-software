from aura import AURADevice, Register, Group, Message, MessageGroup, MessageSeverity

device = AURADevice("HVSD-35/400-3", word_width=32, desc="3ch hiogh voltage servo drive", interfaces="shell")

# # Registers
device.regmap.add(Register("status",  rw="r",  type="unsigned", width=8))
device.regmap.add(Register("control", rw="rw", type="unsigned", width=8))

fan_group = Group("fan")
fan_group.add(Register("rpm_fbk", rw="r", width=16, type="unsigned", desc="Fan speed in RPM"))
fan_group.add(Register("pwm_cmd", rw="rw", width=8, type="unsigned", min_val=0, max_val=100, default_val=0, desc="Fan PWM command (0-100%)"))

device.regmap.add(fan_group)

# Optional: messages/logging
device.messages.append(MessageGroup("faults", [
    Message("overcurrent", MessageSeverity.ERROR, desc="Phase current exceeded limit"),
]))

#### Motor Channels ####

# Single channel registers
motor = Group("motor", count=3)
motor.add(Register("ipm_ic_temp",   rw="r", type="float"))
motor.add(Register("ipm_thermistor",  rw="r", type="float"))
device.regmap.add(motor)

# Single channel messages
motor_messages = MessageGroup("motor", count=3)

motor_messages.add(Message("overcurrent_U", MessageSeverity.ERROR, desc="Phase current exceeded IPM limit"))
motor_messages.add(Message("overcurrent_V", MessageSeverity.ERROR, desc="Phase current exceeded IPM limit"))
motor_messages.add(Message("overcurrent_W", MessageSeverity.ERROR, desc="Phase current exceeded IPM limit"))
motor_messages.add(Message("phase_imbalance", MessageSeverity.ERROR, desc="Phase current imbalance detected"))

motor_messages.add(Message("ipm_IC_overtemp", MessageSeverity.ERROR, desc="IPM IC temperature exceeded limit"))
motor_messages.add(Message("ipm_thermistor_overtemp", MessageSeverity.ERROR, desc="IPM thermistor temperature exceeded limit"))
motor_messages.add(Message("ipm_thermistor_fault", MessageSeverity.ERROR, desc="IPM thermistor open/short circuit detected"))
motor_messages.add(Message("ipm_fault", MessageSeverity.ERROR, desc="IPM fault pin triggered"))

motor_messages.add(Message("pwm_break_input_active", MessageSeverity.ERROR, desc="Break input was active during operation"))

mode_messages = MessageGroup("mode", count=1)

pmsm_ident_mode_messages = MessageGroup("pmsm_ident", count=1)
pmsm_ident_mode_messages.add(Message("resistance_too_high", MessageSeverity.ERROR, desc="Unable to reach target current, resistance may be too high, voltage too low, or open circuit"))
pmsm_ident_mode_messages.add(Message("resistance_phase_sense_imbalance", MessageSeverity.ERROR, desc="Abnormal current measured on quiet phase during resistance measurement"))
pmsm_ident_mode_messages.add(Message("resistance_slope_mismatch", MessageSeverity.ERROR, desc="Resistance measurement slopes disagree"))
pmsm_ident_mode_messages.add(Message("current_decay_timeout", MessageSeverity.ERROR, desc="Current did not settle to zero before starting inductance measurement"))
pmsm_ident_mode_messages.add(Message("inductance_no_valid_resistance", MessageSeverity.WARNING, desc="No valid resistance available for inductance measurement"))
pmsm_ident_mode_messages.add(Message("inductance_settle_timeout", MessageSeverity.ERROR, desc="Inductance measurement pass-1 settle timeout"))
pmsm_ident_mode_messages.add(Message("inductance_fit_failure", MessageSeverity.ERROR, desc="Inductance measurement fit failed or low-confidence"))
pmsm_ident_mode_messages.add(Message("resistance_incomplete", MessageSeverity.ERROR, desc="Resistance measurement incomplete or degraded"))
pmsm_ident_mode_messages.add(Message("inductance_incomplete", MessageSeverity.ERROR, desc="Inductance measurement incomplete or degraded"))
pmsm_ident_mode_messages.add(Message("align_current_not_reached", MessageSeverity.ERROR, desc="Unable to reach target current during rotor alignment, motor may be disconnected or open circuit"))
pmsm_ident_mode_messages.add(Message("align_settle_timeout", MessageSeverity.ERROR, desc="Current failed to settle to zero during rotor alignment"))
mode_messages.add(pmsm_ident_mode_messages)

motor_messages.add(mode_messages)

device.messages.append(motor_messages)
# Global registers

# Global messages
all_motor_messages = MessageGroup("motor_all", count=1)

all_motor_messages.add(Message("gate_supply_overvoltage", MessageSeverity.ERROR, desc="Gate supply overvoltage detected"))
all_motor_messages.add(Message("gate_supply_undervoltage", MessageSeverity.ERROR, desc="Gate supply undervoltage detected"))

all_motor_messages.add(Message("vbus_undervoltage", MessageSeverity.ERROR, desc="VBUS undervoltage detected"))
all_motor_messages.add(Message("vbus_overvoltage", MessageSeverity.ERROR, desc="VBUS overvoltage detected"))

all_motor_messages.add(Message("analog_phase_current_watchdog_triggered", MessageSeverity.ERROR, desc="Analog hardware watchdog triggered for phase current"))

all_motor_messages.add(Message("sto_ch1_fault", MessageSeverity.ERROR, desc="STO channel 1 fault detected"))
all_motor_messages.add(Message("sto_ch2_fault", MessageSeverity.ERROR, desc="STO channel 2 fault detected"))

all_motor_messages.add(Message("aux_adc_cycle_not_done", MessageSeverity.CRITICAL, desc="Aux ADC never completed a cycle in timeout period"))

device.messages.append(all_motor_messages)

device.generate("firmware/generated")