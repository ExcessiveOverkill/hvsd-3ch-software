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

device.messages.append(motor_messages)
# Global registers

# Global messages
all_motor_messages = MessageGroup("motor_all", count=1)

all_motor_messages.add(Message("gate_supply_overvoltage", MessageSeverity.ERROR, desc="Gate supply overvoltage detected"))
all_motor_messages.add(Message("gate_supply_undervoltage", MessageSeverity.ERROR, desc="Gate supply undervoltage detected"))

all_motor_messages.add(Message("vbus_undervoltage", MessageSeverity.ERROR, desc="VBUS undervoltage detected"))
all_motor_messages.add(Message("vbus_overvoltage", MessageSeverity.ERROR, desc="VBUS overvoltage detected"))

all_motor_messages.add(Message("analog_phase_current_watchdog_triggered", MessageSeverity.ERROR, desc="Analog hardware watchdog triggered for phase current"))

device.messages.append(all_motor_messages)

device.generate("firmware/generated")