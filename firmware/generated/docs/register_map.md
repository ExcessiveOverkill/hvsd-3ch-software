# HVSD-35/400-3 Register Map

## Global Settings

| Setting | Value |
|---------|-------|
| Word Width | 32 bit |
| Min Access Words | 1 |
| Compatible Drivers | - |

---

## Registers

| Address | Name | Type | R/W | Bits | Default | Min | Max | Unit | Description |
|---------|------|------|-----|------|---------|-----|-----|------|-------------|
| 0x0000 | `status` | unsigned | r | [7:0] | - | - | - | - | - |
| 0x0001 | `control` | unsigned | rw | [7:0] | - | - | - | - | - |

#### `fan` — 0x0002–0x0003

**Alignment:** 2

| Address | Name | Type | R/W | Bits | Default | Min | Max | Unit | Description |
|---------|------|------|-----|------|---------|-----|-----|------|-------------|
| 0x0002 | `rpm_fbk` | unsigned | r | [15:0] | - | - | - | - | Fan speed in RPM |
| 0x0003 | `pwm_cmd` | unsigned | rw | [7:0] | 0 | 0 | 100 | - | Fan PWM command (0-100%) |

#### `motor` — 0x0004–0x0009

**Count:** 3  **Alignment:** 2

| Address | Name | Type | R/W | Bits | Default | Min | Max | Unit | Description |
|---------|------|------|-----|------|---------|-----|-----|------|-------------|
| 0x0004 | `ipm_ic_temp` | float | r | [31:0] | - | - | - | - | - |
| 0x0005 | `ipm_thermistor` | float | r | [31:0] | - | - | - | - | - |

