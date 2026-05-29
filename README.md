# LiftMotor_Flyer_v8

**Platform:** STM32G431CBTx (ARM Cortex-M4, 170 MHz)  
**Machine:** Flyer Frame (FF) — Left Lift & Right Lift axes  
**Firmware:** v8  
**Part No:** CUST-0015 (Motor 300W-GS-LIBD)

---

## What This Firmware Does

Controls a BLDC motor that drives a **lead-screw lift mechanism** on the Flyer Frame machine. Two identical boards run the same firmware — one for **Left Lift** (CAN ID 4) and one for **Right Lift** (CAN ID 5).

The lift moves the bobbin carriage up and down in a traverse stroke pattern during winding. The firmware supports open-loop (duty-cycle) and closed-loop (position) control using two encoders — a 14-bit SPI motor encoder and a PWM gearbox encoder.

---

## Hardware

| Component | Device | Purpose |
|-----------|--------|---------|
| MCU | STM32G431CBTx | Runs all firmware |
| Motor Driver | BLDC Drive 48V / ~12A | Drives motor phases U, V, W |
| Motor Encoder | AS5x47P 14-bit SPI (16384 cnt/rev) | Shaft angle for commutation + calibration |
| Gearbox Encoder | PWM absolute encoder TIM3 (45265 cnt/rev) | Lead-screw absolute position (80.6:1 ratio) |
| Speed Encoder | TIM2 quadrature (2048 cnt/rev) | RPM feedback for PID |
| Position Resolution | 140 TIM3 counts per mm | Stroke position accuracy |
| EEPROM | I2C (hi2c1) | Stores PID, offsets, CAN ID, homing, GB bin errors |
| CAN | FDCAN 1 Mbit/s | Communication with Flyer MainBoard |
| Console | UART3 115200 baud | Service menu via USB-UART programmer |
| Position Loop | TIM16 40 ms periodic | Closed-loop position control execution |
| LOB Watchdog | TIM17 | Lift out-of-bounds safety watchdog |

---

## How a Stroke Works

```
MainBoard sends LIFT_SETUP (distance, time, direction) over CAN
  └─> Firmware sets up trapezoid velocity profile (PosCntrl)
  └─> MainBoard sends LIFT_NEW_STROKE command
  └─> Motor ramps up following velocity profile
  └─> Position tracked via both GB encoder and motor encoder
  └─> PID corrects error between target and actual GB position
  └─> Motor ramps down → stops at target distance
  └─> Stroke complete → IDLE_STATE

MainBoard sends RAMPDOWN_STOP at any time
  └─> Motor ramps down and stops
  └─> Returns to IDLE_STATE
```

### Closed-Loop Position Control

- **Velocity profile:** Trapezoid — ramp up → cruise → ramp down
- **Primary feedback:** Gearbox (GB) PWM encoder — absolute position in mm
- **Secondary feedback:** Motor encoder — cross-checks GB position
- **PID error:** `targetStrokePos − GBActualStroke`
- **Position loop rate:** Every 40 ms (TIM16 interrupt)
- **Velocity range:** 1.5 – 4.0 mm/s (set by stroke time input)

---

## Motor States

| State | Description |
|-------|-------------|
| `IDLE_STATE` | Motor stopped. Accepts CAN commands and console trigger |
| `RUN_STATE` | Stroke in progress — open-loop or closed-loop |
| `ERROR_STATE` | Fault active. Motor stopped. Console accessible for diagnosis |
| `CONFIG_STATE` | Console active (blocking). CAN commands ignored |
| `CALIBRATION_STATE` | Motor encoder calibration — 30-step blocking routine |
| `HOMING_STATE` | Gearbox homing routine in progress |
| `PAUSE_STATE` | Stroke paused mid-run. Resumable via CAN RESUME |

---

## EEPROM Parameters

| Parameter | Address | Default | Description |
|-----------|---------|---------|-------------|
| `Kp` | 0x02 | 0 | PID proportional gain |
| `Ki` | 0x06 | 0 | PID integral gain |
| `start_offset` | 0x0A | 100 | PWM offset at motor startup |
| `ff_percent` | 0x0C | 10 | Feed-forward % of target velocity |
| `MOTID` | 0x10 | — | CAN node ID (4 = Left Lift, 5 = Right Lift) |
| `AMS_offset_index` | 0x12 | — | Encoder zero from calibration routine |
| `default_direction` | 0x14 | 0 (CW) | CW = DOWN, CCW = UP for lift motors |
| `LIFT_HOMING_POSITION` | 0x16 | — | GB PWM count at physical home (≈ 6–7% duty) |
| `GEARBOX_ERROR_VALUE` | 0x20 | 0.0 | 21-bin correction table (float × 21 = 84 bytes) |

---

## CAN Interface

| Function ID | Direction | Description |
|-------------|-----------|-------------|
| `0x01` | Motor → MB | Motor state byte |
| `0x02` | Motor → MB | Error bitmask (all fault bits) |
| `0x09` | Motor → MB | Runtime data (position, RPM, current, voltage) |
| `0x10` | MB → Motor | Lift stroke setup: distance, time, direction |
| `0x15` | MB → Motor | New stroke start command |
| `0x16` | MB → Motor | Request gearbox position data |
| `0x17` | Motor → MB | Gearbox position data response |
| `0x06` | MB → Motor | PID tuning update (Kp, Ki, FF, StartOffset) |

**CAN Node IDs:**

| Motor | CAN ID |
|-------|--------|
| Left Lift | `0x04` |
| Right Lift | `0x05` |

---

## New Motor Commissioning (Console)

Connect programmer to UART3, open terminal at **115200 baud**, trigger console with `$$$$` (Shift+4 four times). Motor must be in **IDLE_STATE**.

| Step | Option | Action | Note |
|------|--------|--------|------|
| 1 | Option 2 | Set CAN ID (4 = Left, 5 = Right) | Save + Restart (Option 14) |
| 2 | Option 4 | Set direction — CW (0) = DOWN | Save + Restart |
| 3 | Option 6 | Run encoder calibration — **gearbox must NOT be assembled** | Record `encIndex_wOffset` |
| 4 | Option 3 | Save `encIndex_wOffset` to EEPROM | Restart + verify via Option 1 |
| 5 | — | Assemble gearbox. Refit lead screw — **no nut yet** | |
| 6 | Option 7 | Gearbox homing — jog until GB duty = 6–7%, save | Restart |
| 7 | Option 12 | Open-loop run DOWN 300 mm to GB count ≈ 45000 | Prep for calibration |
| 8 | Option 8 | Gearbox calibration — run full stroke, check all bins ±15, save | Refit nut after |
| 9 | Option 11 | Closed-loop test — 50 mm UP in 20 s | Verify tracking within ±5% |
| 10 | Option 13 | Sync encoder abs position to GB position (press 2) | |

---

## Gearbox Calibration — How It Works

The gearbox PWM encoder has position-dependent errors. The firmware maps these across **21 bins** (each = 15 mm of travel) and stores a correction table in EEPROM.

- Motor runs UP from GB count ≈ 45000 → 0 (full stroke)
- Each bin records: `Average_Error = GB_reading − Motor_encoder_reading`
- Correction applied in real time during closed-loop strokes
- All 21 bins must be within **±15.0** — values outside this = mechanical problem

---

## Error Codes

| Bit | Fault | Recovery |
|-----|-------|---------|
| 1 | Over-current (> 12A) | Check load and wiring |
| 2 | Over-voltage (> 50V) | Check PSU |
| 3 | Under-voltage (< 40V) | Check PSU under load |
| 4 | Motor thermistor fault | Check NTC wiring (10kΩ @ 25°C) |
| 5 | FET thermistor fault | Check FET NTC wiring |
| 6 | Motor over-temperature | Allow cool-down, check airflow |
| 7 | FET over-temperature | Check heatsink + thermal paste |
| 8 | EEPROM write error | Check I2C bus |
| 9 | EEPROM bad value at boot | Re-enter all settings, restart |
| 10 | Tracking error | Re-tune PID (Option 5) |
| 11 | Motor encoder setup fail | Check SPI wiring + magnet gap |
| 12 | Lift position tracking error | Re-run gearbox calibration (Option 8) |
| 13 | Lift synchronicity fail | Resync via Option 13 |
| 14 | Lift out of bounds (LOB) | Use Option 10 to jog to safe position |
| 15 | EEPROM bad homing position | Re-run homing (Option 7) |

---

## Console Menu Summary

| Option | Name | When to Use |
|--------|------|------------|
| 1 | View Settings | First check in any session — reads all EEPROM + live sensor values |
| 2 | Change CAN ID | New motor or CAN conflict |
| 3 | Change Encoder Offset | After encoder calibration (Option 6) |
| 4 | Change Direction | Wrong rotation direction |
| 5 | PID Settings | Tuning — Kp, Ki, FF%, StartOffset |
| 6 | Encoder Calibration | Mandatory for new motor — shaft must be free |
| 7 | Gearbox Homing | Set physical home reference position |
| 8 | Gearbox Calibration | Map 21-bin error correction table |
| 9 | GB Bin Management | View or zero bin error table |
| 10 | Toggle LOB Limit | Engineering only — disable out-of-bounds check |
| 11 | Run Closed-Loop | Position-controlled stroke test |
| 12 | Run Open-Loop | Manual jog — duty-cycle controlled |
| 13 | Zero EncAbsPosition | Sync motor encoder to gearbox position |
| 14 | Restart Motor Code | Always run after saving any EEPROM value |

---

## Build

- IDE: STM32CubeIDE  
- Import project → Build (`Ctrl+B`)  
- Flash with ST-LINK V2 via SWD or STM32CubeProgrammer (`.elf` / `.hex`)  
- Do **not** regenerate the `.ioc` file without engineering review
