# POD-042 — STM32 Firmware

Firmware for the peripheral control unit of POD-042: a battery-powered
wireless digital level built around an **STM32F103**, an **MPU6050** and an
**ESP32**. The STM32 samples the sensor, runs the orientation filter, drives
the status LED and speaks a small line-based UART protocol with the ESP32,
which handles Bluetooth LE.

The device reports **roll and pitch only** (no yaw, no magnetometer). The
design target is **±1–2°** after calibration; that figure has not yet been
verified against an angular reference. See
[Validation status](#validation-status) for what has and has not been tested.

> **Prototype.** This is not a finished product. Features added most recently
> are built but still pending hardware validation, and the OLED panel of the
> current prototype does not work. Both points are tracked below.

---

## Features

- MPU6050 sampled at **20 Hz** with a complementary filter and gyro bias
  calibration.
- Roll wrapped to **(−180°, 180°]**; pitch limited to ±90°.
- Absolute level detection with hysteresis (**2°/3°**) and debounce
  (**200/400 ms**).
- **Tare**: relative zero stored in the last flash page and clearable over BLE
  without an angle jump.
- **Auto-hold** after 1 s still and **IMU auto-off** after 3 min without
  motion.
- Real **I2C bus recovery** (SCL pulses + STOP + reinit) after repeated bus
  errors.
- **Sleep/wake** with STOP mode and an EXTI wake button.
- **Battery measurement** with VREFINT compensation.
- Optional **SSD1306 OLED**, excluded from the default build.
- Release build (`-Os`) without compiler warnings.

---

## Hardware

| Component | Role |
|---|---|
| STM32F103C8T6 | Peripheral controller (64 KB flash, 20 KB RAM) |
| MPU6050 | Accelerometer + gyroscope over I2C2 |
| ESP32 | BLE bridge and central unit (see the `esp32` branch) |
| LiPo battery | Power source, measured through a switched divider |
| OLED SSD1306 128×32 | Optional HUD; **not working on this prototype** |

### Pin configuration

| Pin | Function |
|---|---|
| PA9 | UART1 TX → ESP32 |
| PA10 | UART1 RX ← ESP32 |
| PA3 | Wake button, EXTI3 |
| PC13 | Status LED, active low |
| PB10 | I2C2 SCL — MPU6050 (and OLED when enabled) |
| PB11 | I2C2 SDA — MPU6050 (and OLED when enabled) |
| PA8 | Battery divider enable, active high |
| PB1 | ADC channel 9 — battery divider |
| PA13/PA14 | SWD debug/programming |

The UART runs at **115200 8N1**. I2C2 runs at **400 kHz**.

---

## Repository layout

```
Core/                       CubeMX startup, clock/HAL config, main loop
Drivers/Drivers_POD/        Application: FSM, IMU filter, MPU6050 driver, UART
Drivers/OLED/               SSD1306 driver (compiled, excluded from the binary by default)
docs/                       Design notes: OLED build flag, I2C recovery
Release/                    Build output (git-ignored)
STM32F103C8TX_FLASH.ld      Linker script: 63 KB flash, last 1 KB reserved for TARE
POD_STM32.ioc               CubeMX project
```

---

## Firmware architecture

The main loop is intentionally small:

```c
while (1) {
    FINITE_STATE_MACHINE_MAIN();   /* commands, sleep, LED */
#if POD_USE_OLED
    POD_UI_TASK();                 /* display, disabled by default */
#endif
    IMU_Processloop();             /* sample, filter, telemetry, auto-off */
}
```

### Main FSM

| State | Behaviour |
|---|---|
| `POD_STATE_INIT` | Waits ~3 s, configures the MPU6050, reads the battery, goes to IDLE |
| `POD_STATE_IDLE` | Dispatches UART commands, updates the level LED, watches the sleep timeout |
| `POD_STATE_ERROR` | Unknown command; returns to IDLE on the next valid command |

### IMU processing

Every 50 ms, `IMU_Processloop()` performs:

1. **Read** 14 bytes (accel, temperature, gyro) over I2C2 with a 100 ms HAL
   timeout.
2. **Scale** accelerometer to ±4 g (8192 LSB/g) and gyro to ±250 dps
   (131 LSB/dps), with DLPF at 21 Hz (accel) / 20 Hz (gyro).
3. **Reject** samples whose acceleration magnitude leaves the 0.6–1.4 g
   window; they do not touch the filter.
4. **Remap** the board axes to the enclosure axes:
   `Xv = −Y_s`, `Yv = +Z_s`, `Zv = −X_s` (determinant +1).
5. **Calibrate** the gyro bias over 50 stationary samples (2.5 s). Motion or
   an acceleration far from 1 g restarts the calibration.
6. **Fuse** accelerometer and gyro with a complementary filter
   (`alpha = 0.98` moving, `0.95` still). Prediction and innovation are
   wrapped to the short angular path, so crossing ±180° does not add a
   spurious ~360° correction.
7. **Detect motion** from gyro (>3 dps) and acceleration (>0.05 g) with a
   300 ms debounce.
8. **Hold** freezes the reported angles. It can be manual (`STM:HOLD`) or
   automatic after 1 s without motion.
9. **Level** sets `L=1` when both angles stay inside 2° for 200 ms, and clears
   it when either leaves 3° for 400 ms.
10. **Publish** telemetry at 10 Hz (even though sampling runs at 20 Hz).

### Tare and persistence

- The absolute angle is `fused + tare`.
- `STM:TARE` adds the current fused angle to the offset, zeroes the filter
  output and writes the result to flash.
- The record lives at `0x0800FC00` (last 1 KB page; the linker script reserves
  it), contains magic `"TARE"`, version, roll, pitch and a software CRC32, and
  is written **only when tare changes**. A 1 KB page tolerates roughly 10 000
  erase cycles.
- `STM:TARECLR` converts the relative zero back to an absolute angle
  (`fused += tare`), erases the record and returns to the sensor's absolute
  angle. The reading does not jump.
- The stored zero is reloaded when the IMU is activated.

### I2C bus recovery

If the MPU does not answer for **5 consecutive samples** (~250 ms), the
firmware does not just rewrite sensor registers: it releases the bus with up to
9 SCL pulses, generates a manual STOP, reinitialises I2C2 and then probes the
sensor before reconfiguring it. A 1 s cooldown prevents hammering a dead bus.
The result is reported as `IMU:I2C:RECOVER` or `IMU:I2C:FAIL`. Rationale and
test procedure: [`docs/I2C_RECOVERY.md`](docs/I2C_RECOVERY.md).

### Sleep, auto-off and LED

- With the IMU **inactive**, 60 s without commands or activity enters STOP
  mode. The PA3 button wakes the MCU via EXTI; the clock is restored, the
  UART resumed and the MPU reconfigured.
- With the IMU **active**, 180 s without motion turns it off automatically and
  sends `IMU:OFF`.
- While the IMU is active, the LED mirrors the level state (`L=1` → LED on).
  With the IMU inactive the LED can be toggled with `STM:LED`.

---

## UART protocol

Line-based ASCII, terminated by `\n`. The receive buffer is 64 bytes.

### Commands received from the ESP32

| Command | Action |
|---|---|
| `STM:PING` | Heartbeat; replies `ESP:PING:RECEIVE` |
| `STM:BATT` | Reads and reports battery voltage |
| `STM:LED` | Toggles the LED when the IMU is inactive |
| `STM:MPU` | Toggles the IMU; replies `IMU:ON` / `IMU:OFF` |
| `STM:TARE` | Sets the relative zero (only while the IMU is active) |
| `STM:TARECLR` | Clears the stored zero; replies `ESP:TARE:CLEAR` / `ESP:TARE:ERR` |
| `STM:HOLD` | Toggles manual hold (only while the IMU is active) |
| `STM:LIVE` | Toggles live mode, which bypasses auto-hold; replies `ESP:LIVE:ON/OFF` |

### Messages sent to the ESP32

| Message | Meaning |
|---|---|
| `ESP:PING:RECEIVE` | Heartbeat acknowledge |
| `ESP:LED:ON` / `ESP:LED:OFF` | LED state |
| `ESP:ADC:RAW:vref=…,vbatt=…,vdd=…` | Raw ADC values, for calibration |
| `ESP:ADC:BATT` | Battery measurement start marker |
| `ESP:ADC:BATT:<mV>` | Battery voltage in millivolts |
| `ESP:ADC:BATT:ERR` | ADC or VREFINT failure |
| `IMU:ON` / `IMU:OFF` | IMU activation state |
| `IMU:L<0\|1>,R<deg×10>,P<deg×10>,H<0\|1>,M<0\|1>` | Level, roll, pitch, hold, motion |
| `IMU:I2C:RECOVER` / `IMU:I2C:FAIL` | Bus recovery result |
| `ESP:TARE:CLEAR` / `ESP:TARE:ERR` | Tare-clear result |
| `ESP:LIVE:ON` / `ESP:LIVE:OFF` | Live-mode state |
| `STM:SLEEP` / `STM:WAKE` | Entering / leaving STOP mode |

Telemetry example: `IMU:L1,R-103,P25,H0,M0` means level, roll −10.3°,
pitch +2.5°, hold off, no motion.

> Legacy gesture (`IMU:SHAKE`) and orientation-zone helpers remain in the
> source for reference, but the level firmware does not call them.

---

## Battery measurement

The battery is switched on for the measurement, then turned off:

```
VDD   = 1170 mV × 4095 / ADC_VREFINT     (VREFINT factory calibration)
Vpin  = ADC_VBATT × VDD / 4095           (16 samples averaged)
Vbatt = Vpin × (R1 + R2)/R2 + Vce        (BATT_GAIN_MILLI, BATT_DROP_MV)
```

With `R1 = R2`, `BATT_GAIN_MILLI = 2000`; `BATT_DROP_MV = 200` models the
switching PNP. Both values are defined in `pod_fsm.c` and must be adjusted if
the resistors change. The result is approximate: the PNP drop is estimated,
the divider has ±10 % tolerance, and the measurement needs a common ground.

---

## OLED interface (optional)

The SSD1306 panel shares **I2C2 with the MPU6050** and `ssd1306.c` waits with
`HAL_MAX_DELAY`. A dead or stuck panel can therefore block the sensor bus. It
is also why the default build excludes the display entirely.

`POD_USE_OLED` in `Drivers/Drivers_POD/pod_shared.h` defaults to **0**:

| Value | Behaviour |
|---|---|
| `0` | Not initialised, not refreshed. Code still compiled; linker drops it. |
| `1` | Initialised, level screen drawn, panel turned off/on with sleep. |

The display code remains in the repository and compiles in both cases. Full
rationale and the measurement procedure are in
[`docs/OLED_BUILD.md`](docs/OLED_BUILD.md).

---

## Build and flash

### STM32CubeIDE

1. Open `POD_STM32.ioc` / the project in STM32CubeIDE.
2. Select the **Release** configuration.
3. Build with `Ctrl+B` and flash with `Run`.

### Command line

```bash
# Toolchain bundled with STM32CubeIDE 2.1.1, or any arm-none-eabi-gcc in PATH
PATH=/opt/st/stm32cubeide_2.1.1/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin:$PATH \
  make -C Release -j4 all

arm-none-eabi-size Release/POD_STM32.elf
```

To build with the display, change `POD_USE_OLED` to `1` in `pod_shared.h` and
rebuild. The macro is guarded with `#ifndef`, so it can also be supplied as a
preprocessor define in the project configuration.

### Current size (Release, `-Os`, no OLED)

| text | data | bss | Flash (text+data) | RAM (data+bss) |
|---:|---:|---:|---:|---:|
| 25 156 | 100 | 2 528 | **25 256 B** (38.5 % of 64 KB) | **2 628 B** (12.8 % of 20 KB) |

Over the 63 KB available after reserving the TARE page, flash usage is 39.1 %.
Enabling the OLED added about **11.7 KB of flash and 0.5 KB of RAM** in the
last measurement; that figure predates the roll and I2C changes and has not
been re-measured.

---

## Validation status

The firmware builds and has been run on the target during development. The
following table separates what was exercised on hardware from what is only
built.

| Area | Status |
|---|---|
| UART protocol and command dispatch | Exercised on hardware in earlier sessions |
| IMU telemetry, level + LED, hold, sleep/wake | Exercised on hardware in earlier sessions |
| TARE persistence and `TARECLR` | Validated on hardware |
| Battery measurement | Measured; approximate by design (see limitations) |
| Roll wrap around ±180° | **Built; pending hardware validation** |
| I2C bus recovery | **Built; pending hardware validation** |
| OLED-disabled build | **Built; pending cadence measurement** |
| Absolute accuracy vs. an angular reference | **Not demonstrated** |
| Debounce timing at 1 Hz and 5 Hz | **Not demonstrated** |
| OLED panel | Not working on this prototype; excluded from the build |

There is no automated test suite. Validation is done by flashing the board and
capturing UART telemetry; host-side simulations used during development are
not part of the repository.

---

## Known limitations

- **Roll and pitch only.** No yaw or magnetometer; heading is not available.
- **Accuracy target unverified.** Repeatability has been observed, but the
  ±1–2° figure has not been checked against a pattern.
- **OLED is not functional on this prototype.** The code is kept and can be
  re-enabled with one macro.
- **Telemetry cadence.** Earlier captures ran well below the 100 ms telemetry
  period (around 5 Hz). The cause is not confirmed; the OLED is now excluded
  as a candidate fix, but the effect has not been measured yet.
- **Battery reading is approximate** (PNP drop and resistor tolerance).
- **Single I2C bus.** The MPU and the optional OLED share I2C2.
- **Flash wear.** The TARE page is written only when the zero changes; the
  endurance is about 10 000 cycles per page.

---

## AI assistance

Parts of the firmware, documentation and host-side test harnesses were
prepared with the help of an AI coding agent (OpenCode). The design decisions,
hardware assembly, flashing and physical validation were performed by the
author. AI-assisted changes were reviewed and compiled; their hardware status
is stated explicitly in [Validation status](#validation-status).

---

## License

Released under the [MIT License](LICENSE).

---

## Documentation

- [`docs/OLED_BUILD.md`](docs/OLED_BUILD.md) — why the display is out of the
  default build, its measured cost and how to re-enable it.
- [`docs/I2C_RECOVERY.md`](docs/I2C_RECOVERY.md) — bus-clear sequence, error
  propagation and the hardware test procedure.
- ESP32 firmware: `esp32` branch of this repository.
