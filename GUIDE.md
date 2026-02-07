# ESP32 Air Drums — Complete Build & Calibration Guide

This guide walks through **everything** needed to assemble, wire, calibrate, test, and run the ESP32 air drum system using the provided `airdrums_esp32.ino` sketch and shared-zone audio files.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Bill of Materials](#bill-of-materials)
3. [Power & Safety](#power--safety)
4. [Wiring Guide (Step-by-Step)](#wiring-guide-step-by-step)
5. [MPU6050 Placement on the Drumsticks](#mpu6050-placement-on-the-drumsticks)
6. [Audio Chain (DAC → RC Filter → PAM8403 → Speakers)](#audio-chain-dac--rc-filter--pam8403--speakers)
7. [Preparing WAV Files](#preparing-wav-files)
8. [Uploading WAVs to LittleFS](#uploading-wavs-to-littlefs)
9. [Flashing the Firmware](#flashing-the-firmware)
10. [IMU Test & Calibration](#imu-test--calibration)
11. [Tuning & Troubleshooting](#tuning--troubleshooting)
12. [Maintenance Tips](#maintenance-tips)

---

## System Overview

- **MCU:** ESP32-WROOM (DAC pins available on GPIO25/26)
- **Sensors:** 2× MPU6050 (one per stick, I²C at 400 kHz)
- **Audio:** ESP32 DAC → RC filter → PAM8403 stereo amplifier → speakers
- **Zones:** Shared 6-zone set (Center, Left, Right, Top Left, Top Right, Front)
- **Audio Files:** Stored on LittleFS in `/zone_*.wav`

---

## Bill of Materials

**Required**
- ESP32-WROOM DevKit (30- or 38-pin)
- 2× MPU6050 breakout (GY-521)
- PAM8403 stereo amplifier module
- 2× 8Ω speakers
- RC filter parts: 1kΩ resistors (2x), 10nF caps (2x)
- 0.1 µF decoupling caps (at least 2)
- 100 µF bulk cap on 5V rail

**Power**
- Li-ion cell(s) + TP4056 + boost converter **or** 5V USB power supply

---

## Power & Safety

- **All grounds must be common** (ESP32, MPU6050s, PAM8403, battery).
- If using battery power, ensure the boost converter outputs **exactly 5.0V**.
- Add a 100 µF cap across the 5V rail and 0.1 µF close to each module.

---

## Wiring Guide (Step-by-Step)

### ESP32 ↔ MPU6050 (shared I²C)

| ESP32 Pin | MPU6050 Pin | Notes |
|----------|-------------|------|
| 3.3V     | VCC         | Power both sensors from 3.3V |
| GND      | GND         | Common ground |
| GPIO21   | SDA         | Shared I²C SDA |
| GPIO22   | SCL         | Shared I²C SCL |

Set **AD0** on one MPU6050 to **GND** (0x68) and the other to **3.3V** (0x69).

### ESP32 DAC → Audio

| ESP32 Pin | RC Filter | PAM8403 |
|----------|-----------|---------|
| GPIO25   | 1kΩ + 10nF | INL/INR (both tied for mono) |

---

## MPU6050 Placement on the Drumsticks

Correct placement dramatically affects **gesture recognition** and **hit detection**:

**Recommended position:**
- Mount the MPU6050 **near the tip**, but **not at the very end**.
- The best balance is **~3–6 cm below the tip**.

**Why not near your wrist?**
- The closer to your wrist, the more wrist rotation dominates the signal.
- Hit detection becomes inconsistent and zones feel less physical.

**Why not at the extreme tip?**
- Excessive vibration and impact noise cause false triggers.
- The sensor may saturate on strong hits.

**Orientation**
- Keep both IMUs oriented the same way (Z-axis aligned along stick length).
- If one is rotated, zone detection will feel reversed.

Secure with foam tape or epoxy to avoid vibration shift.

---

## Audio Chain (DAC → RC Filter → PAM8403 → Speakers)

**RC filter per channel (mono):**
```
GPIO25 → 1kΩ → junction → PAM8403 INL/INR
                      |
                     10nF
                      |
                     GND
```

**Speaker output:**
- Use **OUTL+ / OUTL-** to drive 2×8Ω speakers in parallel (4Ω load).

---

## Preparing WAV Files

**Required format**:
- 8-bit unsigned PCM
- Mono
- 16 kHz sample rate

**Required filenames (shared zones only):**
```
/zone_center.wav
/zone_left.wav
/zone_right.wav
/zone_top_left.wav
/zone_top_right.wav
/zone_front.wav
```

**Conversion example (FFmpeg):**
```
ffmpeg -i input.wav -ac 1 -ar 16000 -f u8 zone_center.wav
```

---

## Uploading WAVs to LittleFS

1. Create a `data/` folder next to your `.ino` sketch.
2. Place all `/zone_*.wav` files inside `data/`.

```
airdrums_esp32/
  airdrums_esp32.ino
  data/
    zone_center.wav
    zone_left.wav
    zone_right.wav
    zone_top_left.wav
    zone_top_right.wav
    zone_front.wav
```

3. In Arduino IDE:
   - Install **ESP32 Sketch Data Upload** tool
   - Select board + port
   - In **Tools → Partition Scheme**, pick a layout that includes **LittleFS**
   - Use **Tools → ESP32 Sketch Data Upload** (uploads to LittleFS)

---

## Flashing the Firmware

1. Open `airdrums_esp32.ino` in Arduino IDE.
2. Select **Board: ESP32 Dev Module**.
3. **ESP32 core compatibility:** `XT_DAC_Audio` needs a small patch for ESP32 Arduino core **3.x** (timer/DAC register API changes). If you prefer to avoid patching, use **ESP32 core 2.0.x** (e.g. 2.0.17).
4. **If you are on core 3.x**, apply the included patch to your local library:
   - Locate your library folder, e.g.
     `C:\\Users\\<you>\\Documents\\Arduino\\libraries\\XT_DAC_Audio\\XT_DAC_Audio.cpp`
   - Apply the patch file: `patches/xt_dac_audio_core3.patch`
   - Reopen Arduino IDE (so it reloads the library)
5. Choose the correct COM port.
6. Click **Upload**.

**What is `patches/xt_dac_audio_core3.patch`?**  
It is a small patch that updates the **XT_DAC_Audio** library to work with ESP32 Arduino core **3.x**. It replaces the old timer/DAC register calls with the new core 3.x APIs (e.g., `timerBegin(...)`, `timerAlarm(...)`, and `dac_output_voltage()`), fixing the compile errors you saw on ESP32 core 3.3.6.

---

## IMU Test & Calibration

### 1) Basic I²C test
Open Serial Monitor (115200 baud). On boot, you should see:
```
MPU6050 #1 OK
MPU6050 #2 OK
```

If not:
- Check SDA/SCL wiring and pullups
- Verify addresses (0x68 / 0x69)

### 2) Calibration flow
On boot, the firmware runs a calibration stage:
- Hold both sticks **vertical and still** for 3 seconds
- Then hold your **neutral playing position**

After calibration:
```
✓ Calibration saved to NVS
```

### 3) Serial commands
Use Serial Monitor:
- `c` → re-calibrate IMU offsets
- `z` → re-set neutral zone baseline
- `t` → test mode

---

## Testing IMU Components

### Quick hardware sanity check
1. Upload `imu_serial_test.ino`
2. Open Serial Monitor (115200)
3. Wiggle each stick and confirm accel/gyro values change

### What to look for
- **Accel X/Y/Z** should show ~9.81 m/s² combined when still
- **Gyro** should be near 0 when not moving
- If one axis is inverted, your IMU orientation differs

---

## Tuning & Troubleshooting

### Missed hits
- Reduce `HIT_THRESHOLD`
- Verify IMU is closer to tip

### Double triggers
- Increase `HIT_THRESHOLD`
- Ensure IMU is firmly mounted (no loose vibration)

### Zones flickering
- Increase `ZONE_LOCK_DURATION`
- Increase `SOFT_SWITCH_THRESHOLD`

### No sound
- Check LittleFS upload
- Confirm file names match exactly
- Check RC filter wiring and PAM8403 power

---

## Maintenance Tips

- Recalibrate if stick placement changes
- Re-upload WAVs if files are replaced
- Avoid loose solder joints on DAC pin and PAM8403 input

---

## Quick Checklist

- [ ] Both IMUs detected (0x68, 0x69)
- [ ] WAV files uploaded to LittleFS
- [ ] DAC filter and amplifier wired correctly
- [ ] Calibration saved to NVS
- [ ] Zones respond predictably

---

If you want, I can also provide a printable wiring diagram or a conversion script for batch WAV processing.
