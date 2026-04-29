# Hexapod Arduino Setup Instructions

This guide takes you from unboxing to a walking hexapod.  
Follow the steps in order — wiring mistakes with servos can damage both the servo and the PCA9685 board.

---

## 1. Parts List

| Item | Qty | Notes |
|------|-----|-------|
| Arduino Uno R3 | 1 | Any genuine or clone Uno |
| PCA9685 16-channel PWM servo driver | 2 | Adafruit #815 or equivalent clone |
| Micro servo SG90 or MG90S | 18 | MG90S (metal gear) recommended for durability |
| 5 V DC power supply, ≥ 10 A | 1 | Powers servos only — **never** power servos from the Uno's 5 V pin |
| Female DC barrel jack breakout (5.5 / 2.1 mm) | 1 | To connect PSU to breadboard / screw terminal |
| Jumper wires (male–male, male–female) | ~30 | Various lengths |
| Breadboard or PCB screw terminal block | 1 | For distributing servo power |
| USB-A to USB-B cable | 1 | Programs the Uno and provides serial console |
| 4.7 kΩ resistors | 2 | I2C pull-ups — only needed if your PCA9685 boards do not include them |

**Total servo load estimate:** 18 servos × ~550 mA stall current = ~9.9 A peak.  
A 10 A supply at 5 V is the minimum; 15 A gives comfortable headroom.

---

## 2. PCA9685 I2C Address Configuration

The firmware uses two PCA9685 boards on the same I2C bus.  
They must have **different** I2C addresses.

| Board | Address | Solder bridge to close |
|-------|---------|------------------------|
| Board 0 (channels 0-15) | `0x40` | None — this is the factory default |
| Board 1 (channels 16-17) | `0x41` | Close the **A0** solder bridge on the back of the board |

To close A0 on Board 1: apply a small blob of solder to bridge the two pads labelled `A0` on the underside of the PCB.  Verify with a multimeter (continuity between A0 pad and VCC) before powering on.

---

## 3. Wiring

### 3.1  I2C Bus (Arduino → Both PCA9685 boards)

Both boards share the same four wires:

| Arduino Uno pin | PCA9685 pin |
|-----------------|-------------|
| 5 V | VCC |
| GND | GND |
| A4 (SDA) | SDA |
| A5 (SCL) | SCL |

Daisy-chain the second board from the first: SDA→SDA, SCL→SCL, 5 V→VCC, GND→GND.

> **Note:** The PCA9685's VCC pin powers the I2C logic only (~10 mA).  
> Servo power goes through the V+ / GND screw terminals (see §3.2).

### 3.2  Servo Power (PSU → Both PCA9685 boards)

**Do not use the Arduino 5 V pin to power servos.** The Uno's on-board regulator is rated for 50 mA — one servo can draw ten times that under load.

1. Connect the PSU positive (+) to the **V+** screw terminal on both PCA9685 boards (daisy-chain).  
2. Connect the PSU negative (−) to the **GND** screw terminal on both boards.  
3. Also connect PSU GND to one Arduino GND pin — the Arduino and PSU must share a common ground.

### 3.3  Servo Channel Assignments

Plug each servo's 3-pin connector (signal / power / ground) into the numbered channel on the correct board.  The signal wire (usually orange or yellow) faces outward on the PCA9685 header.

| Leg | Joint | Channel | Board addr |
|-----|-------|---------|------------|
| 0 — Right Front | Coxa (j1) | 0 | 0x40 |
| 0 — Right Front | Femur (j2) | 1 | 0x40 |
| 0 — Right Front | Tibia (j3) | 2 | 0x40 |
| 1 — Right Middle | Coxa | 3 | 0x40 |
| 1 — Right Middle | Femur | 4 | 0x40 |
| 1 — Right Middle | Tibia | 5 | 0x40 |
| 2 — Right Rear | Coxa | 6 | 0x40 |
| 2 — Right Rear | Femur | 7 | 0x40 |
| 2 — Right Rear | Tibia | 8 | 0x40 |
| 3 — Left Rear | Coxa | 9 | 0x40 |
| 3 — Left Rear | Femur | 10 | 0x40 |
| 3 — Left Rear | Tibia | 11 | 0x40 |
| 4 — Left Middle | Coxa | 12 | 0x40 |
| 4 — Left Middle | Femur | 13 | 0x40 |
| 4 — Left Middle | Tibia | 14 | 0x40 |
| 5 — Left Front | Coxa | 15 | 0x40 |
| 5 — Left Front | Femur | 16 | **0x41** |
| 5 — Left Front | Tibia | 17 | **0x41** |

> **Left-side legs (3-5) note:** The firmware automatically inverts the servo  
> direction for left-side legs to compensate for the mirrored mounting  
> orientation. You do **not** need to reverse connectors physically.

---

## 4. Arduino IDE Setup

### 4.1  Install the Arduino IDE

Download from [arduino.cc/en/software](https://www.arduino.cc/en/software) (version 2.x recommended).

### 4.2  Install Required Library

1. Open Arduino IDE → **Tools → Manage Libraries…**
2. Search for **Adafruit PWM Servo Driver**
3. Click **Install** — accept the prompt to also install *Adafruit BusIO* (a dependency)

The built-in `Wire` library (for I2C) requires no separate installation.

### 4.3  Open the Sketch

1. **File → Open…**
2. Navigate to `arduino/hexapod/` in this repository
3. Select `hexapod.ino`

The IDE will automatically include all `.h` and `.cpp` files in the same folder.

### 4.4  Select Board and Port

1. **Tools → Board → Arduino AVR Boards → Arduino Uno**
2. **Tools → Port** → select the COM port (Windows) or `/dev/tty.usbmodem…` (macOS/Linux) that appears when the Uno is plugged in

### 4.5  Upload

Click the **Upload** button (→) or press `Ctrl+U`.  The IDE compiles and flashes the sketch (~15 seconds). You should see `Done uploading.` in the status bar.

---

## 5. First Power-On

Follow this sequence **every time** to avoid servo damage from unexpected movement:

1. Upload the sketch with the PSU **off** (USB only).
2. Open **Serial Monitor** (Tools → Serial Monitor or `Ctrl+Shift+M`).  
   Set baud rate to **115200**.  
   You should see:
   ```
   ─────────────────────────────────────
     Hexapod IK Controller  —  ready
   ─────────────────────────────────────
     S  Start gait      X  Stop
     ...
   ```
3. Turn the PSU on.  All servos will briefly move to the **home position** (all joints at 0°).
4. Visually confirm no servo is straining against a mechanical stop before proceeding.

---

## 6. Serial Command Reference

Send commands by typing a single character in the Serial Monitor and pressing Enter (or set line ending to "No line ending").

| Key | Action |
|-----|--------|
| `S` | **Start** gait |
| `X` | **Stop** gait — servos hold last position |
| `F` | Walk **forward** |
| `B` | Walk **backward** |
| `T` | Switch to **Tripod** gait (default — fastest, most stable) |
| `W` | Switch to **Wave** gait (slowest — maximum stability) |
| `R` | Switch to **Ripple** gait (intermediate) |
| `+` | Speed up (press multiple times) |
| `-` | Slow down |
| `?` | Print current pattern, speed, direction, and running state |

**Recommended first test sequence:** `T` → `S` → `?` → `X`

---

## 7. Servo Calibration

Out-of-the-box the firmware maps the IK output range to ±90° servo travel.  Physical servos vary — you may need to trim individual joints.

### 7.1  Check the PWM limits

In `config.h`, adjust if your servos hit mechanical stops:

```cpp
constexpr uint16_t SERVO_MIN_US  = 500;   // pulse width for SERVO_MIN_DEG
constexpr uint16_t SERVO_MAX_US  = 2500;  // pulse width for SERVO_MAX_DEG
constexpr float    SERVO_MIN_DEG = -90.0f;
constexpr float    SERVO_MAX_DEG =  90.0f;
```

Typical SG90 values: 500 µs = 0°, 2400 µs = 180°.  Adjust and re-upload.

### 7.2  Per-joint trim

In `servo_driver.cpp`, find the `servo_map` table.  Change `trim_deg` for any joint that sits slightly off-centre at rest (all joints at 0° IK output):

```cpp
// Example: trim leg 0 femur by +3 degrees
{{ 1, false, 3.0f }},
```

Re-upload after any change.

### 7.3  Inversion check

If a leg moves in the **wrong direction**, toggle the `invert` flag for that joint in `servo_map`:

```cpp
{{ 9, true,  0.0f }},   // invert=true  → direction reversed
{{ 9, false, 0.0f }},   // invert=false → direction normal
```

---

## 8. Configuration Reference

All tunable parameters are in `config.h`.  Re-upload after any change.

| Constant | Default | Description |
|----------|---------|-------------|
| `FEMUR_LEN` | 3.5 cm | Physical femur segment length |
| `TIBIA_LEN` | 3.5 cm | Physical tibia segment length |
| `STEP_HEIGHT` | 2.0 cm | Foot lift height during swing phase |
| `STEP_LENGTH` | 4.0 cm | Horizontal stride length |
| `DUTY_FACTOR` | 0.5 | Fraction of cycle spent in stance (0–1) |
| `UPDATE_HZ` | 60 | Control loop rate in Hz |
| `SERVO_MIN_US` | 500 µs | Servo pulse at minimum angle |
| `SERVO_MAX_US` | 2500 µs | Servo pulse at maximum angle |

> **Tip:** Measure `FEMUR_LEN` and `TIBIA_LEN` on your 3D-printed parts with calipers  
> and update the values — the IK solution accuracy depends on them.

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Serial Monitor shows nothing | Wrong baud rate | Set to **115200** |
| All servos twitch at power-on but then stop | Normal — home position move | Ignore if no straining sound |
| One or more servos don't move | Wrong channel wiring | Cross-check §3.3 channel table |
| Leg moves in reverse | Invert flag wrong | Toggle `invert` in `servo_map` |
| Servos buzz / strain at rest | PWM range too wide | Reduce `SERVO_MAX_US` / increase `SERVO_MIN_US` |
| `?` shows running=yes but no movement | IK out of range | Increase `FEMUR_LEN`/`TIBIA_LEN` to match your parts |
| Arduino IDE: `Adafruit_PWMServoDriver.h: No such file` | Library not installed | Repeat §4.2 |
| Two PCA9685 boards not found on I2C | A0 bridge not soldered | Re-check §2 |
| Robot walks but legs are out of phase | Phase offset applied before home | Send `X`, then `T`, then `S` |
