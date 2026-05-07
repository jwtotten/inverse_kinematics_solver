#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// config.h  —  Compile-time constants for the hexapod IK solver
//
// All values mirror the Python defaults in Scripts/iksolver.py and
// Scripts/gait_controller.py so the physical robot uses the same geometry
// as the simulation.
// ─────────────────────────────────────────────────────────────────────────────

// ── Leg geometry (cm) ────────────────────────────────────────────────────────
constexpr float FEMUR_LEN       = 3.5f;
constexpr float TIBIA_LEN       = 3.5f;

// ── Per-leg mounting offsets (cm, body frame) ────────────────────────────────
// Legs 0-2 use these directly; legs 3-5 have x and y negated (left side).
// Matches IkSolver.__init__: x_offset=-5, y_offset=-2, z_offset=-6
constexpr float X_OFF_BASE      = -5.0f;
constexpr float Y_OFF_BASE      = -2.0f;
constexpr float Z_OFF_BASE      = -6.0f;

// ── Body geometry (cm) ───────────────────────────────────────────────────────
// Matches IkSolver: x_length=5, y_length=3, z_length=1
constexpr float BODY_X_LEN      = 5.0f;
constexpr float BODY_Y_LEN      = 3.0f;

// ── Base foot-target offsets (cm, before IK mounting offsets) ────────────────
// Mirror simulation.html BASE_X / BASE_Y / BASE_Z.
//
//   BASE_X — constant lateral extension of each foot from the hip attachment.
//             xa = BASE_X + X_OFF_BASE = 1.5 + (−5) = −3.5  (right side)
//   BASE_Y — fore/aft rest-position centre for all legs.
//             Raised from 1.0 → 1.5 for better IK margin (collision fix).
//   BASE_Z — nominal foot height above the ground plane.
//             za = BASE_Z + Z_OFF_BASE = 1.2 + (−6) = −4.8 at ground,
//             giving L_worst ≈ 6.65 < FEMUR+TIBIA = 7.0 ✓
constexpr float BASE_X          = 1.5f;
constexpr float BASE_Y          = 1.5f;
constexpr float BASE_Z          = 1.2f;

// ── Per-leg fore/aft rest stagger (cm) ───────────────────────────────────────
// Mirrors simulation.html LEG_FORE_AFT = 1.5.
// Front legs sit LEG_FORE_AFT ahead of mid; rear sit LEG_FORE_AFT behind.
//
// Collision-prevention derivation (tripod worst case, STEP_L = 2):
//   Required:  LEG_FORE_AFT > STEP_L/2 + |yLeg_mid| + 0.2 buffer
//              1.5           > 1.0      + 0           + 0.2   ✓
constexpr float LEG_FORE_AFT    = 1.5f;

// ── Leg body-attachment positions (cm from body origin) ──────────────────────
// x_body — constant lateral position fed to IK (no stride in x).
//           Right side: +BASE_X  → xa = BASE_X + X_OFF_BASE = −3.5
//           Left  side: −BASE_X  → xa = −BASE_X − X_OFF_BASE = +3.5
//
// y_body — fore/aft rest centre fed to IK; gait_fwd() is added each tick.
//           Derived as  BASE_Y + yOff  for right legs (and negated for left):
//             Front (yOff = +LEG_FORE_AFT): y_body =  BASE_Y + 1.5 =  3.0
//             Mid   (yOff =  0)           : y_body =  BASE_Y + 0   =  1.5
//             Rear  (yOff = −LEG_FORE_AFT): y_body =  BASE_Y − 1.5 =  0.0
//           Left side values are the negatives of the right side.
constexpr float LEG_X_BODY[6]  = {  1.5f,  1.5f,  1.5f, -1.5f, -1.5f, -1.5f };
constexpr float LEG_Y_BODY[6]  = {  3.0f,  1.5f,  0.0f, -3.0f, -1.5f,  0.0f };

// ── Gait defaults ────────────────────────────────────────────────────────────
// STEP_HEIGHT — swing arc height (cm); matches simulation STEP_H = 2.0.
// STEP_LENGTH — total fore/aft stride range (cm); mirrors simulation STEP_L = 2.0.
//               gait_fwd() sweeps ±STEP_LENGTH/2 = ±1.0 cm per cycle.
//               Worst-case IK reach (R-Rear at −1.0): L ≈ 6.65 < 7.0 ✓
constexpr float STEP_HEIGHT     = 2.0f;
constexpr float STEP_LENGTH     = 2.0f;
constexpr float DUTY_FACTOR     = 0.5f;

// ── Control loop ─────────────────────────────────────────────────────────────
constexpr uint16_t UPDATE_HZ    = 60;     // target update rate

// ── Hardware ─────────────────────────────────────────────────────────────────
constexpr uint8_t  NUM_LEGS     = 6;
constexpr uint8_t  JOINTS_PER_LEG = 3;   // coxa (j1), femur (j2), tibia (j3)

// PCA9685 I2C addresses
constexpr uint8_t  PCA9685_ADDR_0 = 0x40;  // channels  0-15
constexpr uint8_t  PCA9685_ADDR_1 = 0x41;  // channels 16-17

// PCA9685 oscillator — factory default; trim if servos drift
constexpr uint32_t PCA9685_OSC_FREQ = 25000000UL;

// ── Servo PWM range ──────────────────────────────────────────────────────────
// Pulse width in microseconds for a standard RC servo.
// Adjust SERVO_MIN_US / SERVO_MAX_US per your servo model.
constexpr uint16_t SERVO_MIN_US  = 500;
constexpr uint16_t SERVO_MAX_US  = 2500;

// Joint angle range the above pulse range maps to (degrees).
// IK output is in radians; angle_to_ticks() converts to degrees first.
constexpr float    SERVO_MIN_DEG = -90.0f;
constexpr float    SERVO_MAX_DEG =  90.0f;

// ── Serial baud rate ─────────────────────────────────────────────────────────
constexpr uint32_t SERIAL_BAUD   = 115200UL;
