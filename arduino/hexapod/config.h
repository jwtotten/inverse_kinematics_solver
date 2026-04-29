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

// ── Leg body-attachment positions (cm from body origin) ──────────────────────
// Derived from IkSolver.__init__ instance-number formula.
// Right side (legs 0-2): x=0,  y = 0 / -0.75 / -1.5
// Left  side (legs 3-5): x=-2.5, y = 0 / -0.75 / -1.5
constexpr float LEG_X_BODY[6]   = { 0.0f,  0.0f,  0.0f, -2.5f, -2.5f, -2.5f };
constexpr float LEG_Y_BODY[6]   = { 0.0f, -0.75f, -1.5f, 0.0f, -0.75f, -1.5f };

// ── Gait defaults ────────────────────────────────────────────────────────────
// Matches GaitController defaults: step_height=2.0, step_length=4.0, duty=0.5
constexpr float STEP_HEIGHT     = 2.0f;
constexpr float STEP_LENGTH     = 4.0f;
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
