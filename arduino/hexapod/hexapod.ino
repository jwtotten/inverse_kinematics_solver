// ─────────────────────────────────────────────────────────────────────────────
// hexapod.ino  —  Arduino Uno R3 hexapod IK controller
//
// C++ port of:
//   Scripts/iksolver.py       → ik.h / ik.cpp
//   Scripts/gait_controller.py → gait.h / gait.cpp
//
// Hardware:
//   Arduino Uno R3
//   2× PCA9685 I2C 16-ch PWM driver  (addr 0x40, 0x41)
//   18× micro servo (SG90 / MG90S)   — 3 per leg (coxa, femur, tibia)
//   External 5 V / 10 A PSU           — servos MUST NOT be powered from Uno
//
// Required libraries (install via Arduino Library Manager):
//   Adafruit PWM Servo Driver Library  (pulls in Adafruit BusIO automatically)
//
// Quick-start:
//   1. Upload this sketch.
//   2. Open Serial Monitor at 115200 baud.
//   3. Send 'T' (TRIPOD gait), then 'S' (START).
//   4. Use '+'/'-' to adjust speed, 'F'/'B' for direction, '?' for status.
// ─────────────────────────────────────────────────────────────────────────────

#include <avr/pgmspace.h>
#include "config.h"
#include "types.h"
#include "ik.h"
#include "gait.h"
#include "servo_driver.h"
#include "serial_cmd.h"

// ── Global state ──────────────────────────────────────────────────────────────
// All allocation is static — no heap use.
static Leg        legs[NUM_LEGS];
static GaitConfig cfg;

// ── setup() ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);

    // ── Initialise each leg ───────────────────────────────────────────────────
    // Mirrors the IkSolver.__init__ instance-number logic in Python:
    //   Legs 0-2 (right side): x_off = X_OFF_BASE,  y_off = Y_OFF_BASE
    //   Legs 3-5 (left  side): x_off = -X_OFF_BASE, y_off = -Y_OFF_BASE
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        legs[i].femur  = FEMUR_LEN;
        legs[i].tibia  = TIBIA_LEN;

        float sign      = (i < 3) ? 1.0f : -1.0f;
        legs[i].x_off   = sign * X_OFF_BASE;
        legs[i].y_off   = sign * Y_OFF_BASE;
        legs[i].z_off   = Z_OFF_BASE;       // z offset is the same for all legs

        legs[i].x_body  = LEG_X_BODY[i];
        legs[i].y_body  = LEG_Y_BODY[i];

        legs[i].phase   = 0.0f;
        legs[i].j1      = 0.0f;
        legs[i].j2      = 0.0f;
        legs[i].j3      = 0.0f;
    }

    // ── Gait defaults ─────────────────────────────────────────────────────────
    cfg.step_height = STEP_HEIGHT;
    cfg.step_length = STEP_LENGTH;   // positive = forward
    cfg.duty        = DUTY_FACTOR;
    cfg.speed       = 1.0f / (float)UPDATE_HZ;  // 1-second gait cycle at 60 Hz
    cfg.pattern     = GaitPattern::TRIPOD;
    cfg.running     = false;

    // Apply initial TRIPOD phase offsets
    gait_set_pattern(legs, NUM_LEGS, &cfg, GaitPattern::TRIPOD);

    // ── Bring up servo driver ─────────────────────────────────────────────────
    servo_driver_init();

    // ── Drive all servos to zero / home position ──────────────────────────────
    // This lets you verify wiring before starting the gait.
    servo_driver_write_all(legs, NUM_LEGS);

    Serial.println(F("─────────────────────────────────────"));
    Serial.println(F("  Hexapod IK Controller  —  ready"));
    Serial.println(F("─────────────────────────────────────"));
    Serial.println(F("  S  Start gait      X  Stop"));
    Serial.println(F("  F  Forward         B  Backward"));
    Serial.println(F("  T  Tripod          W  Wave      R  Ripple"));
    Serial.println(F("  +  Speed up        -  Slow down"));
    Serial.println(F("  ?  Status"));
    Serial.println(F("─────────────────────────────────────"));
}

// ── loop() ───────────────────────────────────────────────────────────────────
// Non-blocking timing gate: check elapsed micros() against the update period.
// micros() rolls over every ~70 minutes; unsigned subtraction handles wrap
// correctly without any special case.
void loop() {
    static uint32_t last_us = 0;
    constexpr uint32_t PERIOD_US = 1000000UL / UPDATE_HZ;

    uint32_t now = micros();
    if (now - last_us >= PERIOD_US) {
        last_us = now;

        // Always poll serial — robot responds to commands even while stopped
        serial_cmd_poll(&cfg, legs, NUM_LEGS);

        if (cfg.running) {
            gait_update(legs, NUM_LEGS, &cfg);
            servo_driver_write_all(legs, NUM_LEGS);
        }
    }
}
