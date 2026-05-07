#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <avr/pgmspace.h>
#include <math.h>
#include "servo_driver.h"
#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// servo_driver.cpp  —  PCA9685 servo output implementation
// ─────────────────────────────────────────────────────────────────────────────

// ── PCA9685 board instances ───────────────────────────────────────────────────
static Adafruit_PWMServoDriver pwm0(PCA9685_ADDR_0);
static Adafruit_PWMServoDriver pwm1(PCA9685_ADDR_1);

// ── Servo channel map (PROGMEM — zero SRAM cost) ─────────────────────────────
// Layout: servo_map[leg_index][joint_index]  joint: 0=coxa, 1=femur, 2=tibia
//
// invert=true  for legs 3-5 (left side): the servo horns face the opposite
// direction to their right-side counterparts, so we negate the angle to keep
// the IK coordinate system consistent.
//
// trim_deg is set to 0.0f here; tune per servo after physical assembly using
// the 'trim' serial command (future feature) or by editing these values.
static const ServoMap servo_map[NUM_LEGS][JOINTS_PER_LEG] PROGMEM = {
    // Leg 0 — Right Front
    {{ 0, false, 0.0f }, {  1, false, 0.0f }, {  2, false, 0.0f }},
    // Leg 1 — Right Middle
    {{ 3, false, 0.0f }, {  4, false, 0.0f }, {  5, false, 0.0f }},
    // Leg 2 — Right Rear
    {{ 6, false, 0.0f }, {  7, false, 0.0f }, {  8, false, 0.0f }},
    // Leg 3 — Left Rear   (inverted — mirrors IkSolver left-side negate)
    {{ 9, true,  0.0f }, { 10, true,  0.0f }, { 11, true,  0.0f }},
    // Leg 4 — Left Middle (inverted)
    {{12, true,  0.0f }, { 13, true,  0.0f }, { 14, true,  0.0f }},
    // Leg 5 — Left Front  (inverted; channels 16-17 are on board 1)
    {{15, true,  0.0f }, { 16, true,  0.0f }, { 17, true,  0.0f }},
};

// ── angle_to_ticks ────────────────────────────────────────────────────────────
// Maps a joint angle in radians to a PCA9685 12-bit tick count.
//
// Steps:
//   1. Convert radians → degrees.
//   2. Apply inversion (negate) and trim offset.
//   3. Linear-interpolate degrees to microseconds over [SERVO_MIN_DEG, SERVO_MAX_DEG].
//   4. Convert microseconds to ticks: ticks = us / period_us × 4096
//      where period_us = 20,000 µs (50 Hz).
uint16_t angle_to_ticks(float angle_rad, bool invert, float trim_deg) {
    float deg = angle_rad * (180.0f / (float)M_PI) + trim_deg;
    if (invert) deg = -deg;

    // Clamp to physical servo range
    if (deg < SERVO_MIN_DEG) deg = SERVO_MIN_DEG;
    if (deg > SERVO_MAX_DEG) deg = SERVO_MAX_DEG;

    float us = SERVO_MIN_US
             + (deg - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG)
               * (float)(SERVO_MAX_US - SERVO_MIN_US);

    // 50 Hz → 20,000 µs period; PCA9685 resolution = 4096 ticks/period
    return (uint16_t)((us / 20000.0f) * 4096.0f);
}

// ── Internal: write one tick count to the correct board ──────────────────────
static void write_ticks(uint8_t logical_channel, uint16_t ticks) {
    if (logical_channel < 16) {
        pwm0.setPWM(logical_channel, 0, ticks);
    } else {
        pwm1.setPWM(logical_channel - 16, 0, ticks);
    }
}

// ── servo_driver_init ─────────────────────────────────────────────────────────
void servo_driver_init() {
    Wire.begin();
    Wire.setClock(400000);   // I2C fast mode — PCA9685 supports 400 kHz

    pwm0.begin();
    pwm0.setOscillatorFrequency(PCA9685_OSC_FREQ);
    pwm0.setPWMFreq(50);     // 50 Hz standard RC servo

    pwm1.begin();
    pwm1.setOscillatorFrequency(PCA9685_OSC_FREQ);
    pwm1.setPWMFreq(50);
}

// ── servo_driver_write_all ────────────────────────────────────────────────────
void servo_driver_write_all(const Leg legs[], uint8_t num_legs) {
    for (uint8_t i = 0; i < num_legs; i++) {
        const float joints[JOINTS_PER_LEG] = { legs[i].j1, legs[i].j2, legs[i].j3 };

        for (uint8_t j = 0; j < JOINTS_PER_LEG; j++) {
            // Read ServoMap entry from PROGMEM into a local copy
            ServoMap sm;
            memcpy_P(&sm, &servo_map[i][j], sizeof(ServoMap));

            uint16_t ticks = angle_to_ticks(joints[j], sm.invert, sm.trim_deg);
            write_ticks(sm.channel, ticks);
        }
    }
}

// ── servo_driver_write_channel ────────────────────────────────────────────────
void servo_driver_write_channel(uint8_t channel, float angle_rad) {
    uint16_t ticks = angle_to_ticks(angle_rad, false, 0.0f);
    write_ticks(channel, ticks);
}
