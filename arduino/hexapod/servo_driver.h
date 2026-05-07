#pragma once
#include <stdint.h>
#include "types.h"

// ─────────────────────────────────────────────────────────────────────────────
// servo_driver.h  —  PCA9685 servo output abstraction
//
// Drives 18 servos across two PCA9685 I2C PWM driver boards:
//   Board 0 (addr 0x40): logical channels  0-15  (legs 0-4 + leg 5 coxa)
//   Board 1 (addr 0x41): logical channels 16-17  (leg 5 femur + tibia)
//
// Channel assignment (3 joints × 6 legs = 18):
//   Leg | Coxa (j1) | Femur (j2) | Tibia (j3)
//    0  |     0     |      1     |      2
//    1  |     3     |      4     |      5
//    2  |     6     |      7     |      8
//    3  |     9     |     10     |     11
//    4  |    12     |     13     |     14
//    5  |    15     |     16*    |     17*    (* on board 1)
//
// Left-side legs (3-5) have invert=true in servo_map so the servo direction
// matches the mirrored coordinate system used by the IK solver.
// ─────────────────────────────────────────────────────────────────────────────

// Initialise both PCA9685 boards over I2C at 50 Hz PWM (standard RC servo).
// Call once from setup().
void servo_driver_init();

// Write all 18 joint angles to the PCA9685 boards.
// Reads leg->j1, j2, j3 (radians) for each leg; applies inversion and trim
// from the PROGMEM servo_map table; writes 12-bit tick counts to the boards.
void servo_driver_write_all(const Leg legs[], uint8_t num_legs);

// Convert a joint angle (radians) to a PCA9685 12-bit tick count.
// Applies invert flag and trim_deg offset.  Exposed for calibration sketches.
uint16_t angle_to_ticks(float angle_rad, bool invert, float trim_deg);

// Drive a single channel to a known angle — useful for wiring verification.
// channel: logical channel 0-17.
void servo_driver_write_channel(uint8_t channel, float angle_rad);
