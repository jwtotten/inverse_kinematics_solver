#pragma once
#include <stdint.h>
#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// types.h  —  Shared structs for the hexapod firmware
// ─────────────────────────────────────────────────────────────────────────────


// ── Leg ──────────────────────────────────────────────────────────────────────
// All per-leg state.  Six instances live in a global array in hexapod.ino.
// Total size: 11 floats × 4 B × 6 legs = 264 B of SRAM.
struct Leg {
    // Segment lengths (cm) — same for all legs on this robot
    float femur;
    float tibia;

    // Mounting offsets applied before IK (mirrors IkSolver x/y/z_offset).
    // x_off and y_off are negated for left-side legs (3-5).
    float x_off;
    float y_off;
    float z_off;

    // Body-frame attachment point added to FK output (mirrors
    // IkSolver.x_leg_position / y_leg_position).
    float x_body;
    float y_body;

    // Normalised gait phase [0.0, 1.0) — advanced by GaitConfig.speed each tick.
    float phase;

    // Most recent IK solution (radians).
    // Written by ik_solve(); read by servo_driver_write_all().
    float j1;  // coxa  (yaw  around vertical)
    float j2;  // femur (pitch)
    float j3;  // tibia (pitch, relative to femur)
};


// ── GaitConfig ───────────────────────────────────────────────────────────────
// Single global instance controls all six legs simultaneously.
// 20 B of SRAM.
struct GaitConfig {
    float   step_height;  // cm — vertical clearance during swing
    float   step_length;  // cm — horizontal travel per step; negative = reverse
    float   duty;         // stance fraction [0, 1)
    float   speed;        // phase increment per update tick
                          //   e.g. speed = 1.0/UPDATE_HZ → 1-second cycle
    uint8_t pattern;      // 0=TRIPOD, 1=WAVE, 2=RIPPLE
    bool    running;      // false = hold position, servos still powered
};


// ── GaitPattern constants ────────────────────────────────────────────────────
// Mirrors GaitPattern class in Scripts/gait_controller.py
namespace GaitPattern {
    constexpr uint8_t TRIPOD = 0;
    constexpr uint8_t WAVE   = 1;
    constexpr uint8_t RIPPLE = 2;
}


// ── ServoMap ─────────────────────────────────────────────────────────────────
// Maps one joint to a PCA9685 channel with optional inversion and trim.
// 18 entries stored in PROGMEM (Flash) via servo_driver.cpp — zero SRAM cost.
struct ServoMap {
    uint8_t channel;   // PCA9685 logical channel 0-17
    bool    invert;    // flip direction for physical mounting orientation
    float   trim_deg;  // fine trim in degrees (set to 0.0f, tune after assembly)
};
