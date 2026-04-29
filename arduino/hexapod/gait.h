#pragma once
#include "types.h"

// ─────────────────────────────────────────────────────────────────────────────
// gait.h  —  Gait trajectory computation
//
// C++ port of GaitController in Scripts/gait_controller.py.
//
// Key difference from Python: NO trajectory arrays are stored.
// The Python GaitController pre-computes 50-sample arrays which would cost
// 50 × 3 axes × 6 legs × 4 B = 3,600 B — more than the Uno's entire SRAM.
// Instead, foot targets are computed analytically from a single float phase
// per leg on every update tick.
// ─────────────────────────────────────────────────────────────────────────────

// Set per-leg phase offsets for the chosen gait pattern and store the pattern
// in cfg->pattern.  Call whenever the pattern changes (serial command 'T'/'W'/'R').
//
// Mirrors GaitController.set_gait_pattern():
//   TRIPOD : alternating 0 / 0.5 (legs 0,2,4 vs 1,3,5)
//   WAVE   : 1/6 increments  (0, 0.167, 0.333, 0.5, 0.667, 0.833)
//   RIPPLE : 1/4 increments  (0, 0.25,  0.5,   0.75, 0.0,  0.25)
void gait_set_pattern(Leg legs[], uint8_t num_legs,
                      GaitConfig* cfg, uint8_t pattern);

// Advance each leg's phase by cfg->speed, compute the foot target using the
// analytical gait equations, then call ik_solve().  Call once per update tick.
//
// Mirrors the combined effect of GaitController._generate_gait_cycle() +
// GaitController.solve_leg_position_from_target_coordinates() in Python.
void gait_update(Leg legs[], uint8_t num_legs, const GaitConfig* cfg);
