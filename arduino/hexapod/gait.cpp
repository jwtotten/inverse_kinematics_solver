#include <math.h>
#include "gait.h"
#include "ik.h"

// ─────────────────────────────────────────────────────────────────────────────
// gait.cpp  —  Gait trajectory implementation
//
// Foot trajectory equations ported from GaitController in
// Scripts/gait_controller.py.  Both functions are closed-form; no arrays.
//
// Horizontal (replaces carry_leg_equation):
//   stance (t < duty):  x = -step_len × (t / duty)         ← foot sweeps back
//   swing  (t ≥ duty):  x =  step_len × ((t-duty)/(1-duty)) ← foot sweeps forward
//
// Vertical (replaces lift_leg_equation):
//   stance (t < duty):  z = 0
//   swing  (t ≥ duty):  z = step_height × sin(((t-duty)/(1-duty)) × π)
//
// Sign of step_length controls direction: positive = forward, negative = reverse.
// ─────────────────────────────────────────────────────────────────────────────

// ── Internal helpers ─────────────────────────────────────────────────────────

static float gait_x(float t, float step_len, float duty) {
    if (t < duty)
        return -step_len * (t / duty);
    else
        return  step_len * ((t - duty) / (1.0f - duty));
}

static float gait_z(float t, float step_height, float duty) {
    if (t < duty)
        return 0.0f;
    else
        return step_height * sinf(((t - duty) / (1.0f - duty)) * (float)M_PI);
}

// ── Public API ───────────────────────────────────────────────────────────────

void gait_set_pattern(Leg legs[], uint8_t num_legs,
                      GaitConfig* cfg, uint8_t pattern) {
    cfg->pattern = pattern;
    for (uint8_t i = 0; i < num_legs; i++) {
        switch (pattern) {
            case GaitPattern::TRIPOD:
                // Legs 0,2,4 lead; legs 1,3,5 trail by half a cycle.
                // Mirrors: phase_offset = 0.5 if instance_number % 2 else 0
                legs[i].phase = (i % 2 == 0) ? 0.0f : 0.5f;
                break;

            case GaitPattern::WAVE:
                // Each leg is 1/6 of a cycle behind the previous one.
                // Mirrors: phase_offset = (instance_number × 0.167) % 1
                legs[i].phase = fmodf((float)i * (1.0f / 6.0f), 1.0f);
                break;

            case GaitPattern::RIPPLE:
                // Each leg is 1/4 of a cycle behind the previous one.
                // Mirrors: phase_offset = (instance_number × 0.25) % 1
                legs[i].phase = fmodf((float)i * 0.25f, 1.0f);
                break;

            default:
                legs[i].phase = 0.0f;
                break;
        }
    }
}

void gait_update(Leg legs[], uint8_t num_legs, const GaitConfig* cfg) {
    for (uint8_t i = 0; i < num_legs; i++) {
        // ── Advance phase ─────────────────────────────────────────────────────
        legs[i].phase += cfg->speed;
        if (legs[i].phase >= 1.0f)
            legs[i].phase -= 1.0f;   // wrap — no fmodf needed for small steps

        float t = legs[i].phase;

        // ── Compute foot target in leg-local space ────────────────────────────
        // x_body / y_body shift the target to the leg's attachment point on the
        // body, mirroring the x_leg_position / y_leg_position offsets that
        // IkSolver.solve_leg_position_from_target_coordinates() adds in Python.
        float fx = gait_x(t, cfg->step_length, cfg->duty) + legs[i].x_body;
        float fy = legs[i].y_body;   // lateral position fixed (no lateral gait)
        float fz = gait_z(t, cfg->step_height, cfg->duty);

        // ── Solve IK — holds last valid angles on failure ─────────────────────
        ik_solve(&legs[i], fx, fy, fz);
    }
}
