#include <math.h>
#include "config.h"
#include "gait.h"
#include "ik.h"

// ─────────────────────────────────────────────────────────────────────────────
// gait.cpp  —  Gait trajectory implementation
//
// Foot trajectory equations match simulation.html geometry.
// Both functions are closed-form; no arrays are stored.
//
// Fore/aft (y axis — stride direction, mirrors simulation dy):
//   stance (t < duty):  fwd =  step_len × (0.5 − t/duty)
//                               → +step_len/2 (front) … −step_len/2 (rear)
//   swing  (t ≥ duty):  fwd =  step_len × ((t−duty)/(1−duty) − 0.5)
//                               → −step_len/2 … +step_len/2  (continuous!)
//
// Vertical (z axis — replaces lift_leg_equation):
//   stance (t < duty):  z = 0
//   swing  (t ≥ duty):  z = step_height × sin(((t-duty)/(1-duty)) × π)
//
// x axis is constant (= x_body) — no lateral stride component.
// BASE_Z lifts the nominal foot height above ground (mirrors simulation BASE_Z).
//
// Sign of step_length controls direction: positive = forward, negative = reverse.
// ─────────────────────────────────────────────────────────────────────────────

// ── Internal helpers ─────────────────────────────────────────────────────────

// gait_fwd: symmetric fore/aft sweep — continuous at both phase transitions.
// Returns a value in [−step_len/2, +step_len/2].
static float gait_fwd(float t, float step_len, float duty) {
    if (t < duty)
        return step_len * (0.5f - t / duty);
    else
        return step_len * ((t - duty) / (1.0f - duty) - 0.5f);
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

        // ── Compute foot target ───────────────────────────────────────────────
        // x_body: constant lateral extension — xa = x_body + x_off = BASE_X + X_OFF_BASE
        // y_body: per-leg fore/aft rest centre (encodes BASE_Y + yOff stagger)
        //         gait_fwd() sweeps ±STEP_LENGTH/2 around that rest centre.
        // BASE_Z: nominal foot height above ground (added before IK z_off).
        float fx = legs[i].x_body;
        float fy = legs[i].y_body + gait_fwd(t, cfg->step_length, cfg->duty);
        float fz = BASE_Z + gait_z(t, cfg->step_height, cfg->duty);

        // ── Solve IK — holds last valid angles on failure ─────────────────────
        ik_solve(&legs[i], fx, fy, fz);
    }
}
