#include <math.h>
#include "ik.h"

// ─────────────────────────────────────────────────────────────────────────────
// ik.cpp  —  Inverse kinematics implementation
//
// Equations ported from IkSolver.solve_inverse_kinematics() in
// Scripts/iksolver.py:
//
//   j1 = atan2(y, x)
//   H  = sqrt(y² + x²)
//   L  = sqrt(H² + z²)
//   j3 = acos((femur² + tibia² - L²) / (2·femur·tibia))
//   b  = acos((L²   + femur² - tibia²) / (2·L·femur))
//   j2 = b + atan2(z, H)
//
// All lengths in cm, all angles in radians.
// Uses single-precision variants (atan2f, acosf, sqrtf) throughout — on AVR
// float and double are both 32-bit, but explicit 'f' suffix avoids accidental
// double promotion in expressions and clearly signals intent.
// ─────────────────────────────────────────────────────────────────────────────

bool ik_solve(Leg* leg, float x, float y, float z) {
    // ── Apply mounting offsets (mirrors IkSolver.apply_offsets) ──────────────
    float xa = x + leg->x_off;
    float ya = y + leg->y_off;
    float za = z + leg->z_off;

    // ── j1: coxa yaw — rotation in the horizontal plane ──────────────────────
    leg->j1 = atan2f(ya, xa);

    // ── Reach distances ───────────────────────────────────────────────────────
    float H  = sqrtf(ya * ya + xa * xa);   // horizontal distance from coxa
    float L2 = H * H + za * za;            // squared 3-D distance to foot
    float L  = sqrtf(L2);                  // 3-D distance to foot

    // ── j3: tibia angle (law of cosines) ──────────────────────────────────────
    float cos_j3 = (leg->femur * leg->femur + leg->tibia * leg->tibia - L2)
                   / (2.0f * leg->femur * leg->tibia);

    // Reachability guard — acos domain is [-1, 1].
    // Python lets this silently raise a domain error; C++ returns false and
    // leaves joint angles unchanged so servos hold their last valid position.
    if (cos_j3 < -1.0f || cos_j3 > 1.0f) return false;
    leg->j3 = acosf(cos_j3);

    // ── j2: femur angle (law of cosines + elevation angle) ───────────────────
    float cos_b = (L2 + leg->femur * leg->femur - leg->tibia * leg->tibia)
                  / (2.0f * L * leg->femur);

    if (cos_b < -1.0f || cos_b > 1.0f) return false;
    float b  = acosf(cos_b);
    float a  = atan2f(za, H);   // elevation angle to foot
    leg->j2  = b + a;

    return true;
}
