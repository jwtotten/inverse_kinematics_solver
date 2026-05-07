#pragma once
#include "types.h"

// ─────────────────────────────────────────────────────────────────────────────
// ik.h  —  3-DOF inverse kinematics solver
//
// Direct C++ port of IkSolver.solve_inverse_kinematics() in
// Scripts/iksolver.py.  Operates on a single Leg struct; no dynamic
// allocation, no global state.
// ─────────────────────────────────────────────────────────────────────────────

// Solve IK for one leg given a foot target in leg-local coordinates (cm).
//
// Applies leg->x_off / y_off / z_off (mounting offsets) before computing
// angles, exactly as IkSolver.apply_offsets() does in Python.
//
// Writes results into leg->j1, leg->j2, leg->j3 (radians).
//
// Returns true  if the target is geometrically reachable.
// Returns false if the target is out of reach (cos value leaves [-1,1]).
//              The leg's previous joint angles are preserved on failure so
//              the servos hold their last valid position.
bool ik_solve(Leg* leg, float x, float y, float z);
