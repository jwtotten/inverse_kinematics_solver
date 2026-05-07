#pragma once
#include "types.h"

// ─────────────────────────────────────────────────────────────────────────────
// serial_cmd.h  —  Single-byte serial command protocol
//
// All commands are single printable ASCII characters so the robot can be
// controlled from any serial terminal (Arduino Serial Monitor, PuTTY, screen)
// without a custom host application.
//
// Command table:
//   'S'  Start gait
//   'X'  Stop gait  (servos stay powered at last position)
//   'F'  Walk forward   (step_length = +STEP_LENGTH)
//   'B'  Walk backward  (step_length = -STEP_LENGTH)
//   'T'  Gait pattern: TRIPOD
//   'W'  Gait pattern: WAVE
//   'R'  Gait pattern: RIPPLE
//   '+'  Increase speed (speed += 0.002 per press)
//   '-'  Decrease speed (speed -= 0.002 per press)
//   '?'  Query — print pattern, speed, and running state over Serial
// ─────────────────────────────────────────────────────────────────────────────

// Call once per loop() iteration.
// Drains Serial RX buffer and acts on any recognised command bytes.
// Does nothing if Serial has no data — safe to call every tick.
void serial_cmd_poll(GaitConfig* cfg, Leg legs[], uint8_t num_legs);
