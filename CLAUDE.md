# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

**Install dependencies**
```bash
pip install -r requirements.txt
```

**Run all tests**
```bash
python unittesting.py
```

**Run a single test class**
```bash
python -m unittest Unittests.gait_controller_tests.GaitControllerTests
python -m unittest Unittests.ik_solver_tests.IkSolverTests
```

**Run a single test method**
```bash
python -m unittest Unittests.gait_controller_tests.GaitControllerTests.test_01_initialisation
```

**Run simulations**
```bash
python animated_simulation.py     # single-leg 2D animation
python multi_leg_simulation.py    # all 6 legs, 3D animation with gait patterns
```

**Browser simulation:** open `simulation.html` directly in a browser — no server required.

**Arduino:** flash `arduino/hexapod/hexapod.ino` via the Arduino IDE with the Adafruit PWM Servo Driver library installed. Serial commands at 115200 baud: `S` (start), `X` (stop), `F`/`B` (forward/backward), `T`/`W`/`R` (tripod/wave/ripple), `+`/`-` (speed), `?` (status).

## Architecture

This project has three parallel implementations of the same hexapod IK and gait algorithms: a **Python simulation** (desktop visualization + tests), an **Arduino firmware** (real hardware), and a **browser simulation** (Three.js). All share the same geometry and gait math.

### Python — class hierarchy and data flow

```
IkSolver  ──wraps──►  GaitController
    │                       │
    └──── both implement ───┘
          get_motion() → [x_targets, y_targets, z_targets]
          solve_leg_position_from_target_coordinates(x, y, z) → [[joint0], [joint1], [joint2]]
          
Plotter accepts either an IkSolver or a GaitController (or a list of either)
via plot_animated_3d_projection(ik_solver)
```

### IkSolver (`Scripts/iksolver.py`)

- Enforces a hard limit of **6 instances** via `__new__`/`__del__` and a class-level `_instances` list — one per hexapod leg.
- Instance creation order determines leg side and direction: instances 1–3 are right-side legs (forward motion), instances 4–6 are left-side (x/y targets and offsets are negated).
- Odd-numbered instances get `leg_direction = 'forward'`, even-numbered get `'backwards'` — this creates the alternating gait needed for straight-line walking.
- `solve_inverse_kinematics(x, y, z)` applies coordinate offsets first, then uses the geometric 3-DOF solution: j1 = atan2(y,x) for base rotation, law-of-cosines for j2/j3.
- `solve_forward_kinematics(q1, q2, q3)` reconstructs joint positions (coxa→femur→tibia) and is used internally to get plottable coordinates from IK angles.
- `solve_leg_position_from_target_coordinates` chains IK→FK and applies `x_leg_position`/`y_leg_position` offsets to place each leg relative to the robot body.

### GaitController (`Scripts/gait_controller.py`)

- Wraps an existing `IkSolver` instance; does not create new solvers.
- Delegates `solve_leg_position_from_target_coordinates` directly to its wrapped `IkSolver`.
- Generates smooth cyclic trajectories: `lift_leg_equation()` produces a sine-wave swing phase + zero stance phase; `carry_leg_equation()` produces linear forward/backward ramps.
- `set_gait_pattern(GaitPattern.TRIPOD/WAVE/RIPPLE)` sets `phase_offset` based on leg instance index, then calls `_generate_gait_cycle()` which uses `np.roll` to shift the trajectory.
- **Known bug**: `stationary_leg_equation()` references `self.iksolver.y0` which does not exist on `IkSolver`.

### Plotter (`Scripts/plotter.py`)

- Stateless except for `plot_index` (used to number matplotlib figures).
- `plot_animated_3d_projection` accepts either a single solver or a list — when given a list it iterates all legs and animates them concurrently via `FuncAnimation`.
- Both `IkSolver` and `GaitController` satisfy the duck-typed interface expected by `Plotter` (`get_motion()` and `solve_leg_position_from_target_coordinates()`).

### Tests (`Unittests/`)

Tests use `unittest` and are discovered via `Unittests/__init__.py`. `setUpClass` is used in `GaitControllerTests` to create a single shared `IkSolver` instance across all test methods — this is intentional because `IkSolver` has a 6-instance limit. Tests use `@unittest.skipIf` flags to skip dependent tests when earlier ones fail.

### Arduino firmware (`arduino/hexapod/`)

The firmware is a C++ port of the Python IK and gait logic targeting an Arduino Uno R3.

**Hardware:** 18× SG90/MG90S servos (3 per leg), driven by two PCA9685 I2C PWM boards (0x40, 0x41) at 50 Hz. External 5 V / 10 A PSU required — never power servos from the Uno's 5 V pin.

**Key architectural difference from Python:** there are no pre-computed trajectory arrays (would cost ~3600 B of the Uno's 2 KB SRAM). Foot targets are computed analytically from the current phase on every tick. Memory budget: 6 `Leg` structs (264 B) + `GaitConfig` (20 B); servo channel map stored in PROGMEM (zero SRAM).

**Module responsibilities:**
- `types.h` — `Leg` and `GaitConfig` structs, `GaitPattern` constants
- `ik.h/.cpp` — direct port of `solve_inverse_kinematics()`; returns `bool` for reachability instead of throwing
- `gait.h/.cpp` — phase-based foot target computation, delegates to IK, advances phase each tick
- `servo_driver.h/.cpp` — PCA9685 init + `angle_to_ticks()` (radians → 12-bit PWM); left-side legs (3–5) have `invert=true` in the servo map to compensate for mirror mounting
- `serial_cmd.h/.cpp` — non-blocking single-character serial command parser
- `config.h` — all constants; kept in sync with Python defaults for sim/firmware parity

**Step length discrepancy:** Python `GaitController` defaults to `step_length = 4.0` cm; Arduino `config.h` uses `STEP_LENGTH = 2.0` cm. This is a known divergence and has not been harmonised.

### Browser simulation (`simulation.html`)

Self-contained Three.js 3D visualisation — no build step, no server. Re-implements the same IK and gait math in JavaScript. Key constants: `LEG_FORE_AFT = 1.5` cm fore/aft separation (increased from 0.75 to prevent tripod inter-leg collisions in the worst-case 180° out-of-phase configuration), `BASE_Y = 1.5` cm lateral offset.
