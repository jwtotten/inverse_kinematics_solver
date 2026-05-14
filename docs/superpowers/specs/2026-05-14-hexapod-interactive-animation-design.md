# Hexapod Interactive Animation Design

**Date:** 2026-05-14

## Context

`multi_leg_simulation.py` currently shows a static 3D animation of all 6 legs cycling in TRIPOD gait with no user interaction and no body translation. This design adds `interactive_simulation.py`: the hexapod body physically translates through space (forward/backward), with real-time gait switching (TRIPOD/WAVE/RIPPLE), direction control, and speed control via matplotlib widgets and keyboard shortcuts. Existing files are untouched.

## Architecture

**New files:**
- `Scripts/animation_controller.py` — pure functions, no matplotlib dependency, fully testable
- `interactive_simulation.py` — thin matplotlib layer, owns FuncAnimation, wires widgets to animation_controller functions
- `Unittests/animation_controller_tests.py` — TDD tests for animation_controller

**Existing files used (read-only):**
- `Scripts/gait_controller.py` — GaitController, GaitPattern, set_gait_pattern(), _generate_gait_cycle(), step_length
- `Scripts/iksolver.py` — IkSolver, solve_leg_position_from_target_coordinates()
- `Scripts/leg_collision_checker.py` — get_crossing_leg_indices()

**Existing files NOT touched:**
- `Scripts/plotter.py`, `multi_leg_simulation.py`, `animated_simulation.py`

## State (owned by interactive_simulation.py)

```python
body_offset: float    # accumulated world-X position of robot body
direction: int        # 1 (fwd) | 0 (stop) | -1 (back)
speed: float          # 0.2–3.0, frame-advance multiplier
frame_counter: float  # fractional, advances by speed each tick; mod num_samples
current_gait: GaitPattern
gait_controllers: list[GaitController]  # 6 controllers
step_length_abs: float  # always positive (4.0), sign applied via direction
```

## animation_controller.py — Pure Functions

```python
def advance_frame(frame_counter: float, speed: float, num_samples: int) -> float
def compute_body_offset(current_offset: float, direction: int, step_length: float, num_samples: int) -> float
def clamp_speed(speed: float) -> float  # clamps to [0.2, 3.0]
def apply_direction_to_controllers(controllers: list, direction: int, step_length_abs: float) -> None
def apply_gait_to_controllers(controllers: list, pattern) -> None
def translate_joints(joints: list, body_offset: float) -> list
def build_ground_grid(body_offset: float, spacing: float = 1.0, half_width: float = 12.0) -> tuple
```

## Figure Layout

```
┌─────────────────────────────┬──────────────┐
│                             │  GAIT        │
│      3D Axes                │  ○ Tripod    │
│   (camera follows body,     │  ○ Wave      │
│    ground grid scrolls)     │  ○ Ripple    │
│                             │              │
│                             │  [Forward]   │
│                             │  [ Stop  ]   │
│                             │  [Backward]  │
│                             │              │
│                             │  Speed       │
│                             │  ──●────     │
└─────────────────────────────┴──────────────┘
```

- `xlim = [body_offset - 12, body_offset + 12]` (updates each frame, camera follows body)
- `ylim = [-10, 10]`, `zlim = [-2, 8]`
- Ground grid: grey dashed lines at z=0, 1 cm spacing, 24 cm window, redrawn each frame
- `blit=False` (xlim changes each frame)
- FuncAnimation interval: 50ms

## Controls

**Direction (button or keyboard):**
1. Set `direction` = 1 / -1 / 0
2. `apply_direction_to_controllers(gait_controllers, direction, step_length_abs)`
3. frame_counter continues uninterrupted

**Gait switch (RadioButton or keyboard):**
1. `apply_gait_to_controllers(gait_controllers, new_pattern)`
2. frame_counter continues

**Speed (Slider or keyboard):**
1. `clamp_speed(new_speed)` → update speed
2. Slider and keyboard `+`/`-` sync each other
3. `+`/`-` step: 0.2

**Keyboard bindings:**
- `f` → Forward, `b` → Backward, `s` → Stop
- `t` → TRIPOD, `w` → WAVE, `r` → RIPPLE
- `+` / `=` → speed up, `-` → speed down

**Stop behavior:** direction=0, legs still cycle, body does not translate.

## update() Loop (per frame)

```
1. frame_counter = advance_frame(frame_counter, speed, num_samples)
2. i = int(frame_counter)
3. body_offset = compute_body_offset(body_offset, direction, step_length_abs, num_samples)
4. For each gc: get joint positions via gc.get_motion()[axis][i] → solve_leg_position_from_target_coordinates()
5. translate_joints(joints, body_offset) on each leg result
6. collision check via get_crossing_leg_indices()
7. update 3D artists (set_data_3d)
8. redraw ground grid via build_ground_grid(body_offset)
9. update ax.set_xlim([body_offset - 12, body_offset + 12])
```

## Visual Style

- Legs: blue lines + red joint markers; collision legs turn red
- Body box: green reference lines, translated by body_offset
- Ground grid: grey dashed lines at z=0
- Title: `"F/B/S: direction  T/W/R: gait  +/-: speed"`

## Verification

Run `python interactive_simulation.py`. Check:
1. Legs animate in TRIPOD on launch
2. F key / Forward button → body scrolls +X, ground scrolls past
3. B key / Backward button → body scrolls -X
4. S / Stop → body halts, legs still cycle
5. T/W/R → gait phase offsets change visibly mid-animation
6. RadioButtons sync with keyboard gait commands
7. +/- → speed changes, slider tracks
8. Slider updates speed, +/- syncs slider
9. Collision detection: crossing legs turn red
10. Axes xlim follows body_offset

## TDD Approach

Pure functions in `animation_controller.py` are test-driven before implementation. Integration functions (`apply_direction_to_controllers`, `apply_gait_to_controllers`) use real GaitController/IkSolver — no mocks.
