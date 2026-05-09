# Plan: Hexapod Leg Crossing Detection

## Context

The R-Rear and R-Front legs are visually crossing in the 3D preview. This plan adds detection logic to identify when any two leg segments (across all 6 legs) cross or touch each other, with visual feedback in the animation. There are currently zero collision checks in the codebase.

---

## Key Files

| File | Role |
|---|---|
| `Scripts/plotter.py` | 3D animation — integration point for the checker |
| `Scripts/iksolver.py` | Leg coordinate output (`solve_leg_position_from_target_coordinates`) |
| `Unittests/__init__.py` | Test runner — needs update to include new tests |
| `multi_leg_simulation.py` | Entry point — passes list of 6 legs to `plot_animated_3d_projection()` |

> **Note**: `Scripts/` has no `__init__.py`. All imports must be **absolute** (`from Scripts.geometry_utils import ...`), not relative. The test runner `unittesting.py` already uses absolute imports.

---

## Step 1 — Create `Scripts/geometry_utils.py`

Pure math module. No IkSolver dependency. Uses numpy only.

### Functions

**`segment_to_segment_distance(p0, p1, q0, q1) -> float`**  
Returns the minimum Euclidean distance between two finite 3D line segments using the Eberly parametric algorithm. Clamps both parameters `s` and `t` to `[0, 1]` to stay within segment bounds. Guards against degenerate (zero-length) segments with `EPSILON = 1e-10`.

Core math:
```
d1 = p1 - p0;  d2 = q1 - q0;  r = p0 - q0
a = dot(d1,d1);  e = dot(d2,d2);  f = dot(d2,r)
# Handle degenerate cases (either segment is a point)
# Otherwise solve for s,t that minimise |p0+s*d1 - q0+t*d2|^2
b = dot(d1,d2);  denom = a*e - b*b
s = clamp((b*f - c*e) / denom, 0, 1)
t = (b*s + f) / e; if t < 0 or t > 1, recompute clamped
return norm((p0 + s*d1) - (q0 + t*d2))
```

**`segments_are_crossing(p0, p1, q0, q1, threshold=0.05) -> bool`**  
Returns `True` if `segment_to_segment_distance` is below `threshold`. Default 0.05 is ~1.4% of the 3.5-unit leg length.

**`build_segment_list(coordinates: list) -> list[tuple[np.ndarray, np.ndarray]]`**  
Converts the 3-point output of `solve_leg_position_from_target_coordinates` into two `(start, end)` numpy array pairs:
- `(hip, knee)` — femur segment
- `(knee, foot)` — tibia segment

---

## Step 2 — Create `Scripts/leg_collision_checker.py`

Accepts resolved coordinate lists; does not call IK or import IkSolver.

### `CollisionResult` dataclass
Fields: `leg_a_index: int`, `leg_b_index: int`, `segment_a_name: str`, `segment_b_name: str`, `distance: float`, `is_crossing: bool`

### Functions

**`check_all_leg_collisions(all_leg_coordinates, threshold=0.05) -> list[CollisionResult]`**  
- Iterates `itertools.combinations(range(6), 2)` → 15 unique leg pairs
- For each pair checks all 4 segment combos: femur-femur, femur-tibia, tibia-femur, tibia-tibia
- Returns only results where `is_crossing=True`

**`get_crossing_leg_indices(all_leg_coordinates, threshold=0.05) -> list[int]`**  
Convenience wrapper. Returns flat set of 0-based leg indices involved in any collision — used by the plotter for colour assignment.

---

## Step 3 — Modify `Scripts/plotter.py`

Integration point: `update_animated_plot` inside `plot_animated_3d_projection()` (lines ~233–266). **3D multi-leg branch only** — do not touch the 2D method.

**Add import at top of file:**
```python
from Scripts.leg_collision_checker import get_crossing_leg_indices
```

**Inside the existing leg loop** (after `leg_plots[idx].set_data_3d(x, y, z)`), collect coordinates into `all_coords`. After the loop ends:
```python
crossing_indices = get_crossing_leg_indices(all_coords, threshold=0.05)
for idx in range(len(ik_solver)):
    colour = 'red' if idx in crossing_indices else 'blue'
    leg_plots[idx].set_color(colour)
if crossing_indices:
    print(f"Frame {i}: crossing legs {sorted(crossing_indices)}")
```

Also fix the existing bug: the current `return left_plot` (~line 251) should return `leg_plots` to correctly inform matplotlib's blit renderer when handling multiple legs.

---

## Step 4 — Create `Unittests/leg_crossing_tests.py`

Two test classes. Neither instantiates `IkSolver` (avoids the 6-instance global limit).

### `GeometryUtilsTests(unittest.TestCase)`
Tests for `segment_to_segment_distance` and helpers using known-answer geometry:
- Parallel segments → perpendicular distance
- Crossing segments at origin → distance = 0
- Skew segments at z=1 offset → distance = 1.0
- Segments sharing an endpoint → distance = 0
- Degenerate point segment → distance to nearest segment point
- `segments_are_crossing` true/false cases
- `build_segment_list` output shape and values

### `LegCollisionCheckerTests(unittest.TestCase)`
Uses a `_make_straight_leg(origin_x, origin_y, reach_x, reach_y)` helper generating synthetic `[[hip],[knee],[foot]]` lists:
- Six spread-apart legs → no crossing detected
- Six co-located legs → all 6 indices flagged
- `CollisionResult` fields have correct types
- Only legs 0 and 1 overlapping → only indices 0 and 1 flagged; legs 2–5 clean
- Same-leg segments not compared against each other (avoids false knee-joint flags)

### Update `Unittests/__init__.py`
Add import and include `GeometryUtilsTests`, `LegCollisionCheckerTests` in `__all__`.

---

## Implementation Order

1. `Scripts/geometry_utils.py` — no dependencies, highest-risk math
2. Run `GeometryUtilsTests` to verify the math before proceeding
3. `Scripts/leg_collision_checker.py` — depends on geometry_utils
4. `Unittests/leg_crossing_tests.py` + update `Unittests/__init__.py`
5. Run full test suite: `python unittesting.py`
6. Modify `Scripts/plotter.py`
7. Run `python multi_leg_simulation.py` — confirm crossing legs turn red

---

## Verification

```bash
# Unit tests
python unittesting.py

# Visual check
python multi_leg_simulation.py
```

Expected outcome: crossing legs render red in the 3D animation window; console prints frame number and leg indices whenever a crossing is detected.
