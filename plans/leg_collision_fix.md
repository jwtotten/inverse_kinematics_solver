# Plan: Fix Leg Collisions in Browser Simulation

**Branch:** `feature/arduino_program`  
**File under change:** `simulation.html`, `tests/test_gait_simulation.js`

---

## Problem

When viewing the hexapod simulation from a high elevation, adjacent legs on the
same side of the body collide (feet pass through each other) depending on the
gait pattern selected.

---

## Root-Cause Analysis

### World-space foot y position formula

The FK + body-offset pipeline means the world-y of a foot is:

```
world_y = FK_y + yLeg
        ≈ (y_target + Y_OFF) + yLeg
        = (BASE_Y + yOff + dy − 2) + yLeg
```

### Current world-y corridors (BASE_Y=1.0, yOff=±0.75, STEP_L=2.0, dy∈[−1,+1])

| Leg      | yOff  | yLeg  | world y range    |
|----------|-------|-------|------------------|
| R-Front  | +0.75 |  0    | [−1.25, +0.75]   |
| R-Mid    |  0    | −0.75 | [−2.75, −0.75]   |
| R-Rear   | −0.75 | −1.5  | [−4.25, −2.25]   |

**Front–Mid overlap: 0.5 units** (−1.25 to −0.75).  
**Mid–Rear overlap: 0.5 units** (−2.75 to −2.25). ← confirms collision.

### Which gait patterns collide?

Collision requires `max[dy_adjacent − dy_leg] ≥ 1.5` (the world-y separation
between adjacent corridors at the current yOff/yLeg values).

The gait trajectory `dy` is a triangle wave with amplitude STEP_L/2.
For adjacent legs with phase offset Δφ:

| Gait   | Δφ (adj. same side) | max[dy_j − dy_i]  | Collides? |
|--------|---------------------|-------------------|-----------|
| Tripod | 0.5                 | STEP_L = **2.0**  | ✗ YES     |
| Ripple | 0.25                | STEP_L/2 = 1.0    | ✓ no      |
| Wave   | 1/6                 | STEP_L/3 = 0.67   | ✓ no      |

**Tripod is the worst case** — adjacent legs have exactly opposite phases,
so one foot is at maximum forward (+1.0) while the other is at maximum
backward (−1.0), giving a difference of 2.0 > 1.5.

---

## Fix

### Required condition (non-overlapping corridors with buffer B=0.2)

```
front_corridor_min  >  mid_corridor_max + B
(BASE_Y + yOff_front − STEP_L/2 + Y_OFF + yLeg_front) > (BASE_Y + yOff_mid + STEP_L/2 + Y_OFF + yLeg_mid) + B
yOff_front > STEP_L − |yLeg_mid| − yLeg_front + B
yOff_front > 2.0 − 0.75 − 0 + 0.2
yOff_front > 1.45
```

→ Use `yOff_front = 1.5`, `yOff_mid = 0`, `yOff_rear = −1.5`.  
→ Express as new constant: `LEG_FORE_AFT = 1.5`.

Also raise `BASE_Y` from 1.0 → 1.5 for better visual foot placement and extra
IK headroom.

### Revised world-y corridors (BASE_Y=1.5, yOff=±1.5)

| Leg     | world y range  | Gap to neighbour |
|---------|----------------|-----------------|
| R-Front | [0.0,  +2.0]   | —               |
| R-Mid   | [−2.25, −0.25] | **+0.25 buffer** above R-Mid max ✓ |
| R-Rear  | [−4.5,  −2.5]  | **+0.25 buffer** above R-Rear max ✓ |

### IK validity check

With `BASE_Y=1.5`, `yOff=±1.5`, `STEP_L=2.0`:

| Case                   | xa   | ya   | za   | L     | Valid? |
|------------------------|------|------|------|-------|--------|
| Rear leg, ground min y | −3.5 | −3.0 | −4.8 | 6.65  | ✅     |
| Front leg, ground max y| −3.5 | +1.0 | −4.8 | 6.07  | ✅     |
| Any leg, peak swing    | −3.5 | −1.5 | −2.8 | 4.69  | ✅     |

Worst-case L = **6.65 < 7.0** (FEMUR + TIBIA). Comfortable margin.

---

## Implementation Steps

### Step 1 — Update tests (TDD red phase)

File: `tests/test_gait_simulation.js`

1. Update `TARGET` constants block:
   - `BASE_Y`: 1.0 → 1.5
2. Update `LEGS_TARGET` array `yOff` values:
   - Front/L-Front: `Y_LEN/4` (0.75) → `1.5`
   - Mid/L-Mid:     `0` → `0` (unchanged)
   - Rear/L-Rear:   `−Y_LEN/4` (−0.75) → `−1.5`
3. Add **Test T11** — world-space corridor non-overlap:
   ```javascript
   test('T11 world-y corridors are non-overlapping with ≥0.2 buffer [new]', () => {
     const BUFFER = 0.2;
     // Compute world-y min/max for each right-side leg
     // world_y = (BASE_Y + yOff + dy) + Y_OFF + yLeg
     const corridor = (yOff, yLeg) => ({
       min: TARGET.BASE_Y + yOff - TARGET.STEP_L/2 + Y_OFF + yLeg,
       max: TARGET.BASE_Y + yOff + TARGET.STEP_L/2 + Y_OFF + yLeg,
     });
     const front = corridor(LEGS_TARGET[0].yOff, LEGS_TARGET[0].yLeg);
     const mid   = corridor(LEGS_TARGET[1].yOff, LEGS_TARGET[1].yLeg);
     const rear  = corridor(LEGS_TARGET[2].yOff, LEGS_TARGET[2].yLeg);
     assert.ok(front.min > mid.max + BUFFER,
       `Front min (${front.min.toFixed(3)}) must be > Mid max (${mid.max.toFixed(3)}) + ${BUFFER}`);
     assert.ok(mid.min > rear.max + BUFFER,
       `Mid min (${mid.min.toFixed(3)}) must be > Rear max (${rear.max.toFixed(3)}) + ${BUFFER}`);
   });
   ```
4. Run tests → expect T11 to **fail** (red), T7 may fail with new TARGET constants.

### Step 2 — Implement changes in simulation.html

1. Change `BASE_Y`: `1.0` → `1.5`
2. Add new constant: `const LEG_FORE_AFT = 1.5;`  
   with comment: `// derived: must exceed STEP_L − |yLeg_mid| + 0.2 buffer = 1.45`
3. Update LEGS array `yOff` fields:
   - Front rows (idx 0, 3): `Y_LEN/4` → `LEG_FORE_AFT`
   - Mid rows   (idx 1, 4): `0` → `0` (no change)
   - Rear rows  (idx 2, 5): `−Y_LEN/4` → `−LEG_FORE_AFT`
4. Add a comment block above the LEGS array documenting the corridor analysis.

### Step 3 — Run tests (green phase)

```bash
node tests/test_gait_simulation.js   # expect 11/11 pass
python3 unittesting.py               # expect 11/11 pass (no regressions)
```

### Step 4 — Preview verification

Open simulation.html in preview at high elevation (55°+) and cycle through
Tripod / Ripple / Wave gaits. Legs must not visually cross in any gait.

### Step 5 — Commit

```
fix: eliminate inter-leg collisions by widening fore/aft foot spread

Raises yOff from ±0.75 (Y_LEN/4) to ±1.5 (LEG_FORE_AFT) and BASE_Y
from 1.0 to 1.5. This gives each adjacent leg pair a 0.25-unit world-y
buffer in all gait patterns including worst-case tripod (Δφ=0.5).
Worst-case IK reach L=6.65 < FEMUR+TIBIA=7.0. Adds T11 corridor test.
```

---

## Key Numbers Summary

| Constant       | Before | After  | Why                               |
|----------------|--------|--------|-----------------------------------|
| `BASE_Y`       | 1.0    | 1.5    | Visual centering + IK margin      |
| `yOff` (front) | +0.75  | +1.5   | Non-overlap condition requires >1.45 |
| `yOff` (mid)   |  0     |  0     | Unchanged — reference point       |
| `yOff` (rear)  | −0.75  | −1.5   | Symmetric with front              |
| World buffer   | −0.5 (overlap!) | +0.25 | Fixed                  |
| Worst-case L   | 6.90   | 6.65   | More comfortable                  |
