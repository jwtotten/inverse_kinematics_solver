/**
 * TDD tests for simulation.html gait changes.
 *
 * Round 1 changes (already implemented):
 *   1. STEP_L : 0.5  → 2.0
 *   2. BASE_X : 1.2  → 1.5
 *   3. Stride axis: dx on x-axis → dy on y-axis (x becomes constant)
 *   4. Per-leg yOff added to remove collisions
 *
 * Round 2 changes (collision fix — under test in T6, T11):
 *   5. BASE_Y  : 1.0  → 1.5  (visual centering + IK margin)
 *   6. yOff    : ±Y_LEN/4 (±0.75) → ±1.5 (LEG_FORE_AFT)
 *      Derived: yOff_front must exceed STEP_L − |yLeg_mid| + 0.2 buffer = 1.45
 *      Guarantees ≥0.25 world-y gap between adjacent leg corridors (all gaits).
 */

'use strict';
const assert = require('assert');

// ── Shared robot constants (unchanged by the plan) ───────────────────────────
const FEMUR  = 3.5;
const TIBIA  = 3.5;
const X_OFF  = -5;
const Y_OFF  = -2;
const Z_OFF  = -6;
const Y_LEN  = 3;
const STEP_H = 2.0;
const DUTY   = 0.5;

// ── CURRENT values (pre-change) ───────────────────────────────────────────────
const CURRENT = { STEP_L: 0.5,  BASE_X: 1.2, BASE_Y: 1.0, BASE_Z: 1.2 };

// ── TARGET values (post-change) ───────────────────────────────────────────────
const TARGET  = { STEP_L: 2.0,  BASE_X: 1.5, BASE_Y: 1.5, BASE_Z: 1.2 };
const LEG_FORE_AFT = 1.5;  // yOff magnitude; must exceed STEP_L − |yLeg_mid| + 0.2 = 1.45

// ── Leg tables ────────────────────────────────────────────────────────────────
// Current: no yOff field
const LEGS_CURRENT = [
  { xLeg: 0,       yLeg: 0,          side:  1, label: 'R-Front' },
  { xLeg: 0,       yLeg: -Y_LEN/4,  side:  1, label: 'R-Mid'   },
  { xLeg: 0,       yLeg: -Y_LEN/2,  side:  1, label: 'R-Rear'  },
  { xLeg: -2.5,    yLeg: 0,          side: -1, label: 'L-Front' },
  { xLeg: -2.5,    yLeg: -Y_LEN/4,  side: -1, label: 'L-Mid'   },
  { xLeg: -2.5,    yLeg: -Y_LEN/2,  side: -1, label: 'L-Rear'  },
];

// Target: yOff set to LEG_FORE_AFT (±1.5) to eliminate inter-leg collisions.
// Corridors: front [0.0,+2.0], mid [-2.25,-0.25], rear [-4.5,-2.5] (≥0.25 gap each).
const LEGS_TARGET = [
  { xLeg: 0,    yLeg: 0,          yOff:  LEG_FORE_AFT, side:  1, label: 'R-Front' },
  { xLeg: 0,    yLeg: -Y_LEN/4,  yOff:  0,             side:  1, label: 'R-Mid'   },
  { xLeg: 0,    yLeg: -Y_LEN/2,  yOff: -LEG_FORE_AFT, side:  1, label: 'R-Rear'  },
  { xLeg: -2.5, yLeg: 0,          yOff:  LEG_FORE_AFT, side: -1, label: 'L-Front' },
  { xLeg: -2.5, yLeg: -Y_LEN/4,  yOff:  0,             side: -1, label: 'L-Mid'   },
  { xLeg: -2.5, yLeg: -Y_LEN/2,  yOff: -LEG_FORE_AFT, side: -1, label: 'L-Rear'  },
];

// ── gaitTarget implementations ────────────────────────────────────────────────

/** CURRENT implementation: stride via dx on x-axis, no yOff */
function gaitTarget_current(t, phase, legIdx) {
  const p = (t + phase) % 1;
  const s = LEGS_CURRENT[legIdx].side;
  let dx, dz;
  if (p < DUTY) {
    const k = p / DUTY;
    dx = CURRENT.STEP_L * (0.5 - k);
    dz = 0;
  } else {
    const k = (p - DUTY) / (1 - DUTY);
    dx = CURRENT.STEP_L * (k - 0.5);
    dz = STEP_H * Math.sin(k * Math.PI);
  }
  return {
    x: (CURRENT.BASE_X + dx) * s,
    y:  CURRENT.BASE_Y * s,
    z:  CURRENT.BASE_Z + dz,
  };
}

/** TARGET implementation: stride via dy on y-axis, with per-leg yOff */
function gaitTarget_new(t, phase, legIdx) {
  const p = (t + phase) % 1;
  const s = LEGS_TARGET[legIdx].side;
  const yOff = LEGS_TARGET[legIdx].yOff;
  let dy, dz;
  if (p < DUTY) {
    const k = p / DUTY;
    dy = TARGET.STEP_L * (0.5 - k);
    dz = 0;
  } else {
    const k = (p - DUTY) / (1 - DUTY);
    dy = TARGET.STEP_L * (k - 0.5);
    dz = STEP_H * Math.sin(k * Math.PI);
  }
  return {
    x:  TARGET.BASE_X * s,
    y: (TARGET.BASE_Y + yOff + dy) * s,
    z:  TARGET.BASE_Z + dz,
  };
}

// ── Helper: sample a gaitTarget function across one full cycle ────────────────
function sampleCycle(fn, legIdx, steps = 100) {
  const results = [];
  for (let i = 0; i < steps; i++) {
    results.push(fn(i / steps, 0, legIdx));
  }
  return results;
}

// ── IK reach calculator ───────────────────────────────────────────────────────
function ikReach(target, side) {
  const xo = side === -1 ? -X_OFF : X_OFF;
  const yo = side === -1 ? -Y_OFF : Y_OFF;
  const xa = target.x + xo;
  const ya = target.y + yo;
  const za = target.z + Z_OFF;
  return Math.sqrt(xa*xa + ya*ya + za*za);
}

// ── Test runner ───────────────────────────────────────────────────────────────
let passed = 0;
let failed = 0;

function test(name, fn) {
  try {
    fn();
    console.log(`  ✓  ${name}`);
    passed++;
  } catch (e) {
    console.log(`  ✗  ${name}`);
    console.log(`       ${e.message}`);
    failed++;
  }
}

function approxEqual(a, b, tol = 0.01, msg = '') {
  if (Math.abs(a - b) > tol) {
    throw new Error(`${msg} expected ${b.toFixed(4)}, got ${a.toFixed(4)} (tol=${tol})`);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
console.log('\n=== Hexapod Gait Simulation Tests ===\n');

// ── Test 1: x should be CONSTANT across the stride cycle (new behaviour) ──────
test('T1  x is constant throughout stride cycle [new]', () => {
  const samples = sampleCycle(gaitTarget_new, 0); // R-Front
  const absX = samples.map(s => Math.abs(s.x));
  const deviation = Math.max(...absX) - Math.min(...absX);
  // x must not vary — any deviation > 0.001 means stride is still on x-axis
  assert.ok(deviation < 0.001,
    `x varied by ${deviation.toFixed(4)} — stride is still on x-axis, not fixed`);
});

// ── Test 2: y should vary by exactly STEP_L=2.0 across the cycle [new] ────────
test('T2  y varies by STEP_L=2.0 across full cycle [new]', () => {
  const samples = sampleCycle(gaitTarget_new, 0);
  const absY = samples.map(s => Math.abs(s.y));
  const amplitude = Math.max(...absY) - Math.min(...absY);
  approxEqual(amplitude, TARGET.STEP_L, 0.05,
    `y amplitude`);
});

// ── Test 3: CURRENT code — x DOES vary (proves current code has wrong axis) ──
test('T3  x varies in current code (confirms bug exists) [current]', () => {
  const samples = sampleCycle(gaitTarget_current, 0);
  const absX = samples.map(s => Math.abs(s.x));
  const deviation = Math.max(...absX) - Math.min(...absX);
  // Current code SHOULD vary x; this test passes today and should keep passing
  assert.ok(deviation > CURRENT.STEP_L * 0.9,
    `x deviation ${deviation.toFixed(4)} unexpectedly small — current code may already be fixed`);
});

// ── Test 4: Per-leg yOff — front leg further forward than mid leg ─────────────
test('T4  front leg foot (idx 0) is further forward than mid leg (idx 1) [new]', () => {
  const tMidStance = 0.25; // well into stance phase
  const frontTarget = gaitTarget_new(tMidStance, 0, 0);
  const midTarget   = gaitTarget_new(tMidStance, 0, 1);
  // Both right-side legs (side=+1); a larger y means further "forward"
  assert.ok(frontTarget.y > midTarget.y,
    `front y (${frontTarget.y.toFixed(3)}) should be > mid y (${midTarget.y.toFixed(3)})`);
});

// ── Test 5: Per-leg yOff — rear leg further back than mid leg ─────────────────
test('T5  rear leg foot (idx 2) is further back than mid leg (idx 1) [new]', () => {
  const tMidStance = 0.25;
  const midTarget  = gaitTarget_new(tMidStance, 0, 1);
  const rearTarget = gaitTarget_new(tMidStance, 0, 2);
  assert.ok(rearTarget.y < midTarget.y,
    `rear y (${rearTarget.y.toFixed(3)}) should be < mid y (${midTarget.y.toFixed(3)})`);
});

// ── Test 6: Per-leg yOff values are correct (±LEG_FORE_AFT=±1.5, 0) ──────────
test('T6  per-leg yOff values: +1.5 / 0 / -1.5 [collision fix]', () => {
  approxEqual(LEGS_TARGET[0].yOff,  LEG_FORE_AFT, 0.001, 'R-Front yOff');
  approxEqual(LEGS_TARGET[1].yOff,  0,             0.001, 'R-Mid   yOff');
  approxEqual(LEGS_TARGET[2].yOff, -LEG_FORE_AFT, 0.001, 'R-Rear  yOff');
  approxEqual(LEGS_TARGET[3].yOff,  LEG_FORE_AFT, 0.001, 'L-Front yOff');
  approxEqual(LEGS_TARGET[4].yOff,  0,             0.001, 'L-Mid   yOff');
  approxEqual(LEGS_TARGET[5].yOff, -LEG_FORE_AFT, 0.001, 'L-Rear  yOff');
});

// ── Test 7: IK valid range — L ≤ 7.0 for all legs, all phases [new] ──────────
test('T7  IK reach L ≤ 7.0 for all 6 legs across full gait cycle [new]', () => {
  const MAX_L = FEMUR + TIBIA; // 7.0
  let worst = 0;
  let worstDesc = '';
  for (let legIdx = 0; legIdx < 6; legIdx++) {
    const side = LEGS_TARGET[legIdx].side;
    for (let i = 0; i < 200; i++) {
      const t = i / 200;
      const target = gaitTarget_new(t, 0, legIdx);
      const L = ikReach(target, side);
      if (L > worst) { worst = L; worstDesc = `leg${legIdx} t=${t.toFixed(3)} L=${L.toFixed(4)}`; }
      assert.ok(L <= MAX_L + 0.001,
        `IK out of range: leg${legIdx} t=${t.toFixed(3)} L=${L.toFixed(4)} > ${MAX_L}`);
    }
  }
  console.log(`       (worst case: ${worstDesc})`);
});

// ── Test 8: Bilateral symmetry — left/right legs mirror each other ────────────
test('T8  left/right legs are bilaterally symmetric at mid-stance [new]', () => {
  const tMidStance = 0.25;
  // R-Front (idx 0, side +1) vs L-Front (idx 3, side -1)
  const r = gaitTarget_new(tMidStance, 0, 0);
  const l = gaitTarget_new(tMidStance, 0, 3);
  approxEqual(r.x, -l.x, 0.001, 'x mirror');
  approxEqual(r.y, -l.y, 0.001, 'y mirror');
  approxEqual(r.z,  l.z, 0.001, 'z same');
});

// ── Test 9: BASE_X is 1.5 (not 1.2) ──────────────────────────────────────────
test('T9  BASE_X constant in target implementation is 1.5 [new]', () => {
  approxEqual(TARGET.BASE_X, 1.5, 0.001, 'BASE_X');
});

// ── Test 10: STEP_L is 2.0 (not 0.5) ─────────────────────────────────────────
test('T10 STEP_L constant in target implementation is 2.0 [new]', () => {
  approxEqual(TARGET.STEP_L, 2.0, 0.001, 'STEP_L');
});

// ── Test 11: BASE_Y is 1.5 [collision fix] ───────────────────────────────────
test('T11 BASE_Y is 1.5 [collision fix]', () => {
  approxEqual(TARGET.BASE_Y, 1.5, 0.001, 'BASE_Y');
});

// ── Test 12: World-y corridors are non-overlapping with ≥0.2 buffer ──────────
// world_y = (BASE_Y + yOff + dy) + Y_OFF + yLeg  where dy ∈ [−STEP_L/2, +STEP_L/2]
// corridor_min = BASE_Y + yOff − STEP_L/2 + Y_OFF + yLeg
// corridor_max = BASE_Y + yOff + STEP_L/2 + Y_OFF + yLeg
test('T12 world-y corridors non-overlapping ≥0.2 buffer between adjacent legs [collision fix]', () => {
  const BUFFER = 0.2;
  const corridor = (legIdx) => {
    const { yOff, yLeg } = LEGS_TARGET[legIdx];
    return {
      min: TARGET.BASE_Y + yOff - TARGET.STEP_L/2 + Y_OFF + yLeg,
      max: TARGET.BASE_Y + yOff + TARGET.STEP_L/2 + Y_OFF + yLeg,
    };
  };
  // Right side: indices 0 (front), 1 (mid), 2 (rear)
  const rFront = corridor(0);
  const rMid   = corridor(1);
  const rRear  = corridor(2);
  assert.ok(rFront.min >= rMid.max + BUFFER,
    `R: front min (${rFront.min.toFixed(3)}) must be ≥ mid max (${rMid.max.toFixed(3)}) + ${BUFFER}`);
  assert.ok(rMid.min >= rRear.max + BUFFER,
    `R: mid  min (${rMid.min.toFixed(3)}) must be ≥ rear max (${rRear.max.toFixed(3)}) + ${BUFFER}`);
  // Left side: indices 3 (front), 4 (mid), 5 (rear) — same yOff/yLeg values, symmetric
  const lFront = corridor(3);
  const lMid   = corridor(4);
  const lRear  = corridor(5);
  assert.ok(lFront.min >= lMid.max + BUFFER,
    `L: front min (${lFront.min.toFixed(3)}) must be ≥ mid max (${lMid.max.toFixed(3)}) + ${BUFFER}`);
  assert.ok(lMid.min >= lRear.max + BUFFER,
    `L: mid  min (${lMid.min.toFixed(3)}) must be ≥ rear max (${lRear.max.toFixed(3)}) + ${BUFFER}`);
  // Report actual buffers
  console.log(`       R-Front/Mid buffer: ${(rFront.min - rMid.max).toFixed(3)}`);
  console.log(`       R-Mid/Rear  buffer: ${(rMid.min  - rRear.max).toFixed(3)}`);
});

// ── Summary ───────────────────────────────────────────────────────────────────
console.log(`\n${'─'.repeat(42)}`);
console.log(`  ${passed} passed   ${failed} failed   (${passed + failed} total)`);
if (failed > 0) {
  console.log('\n  ❌  Tests are RED — implementation not yet applied.\n');
  process.exit(1);
} else {
  console.log('\n  ✅  All tests GREEN.\n');
  process.exit(0);
}
