"""
Six-leg hexapod controller for Raspberry Pi 5 with two Adafruit PCA9685 boards.

Copy this file to the Pi. Install deps:
    pip install adafruit-circuitpython-pca9685 adafruit-blinka numpy

Wiring:
    Right PCA9685 at I2C 0x41 (A0 jumper set) — legs 0-2
        Leg 0: channels 0 (coxa), 1 (femur), 2 (tibia)
        Leg 1: channels 3 (coxa), 4 (femur), 5 (tibia)
        Leg 2: channels 6 (coxa), 7 (femur), 8 (tibia)
    Left PCA9685 at I2C 0x40 (factory default) — legs 3-5
        Leg 3: channels 0 (coxa), 1 (femur), 2 (tibia)
        Leg 4: channels 3 (coxa), 4 (femur), 5 (tibia)
        Leg 5: channels 6 (coxa), 7 (femur), 8 (tibia)
    Both boards share SDA/SCL (pins 3/5 on Pi)

Run:        python leg_controller.py
Calibrate:  python leg_controller.py --calibrate
"""

import math
import time
import sys
import argparse
import numpy as np

# --- CONFIGURABLE ---
CYCLE_FREQ = 1.0    # Hz — full gait cycles per second

# I2C addresses for both PCA9685 boards
PCA_RIGHT_ADDR = 0x41   # right board — A0 jumper set
PCA_LEFT_ADDR  = 0x40   # left board  — factory default, no jumpers

# Per-leg configuration. All lists must stay the same length.
# To add legs: append one entry to each list simultaneously.
LEG_INDICES     = [0,         1,         2,         3,         4,         5        ]
LEG_CHANNELS    = [(0, 1, 2), (3, 4, 5), (6, 7, 8), (0, 1, 2), (3, 4, 5), (6, 7, 8)]
LEG_PHASES      = [0.0,       0.5,       0.0,       0.5,       0.0,       0.5      ]
LEG_BOARD_INDEX = [0,         0,         0,         1,         1,         1        ]  # 0=pca_right, 1=pca_left
LEG_INVERT      = [False,     False,     False,     True,      True,      True     ]  # mirror-mounted left servos

# Per-leg calibration trims (j1_deg, j2_deg, j3_deg).
# After calibrating a leg via --calibrate, paste its output tuple here.
TRIMS = [
    (60.0, 90.0, 60.0),  # leg 0 — calibrated
    ( 0.0,  0.0,  0.0),  # leg 1 — to be calibrated
    ( 0.0,  0.0,  0.0),  # leg 2 — to be calibrated
    ( 0.0,  0.0,  0.0),  # leg 3 — to be calibrated
    ( 0.0,  0.0,  0.0),  # leg 4 — to be calibrated
    ( 0.0,  0.0,  0.0),  # leg 5 — to be calibrated
]

# --- GEOMETRY (matches Scripts/iksolver.py and arduino/hexapod/config.h) ---
FEMUR_LEN = 3.5     # cm
TIBIA_LEN = 3.5     # cm
X_OFFSET  = -5.0
Y_OFFSET  = -2.0
Z_OFFSET  = -6.0

# Hip pivot position in body frame (derived from offsets; used to centre gait targets)
HIP_X = -X_OFFSET   # 5.0 cm — hip x from body centre
HIP_Y = -Y_OFFSET   # 2.0 cm — hip y from body centre

# --- GAIT PARAMETERS (matches Scripts/gait_controller.py) ---
STEP_HEIGHT = 2.0   # cm — swing lift height
STEP_LENGTH = 4.0   # cm — stride length
DUTY_FACTOR = 0.5   # stance/swing ratio
NUM_SAMPLES = 50    # trajectory points per cycle

# --- STAND/SIT TRANSITION ---
SIT_Z              = 5.5   # foot-z for "body on floor" rest pose (L=0.5, clear of singularity)
STAND_Z            = 0.0   # foot-z for nominal standing height (matches gait stance)
TRANSITION_STEPS   = 100   # interpolation samples per transition
TRANSITION_DURATION = 2.0  # seconds for stand-up or sit-down

# --- SERVO HARDWARE (matches arduino/hexapod/config.h) ---
SERVO_MIN_US  = 500.0
SERVO_MAX_US  = 2500.0
SERVO_MIN_DEG = -90.0
SERVO_MAX_DEG =  90.0

# Hardware imports — skipped on dev machine, used on Pi
try:
    import board
    import busio
    import adafruit_pca9685
    _HARDWARE_AVAILABLE = True
except ImportError:
    _HARDWARE_AVAILABLE = False


class LegIK:
    """Pure IK math — no hardware dependency. Port of Scripts/iksolver.py."""

    def __init__(self, leg_index: int = 0):
        self.leg_index = leg_index
        self._is_left = leg_index >= 3

    def apply_offsets(self, x: float, y: float, z: float):
        # Left-side legs mirror the x/y offsets (iksolver.py:52-53)
        sign = -1 if self._is_left else 1
        return x + sign * X_OFFSET, y + sign * Y_OFFSET, z + Z_OFFSET

    def solve_ik(self, x: float, y: float, z: float):
        """Return (j1, j2, j3) in radians for foot target (x, y, z) cm."""
        x, y, z = self.apply_offsets(x, y, z)

        j1 = math.atan2(y, x)
        H = math.sqrt(x**2 + y**2)
        L = math.sqrt(H**2 + z**2)
        cos_j3 = (FEMUR_LEN**2 + TIBIA_LEN**2 - L**2) / (2 * FEMUR_LEN * TIBIA_LEN)
        j3 = math.acos(max(-1.0, min(1.0, cos_j3)))
        cos_b = (L**2 + FEMUR_LEN**2 - TIBIA_LEN**2) / (2 * L * FEMUR_LEN)
        b = math.acos(max(-1.0, min(1.0, cos_b)))
        a = math.atan2(z, H)
        j2 = b + a

        return float(j1), float(j2), float(j3)


class GaitCycle:
    """Generates x/z foot trajectory arrays with optional phase offset."""

    def __init__(self, phase_offset: float = 0.0):
        self._phase_offset = phase_offset  # fraction 0.0-1.0; 0.5 = 180°

    def generate(self):
        """Return (x_arr, z_arr) numpy arrays of length NUM_SAMPLES, phase-shifted."""
        n = NUM_SAMPLES
        mid = int(n * DUTY_FACTOR)

        # Z: stance flat, swing sine arc (port of gait_controller.py:87-102)
        z_stance = np.zeros(mid)
        z_swing = STEP_HEIGHT * np.sin(np.linspace(0, math.pi, n - mid))
        z_arr = np.concatenate([z_stance, z_swing])

        # X: linear backward stance, linear forward swing (port of gait_controller.py:115-129)
        t = np.linspace(0, 1.0, n)
        stance_t = t[:mid]
        swing_t = t[mid:]
        x_stance = -STEP_LENGTH * (stance_t / max(stance_t)) if max(stance_t) > 0 else np.zeros(mid)
        x_swing = STEP_LENGTH * (swing_t / max(swing_t)) if max(swing_t) > 0 else np.zeros(n - mid)
        x_arr = np.concatenate([x_stance, x_swing])

        offset = int(n * self._phase_offset)
        return np.roll(x_arr, offset), np.roll(z_arr, offset)


class ServoDriver:
    """PCA9685 servo interface. Pass a real PCA9685 on Pi or a mock for tests."""

    def __init__(self, pca, channels=(0, 1, 2),
                 trim_j1: float = 0.0, trim_j2: float = 0.0, trim_j3: float = 0.0,
                 invert: bool = False):
        self.pca = pca
        self._channels = channels
        self._trims = (trim_j1, trim_j2, trim_j3)
        self._invert = invert

    def angle_to_duty_cycle(self, angle_rad: float, trim_deg: float = 0.0) -> int:
        """Convert radians to 16-bit PCA9685 duty cycle integer."""
        if self._invert:
            angle_rad = -angle_rad
        deg = math.degrees(angle_rad) + trim_deg
        deg = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, deg))
        pulse_us = (
            (deg - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG)
            * (SERVO_MAX_US - SERVO_MIN_US)
            + SERVO_MIN_US
        )
        return int(pulse_us / 20000.0 * 65535)

    def set_angles(self, j1: float, j2: float, j3: float) -> None:
        ch1, ch2, ch3 = self._channels
        self.pca.channels[ch1].duty_cycle = self.angle_to_duty_cycle(j1, self._trims[0])
        self.pca.channels[ch2].duty_cycle = self.angle_to_duty_cycle(j2, self._trims[1])
        self.pca.channels[ch3].duty_cycle = self.angle_to_duty_cycle(j3, self._trims[2])


def _calibrate(pca_right, pca_left):
    """Interactive per-leg calibration. Numbered keys select leg; q/w a/s z/x trim joints."""
    import tty
    import termios

    def _getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    pcas = [pca_right, pca_left]
    ik = LegIK(leg_index=0)

    # Leg selection menu
    print("\n--- CALIBRATION MODE ---")
    print("Select leg to calibrate:")
    for i, (idx, ch) in enumerate(zip(LEG_INDICES, LEG_CHANNELS)):
        board_addr = PCA_RIGHT_ADDR if LEG_BOARD_INDEX[i] == 0 else PCA_LEFT_ADDR
        print(f"  {i + 1} — Leg {i + 1}  index={idx}  channels {ch[0]}/{ch[1]}/{ch[2]}  board 0x{board_addr:02X}")
    print()

    leg_sel = None
    while leg_sel is None:
        ch = _getch()
        n = ord(ch) - ord('1')
        if 0 <= n < len(LEG_CHANNELS):
            leg_sel = n

    trims = list(TRIMS[leg_sel])

    trim_keys = {
        'q': (0, -1), 'w': (0, 1),
        'a': (1, -1), 's': (1, 1),
        'z': (2, -1), 'x': (2, 1),
    }
    leg_switch = {str(i + 1): i for i in range(len(LEG_CHANNELS))}

    print(f"\nCalibrating Leg {leg_sel + 1} (channels {LEG_CHANNELS[leg_sel]}).")
    print("q/w: J1(coxa) -/+1°   a/s: J2(femur) -/+1°   z/x: J3(tibia) -/+1°")
    print(f"1-{len(LEG_CHANNELS)}: switch leg   Enter: save and exit   Esc: cancel\n")

    while True:
        driver_now = ServoDriver(
            pcas[LEG_BOARD_INDEX[leg_sel]],
            channels=LEG_CHANNELS[leg_sel],
            trim_j1=trims[0], trim_j2=trims[1], trim_j3=trims[2],
            invert=LEG_INVERT[leg_sel],
        )
        j1, j2, j3 = ik.solve_ik(HIP_X, HIP_Y, 2.0)
        driver_now.set_angles(j1, j2, j3)
        print(f"\rLeg {leg_sel + 1} | J1={trims[0]:+.1f}°  J2={trims[1]:+.1f}°  J3={trims[2]:+.1f}°  (press key)",
              end='', flush=True)

        ch = _getch()

        if ch == '\r':
            print(f"\n\nSaved. Copy into TRIMS[{leg_sel}]:")
            print(f"    ({trims[0]}, {trims[1]}, {trims[2]}),")
            break
        elif ch == '\x1b':
            print("\nCalibration cancelled.")
            break
        elif ch in trim_keys:
            joint, delta = trim_keys[ch]
            trims[joint] += delta
        elif ch in leg_switch:
            leg_sel = leg_switch[ch]
            trims = list(TRIMS[leg_sel])
            print(f"\n\nSwitched to Leg {leg_sel + 1} (channels {LEG_CHANNELS[leg_sel]}).")


def _transition(drivers, iks, z_from, z_to, duration, steps=TRANSITION_STEPS, sleep=time.sleep):
    """Move all legs in unison from z_from to z_to over `duration` seconds.
    sleep is injectable — pass (lambda _t: None) in tests to skip real delays."""
    if steps < 1:
        raise ValueError("steps must be >= 1")
    step_delay = duration / steps
    for s in range(steps + 1):
        frac = s / steps
        z = z_from + (z_to - z_from) * frac
        for ik, driver in zip(iks, drivers):
            j1, j2, j3 = ik.solve_ik(HIP_X, HIP_Y, z)
            driver.set_angles(j1, j2, j3)
        sleep(step_delay)


def stand_up(drivers, iks, duration=TRANSITION_DURATION,
             steps=TRANSITION_STEPS, sleep=time.sleep):
    """Raise body from SIT_Z (floor) to STAND_Z (nominal standing height)."""
    _transition(drivers, iks, SIT_Z, STAND_Z, duration, steps, sleep)


def sit_down(drivers, iks, duration=TRANSITION_DURATION,
             steps=TRANSITION_STEPS, sleep=time.sleep):
    """Lower body from STAND_Z back to SIT_Z (floor)."""
    _transition(drivers, iks, STAND_Z, SIT_Z, duration, steps, sleep)


def _run(pca_right, pca_left):
    """Main gait loop — runs until Ctrl+C."""
    pcas = [pca_right, pca_left]
    trajectories = [GaitCycle(phase_offset=p).generate() for p in LEG_PHASES]
    drivers = [
        ServoDriver(pcas[LEG_BOARD_INDEX[i]], channels=LEG_CHANNELS[i],
                    trim_j1=TRIMS[i][0], trim_j2=TRIMS[i][1], trim_j3=TRIMS[i][2],
                    invert=LEG_INVERT[i])
        for i in range(len(LEG_CHANNELS))
    ]
    iks = [LegIK(leg_index=idx) for idx in LEG_INDICES]
    step_delay = 1.0 / (NUM_SAMPLES * CYCLE_FREQ)

    print("Standing up...")
    stand_up(drivers, iks)

    print(f"Running {len(LEG_CHANNELS)} legs at {CYCLE_FREQ} Hz. Ctrl+C to stop.")
    try:
        i = 0
        while True:
            for ik, driver, (x_arr, z_arr) in zip(iks, drivers, trajectories):
                j1, j2, j3 = ik.solve_ik(float(x_arr[i]) + HIP_X, HIP_Y, float(z_arr[i]))
                driver.set_angles(j1, j2, j3)
            time.sleep(step_delay)
            i = (i + 1) % NUM_SAMPLES
    except KeyboardInterrupt:
        print("\nStopped. Sitting down...")
    finally:
        sit_down(drivers, iks)


if __name__ == '__main__':
    if not _HARDWARE_AVAILABLE:
        print("Hardware libraries not found. Install: pip install adafruit-circuitpython-pca9685 adafruit-blinka")
        sys.exit(1)

    parser = argparse.ArgumentParser(description='Hexapod leg controller')
    parser.add_argument('--calibrate', action='store_true', help='Run calibration mode')
    args = parser.parse_args()

    i2c = busio.I2C(board.SCL, board.SDA)
    pca_right = adafruit_pca9685.PCA9685(i2c, address=PCA_RIGHT_ADDR)
    pca_right.frequency = 50
    pca_left = adafruit_pca9685.PCA9685(i2c, address=PCA_LEFT_ADDR)
    pca_left.frequency = 50

    try:
        if args.calibrate:
            _calibrate(pca_right, pca_left)
        else:
            _run(pca_right, pca_left)
    finally:
        pca_right.deinit()
        pca_left.deinit()
