import unittest
import math
import numpy as np
from unittest.mock import MagicMock

from raspberry_pi.leg_controller import (
    LegIK, GaitCycle, ServoDriver,
    NUM_SAMPLES, STEP_LENGTH, STEP_HEIGHT,
    X_OFFSET, Y_OFFSET, Z_OFFSET,
    FEMUR_LEN, TIBIA_LEN,
    LEG_INDICES, LEG_CHANNELS, LEG_PHASES, LEG_BOARD_INDEX, LEG_INVERT,
    TRIMS, HIP_X, HIP_Y,
    stand_up, sit_down, SIT_Z, STAND_Z, TRANSITION_STEPS, TRANSITION_DURATION,
)


class RecordingIK:
    """Wraps a real LegIK; records every (x, y, z) passed to solve_ik."""
    def __init__(self, leg_index):
        self._ik = LegIK(leg_index=leg_index)
        self.calls = []

    def solve_ik(self, x, y, z):
        self.calls.append((x, y, z))
        return self._ik.solve_ik(x, y, z)


class TestLegIK(unittest.TestCase):

    def test_solve_ik_returns_three_floats(self):
        ik = LegIK(leg_index=0)
        result = ik.solve_ik(1.0, 1.0, 1.0)
        self.assertEqual(len(result), 3)
        for val in result:
            self.assertIsInstance(val, float)

    def test_j1_is_atan2_after_offsets(self):
        ik = LegIK(leg_index=0)
        x, y, z = 1.0, 1.0, 1.0
        j1, _, _ = ik.solve_ik(x, y, z)
        expected_j1 = math.atan2(y + Y_OFFSET, x + X_OFFSET)
        self.assertAlmostEqual(j1, expected_j1, places=10)

    def test_left_leg_negates_xy_offsets(self):
        # iksolver.py:52-53: left-side legs negate x_offset and y_offset
        right_ik = LegIK(leg_index=0)
        left_ik = LegIK(leg_index=3)
        x, y, z = 0.0, 0.0, 0.0
        x_right, y_right, _ = right_ik.apply_offsets(x, y, z)
        x_left, y_left, _ = left_ik.apply_offsets(x, y, z)
        self.assertAlmostEqual(x_right, X_OFFSET, places=10)   # -5
        self.assertAlmostEqual(x_left, -X_OFFSET, places=10)   # +5
        self.assertAlmostEqual(y_right, Y_OFFSET, places=10)   # -2
        self.assertAlmostEqual(y_left, -Y_OFFSET, places=10)   # +2

    def test_solve_ik_known_value(self):
        ik = LegIK(leg_index=0)
        x_in, y_in, z_in = 1.0, 1.0, 1.0
        j1, j2, j3 = ik.solve_ik(x_in, y_in, z_in)

        # Compute expected values using same math as iksolver.py:163-186
        x = x_in + X_OFFSET
        y = y_in + Y_OFFSET
        z = z_in + Z_OFFSET
        expected_j1 = math.atan2(y, x)
        H = math.sqrt(x**2 + y**2)
        L = math.sqrt(H**2 + z**2)
        expected_j3 = math.acos(
            (FEMUR_LEN**2 + TIBIA_LEN**2 - L**2) / (2 * FEMUR_LEN * TIBIA_LEN)
        )
        b = math.acos(
            (L**2 + FEMUR_LEN**2 - TIBIA_LEN**2) / (2 * L * FEMUR_LEN)
        )
        a = math.atan2(z, H)
        expected_j2 = b + a

        self.assertAlmostEqual(j1, expected_j1, places=10)
        self.assertAlmostEqual(j2, expected_j2, places=10)
        self.assertAlmostEqual(j3, expected_j3, places=10)


class TestGaitCycle(unittest.TestCase):

    def test_generate_returns_correct_length(self):
        gait = GaitCycle()
        x_arr, z_arr = gait.generate()
        self.assertEqual(len(x_arr), NUM_SAMPLES)
        self.assertEqual(len(z_arr), NUM_SAMPLES)

    def test_z_non_negative(self):
        gait = GaitCycle()
        _, z_arr = gait.generate()
        self.assertTrue(np.all(z_arr >= 0.0), "All z values must be >= 0")

    def test_z_max_equals_step_height(self):
        gait = GaitCycle()
        _, z_arr = gait.generate()
        self.assertAlmostEqual(float(z_arr.max()), STEP_HEIGHT, places=5)

    def test_x_range_within_step_length(self):
        gait = GaitCycle()
        x_arr, _ = gait.generate()
        self.assertGreaterEqual(float(x_arr.min()), -STEP_LENGTH)
        self.assertLessEqual(float(x_arr.max()), STEP_LENGTH)

    def test_phase_zero_matches_default(self):
        """GaitCycle(phase_offset=0.0) produces same output as no-arg GaitCycle."""
        base = GaitCycle()
        explicit = GaitCycle(phase_offset=0.0)
        x0, z0 = base.generate()
        x1, z1 = explicit.generate()
        np.testing.assert_array_equal(x0, x1)
        np.testing.assert_array_equal(z0, z1)

    def test_phase_offset_shifts_trajectory(self):
        """phase_offset=0.5 shifts both arrays by NUM_SAMPLES//2 = 25 samples."""
        base = GaitCycle(phase_offset=0.0)
        shifted = GaitCycle(phase_offset=0.5)
        x0, z0 = base.generate()
        x1, z1 = shifted.generate()
        np.testing.assert_array_equal(x1, np.roll(x0, NUM_SAMPLES // 2))
        np.testing.assert_array_equal(z1, np.roll(z0, NUM_SAMPLES // 2))


class TestServoDriver(unittest.TestCase):

    def _make_driver(self, **kwargs):
        return ServoDriver(MagicMock(), **kwargs)

    def test_neutral_angle_gives_mid_pulse(self):
        driver = self._make_driver()
        dc = driver.angle_to_duty_cycle(0.0)
        expected = int(1500 / 20000 * 65535)
        self.assertEqual(dc, expected)

    def test_max_angle_gives_max_pulse(self):
        driver = self._make_driver()
        dc = driver.angle_to_duty_cycle(math.pi / 2)
        expected = int(2500 / 20000 * 65535)
        self.assertEqual(dc, expected)

    def test_min_angle_gives_min_pulse(self):
        driver = self._make_driver()
        dc = driver.angle_to_duty_cycle(-math.pi / 2)
        expected = int(500 / 20000 * 65535)
        self.assertEqual(dc, expected)

    def test_set_angles_writes_three_channels(self):
        mock_pca = MagicMock()
        driver = ServoDriver(mock_pca)
        driver.set_angles(0.0, 0.0, 0.0)
        self.assertEqual(mock_pca.channels.__getitem__.call_count, 3)

    def test_trim_shifts_duty_cycle(self):
        driver_no_trim = self._make_driver()
        driver_trimmed = self._make_driver(trim_j1=10.0)
        dc_base = driver_no_trim.angle_to_duty_cycle(0.0)
        dc_trim = driver_trimmed.angle_to_duty_cycle(0.0, trim_deg=10.0)
        self.assertGreater(dc_trim, dc_base)

    def test_angle_clamped_to_servo_range(self):
        driver = self._make_driver()
        dc_over = driver.angle_to_duty_cycle(math.pi)  # > +90°
        dc_max = driver.angle_to_duty_cycle(math.pi / 2)
        self.assertEqual(dc_over, dc_max)

    def test_custom_channels_used_in_set_angles(self):
        """ServoDriver with channels=(3,4,5) writes to indices 3, 4, 5 not 0, 1, 2."""
        mock_pca = MagicMock()
        driver = ServoDriver(mock_pca, channels=(3, 4, 5))
        driver.set_angles(0.0, 0.0, 0.0)
        called_indices = [c.args[0] for c in mock_pca.channels.__getitem__.call_args_list]
        self.assertEqual(called_indices, [3, 4, 5])

    def test_invert_negates_angle(self):
        driver_normal = self._make_driver()
        driver_inv = self._make_driver(invert=True)
        dc_neg = driver_normal.angle_to_duty_cycle(-math.pi / 6)
        dc_inv = driver_inv.angle_to_duty_cycle(math.pi / 6)
        self.assertEqual(dc_inv, dc_neg)

    def test_invert_false_unchanged(self):
        driver = self._make_driver(invert=False)
        driver2 = self._make_driver()
        dc_explicit = driver.angle_to_duty_cycle(math.pi / 4)
        dc_default = driver2.angle_to_duty_cycle(math.pi / 4)
        self.assertEqual(dc_explicit, dc_default)


class TestMultiLegRun(unittest.TestCase):

    def test_legs_0_2_have_identical_trajectory(self):
        """Legs with phase 0.0 produce identical x/z arrays."""
        g0 = GaitCycle(phase_offset=LEG_PHASES[0])
        g2 = GaitCycle(phase_offset=LEG_PHASES[2])
        x0, z0 = g0.generate()
        x2, z2 = g2.generate()
        np.testing.assert_array_equal(x0, x2)
        np.testing.assert_array_equal(z0, z2)

    def test_leg_1_is_half_cycle_offset_from_leg_0(self):
        """Leg 1 (phase 0.5) == np.roll(leg 0 trajectory, NUM_SAMPLES//2)."""
        g0 = GaitCycle(phase_offset=0.0)
        g1 = GaitCycle(phase_offset=0.5)
        x0, z0 = g0.generate()
        x1, z1 = g1.generate()
        np.testing.assert_array_equal(x1, np.roll(x0, NUM_SAMPLES // 2))
        np.testing.assert_array_equal(z1, np.roll(z0, NUM_SAMPLES // 2))

    def test_all_three_legs_updated_per_step(self):
        """Right-side step writes to 9 channels (3 legs × 3 joints) on same pca."""
        mock_pca = MagicMock()
        ik = LegIK(leg_index=0)
        right_channels = [LEG_CHANNELS[i] for i in range(3)]
        right_trims    = [TRIMS[i] for i in range(3)]
        right_phases   = [LEG_PHASES[i] for i in range(3)]
        trajectories = [GaitCycle(phase_offset=p).generate() for p in right_phases]
        drivers = [
            ServoDriver(mock_pca, channels=ch, trim_j1=t[0], trim_j2=t[1], trim_j3=t[2])
            for ch, t in zip(right_channels, right_trims)
        ]
        for driver, (x_arr, z_arr) in zip(drivers, trajectories):
            j1, j2, j3 = ik.solve_ik(float(x_arr[0]) + HIP_X, HIP_Y, float(z_arr[0]))
            driver.set_angles(j1, j2, j3)
        self.assertEqual(mock_pca.channels.__getitem__.call_count, 9)

    def test_leg_channels_non_overlapping_per_board(self):
        """Each board's channel assignments must not overlap within that board."""
        for board_id in set(LEG_BOARD_INDEX):
            board_channels = [
                ch
                for i, group in enumerate(LEG_CHANNELS)
                if LEG_BOARD_INDEX[i] == board_id
                for ch in group
            ]
            self.assertEqual(len(board_channels), len(set(board_channels)),
                             f"Channel overlap on board {board_id}")


class TestConfigExtension(unittest.TestCase):

    def test_all_config_lists_same_length(self):
        n = len(LEG_INDICES)
        self.assertEqual(len(LEG_CHANNELS), n)
        self.assertEqual(len(LEG_PHASES), n)
        self.assertEqual(len(TRIMS), n)
        self.assertEqual(len(LEG_BOARD_INDEX), n)
        self.assertEqual(len(LEG_INVERT), n)

    def test_right_legs_use_board_0(self):
        self.assertEqual(LEG_BOARD_INDEX[:3], [0, 0, 0])

    def test_left_legs_use_board_1(self):
        self.assertEqual(LEG_BOARD_INDEX[3:], [1, 1, 1])

    def test_left_legs_have_legik_index_3_plus(self):
        for i, board in enumerate(LEG_BOARD_INDEX):
            if board == 1:
                self.assertGreaterEqual(LEG_INDICES[i], 3,
                                        f"Leg {i} on left board but LEG_INDICES[{i}]={LEG_INDICES[i]} < 3")

    def test_left_legs_invert_true(self):
        self.assertTrue(all(LEG_INVERT[i] for i in range(3, 6)))

    def test_right_legs_invert_false(self):
        self.assertTrue(all(not LEG_INVERT[i] for i in range(3)))


class TestTripodPhaseConfig(unittest.TestCase):

    def test_six_leg_two_phase_groups(self):
        self.assertEqual(len(set(LEG_PHASES)), 2)

    def test_groups_half_cycle_apart(self):
        self.assertAlmostEqual(abs(max(LEG_PHASES) - min(LEG_PHASES)), 0.5, places=10)

    def test_equal_group_sizes(self):
        expected = len(LEG_PHASES) // 2
        for phase in set(LEG_PHASES):
            self.assertEqual(LEG_PHASES.count(phase), expected)

    def test_legs_0_2_4_share_phase(self):
        self.assertEqual(LEG_PHASES[0], LEG_PHASES[2])
        self.assertEqual(LEG_PHASES[2], LEG_PHASES[4])

    def test_legs_1_3_5_share_phase(self):
        self.assertEqual(LEG_PHASES[1], LEG_PHASES[3])
        self.assertEqual(LEG_PHASES[3], LEG_PHASES[5])


class TestSixLegRun(unittest.TestCase):

    def _make_drivers(self, mock_right, mock_left):
        pcas = [mock_right, mock_left]
        return [
            ServoDriver(pcas[LEG_BOARD_INDEX[i]], channels=LEG_CHANNELS[i],
                        trim_j1=TRIMS[i][0], trim_j2=TRIMS[i][1], trim_j3=TRIMS[i][2],
                        invert=LEG_INVERT[i])
            for i in range(len(LEG_CHANNELS))
        ]

    def test_all_six_legs_updated_per_step(self):
        mock_right, mock_left = MagicMock(), MagicMock()
        trajectories = [GaitCycle(phase_offset=p).generate() for p in LEG_PHASES]
        drivers = self._make_drivers(mock_right, mock_left)
        iks = [LegIK(leg_index=idx) for idx in LEG_INDICES]
        for ik, driver, (x_arr, z_arr) in zip(iks, drivers, trajectories):
            j1, j2, j3 = ik.solve_ik(float(x_arr[0]) + HIP_X, HIP_Y, float(z_arr[0]))
            driver.set_angles(j1, j2, j3)
        self.assertEqual(mock_right.channels.__getitem__.call_count, 9)
        self.assertEqual(mock_left.channels.__getitem__.call_count, 9)

    def test_right_board_only_receives_right_legs(self):
        mock_right, mock_left = MagicMock(), MagicMock()
        trajectories = [GaitCycle(phase_offset=p).generate() for p in LEG_PHASES]
        drivers = self._make_drivers(mock_right, mock_left)
        iks = [LegIK(leg_index=idx) for idx in LEG_INDICES]
        for ik, driver, (x_arr, z_arr) in zip(iks[:3], drivers[:3], trajectories[:3]):
            j1, j2, j3 = ik.solve_ik(float(x_arr[0]) + HIP_X, HIP_Y, float(z_arr[0]))
            driver.set_angles(j1, j2, j3)
        self.assertEqual(mock_right.channels.__getitem__.call_count, 9)
        self.assertEqual(mock_left.channels.__getitem__.call_count, 0)

    def test_left_legs_use_legik_left_side(self):
        for i, board in enumerate(LEG_BOARD_INDEX):
            if board == 1:
                ik = LegIK(leg_index=LEG_INDICES[i])
                self.assertTrue(ik._is_left,
                                f"Leg {i} (LEG_INDICES[{i}]={LEG_INDICES[i]}) should be left-side")


class TestSitStandConstants(unittest.TestCase):

    def test_sit_z_geometrically_valid(self):
        for leg_index in (0, 3):
            ik = LegIK(leg_index=leg_index)
            result = ik.solve_ik(HIP_X, HIP_Y, SIT_Z)
            self.assertEqual(len(result), 3)
            for val in result:
                self.assertTrue(math.isfinite(val), f"Non-finite value {val} for leg_index={leg_index}")

    def test_sit_z_within_reach(self):
        L = abs(SIT_Z + Z_OFFSET)
        self.assertGreater(L, 0)
        self.assertLess(L, FEMUR_LEN + TIBIA_LEN)

    def test_stand_z_is_zero(self):
        self.assertEqual(STAND_Z, 0.0)

    def test_transition_steps_positive(self):
        self.assertGreaterEqual(TRANSITION_STEPS, 1)


class TestStandUp(unittest.TestCase):

    def setUp(self):
        self.mock_drivers = [MagicMock() for _ in range(6)]
        self.recording_iks = [RecordingIK(leg_index=i) for i in range(6)]
        self._no_sleep = lambda _t: None

    def _run_stand_up(self):
        stand_up(self.mock_drivers, self.recording_iks,
                 duration=TRANSITION_DURATION, steps=TRANSITION_STEPS,
                 sleep=self._no_sleep)

    def test_set_angles_count(self):
        self._run_stand_up()
        for driver in self.mock_drivers:
            self.assertEqual(driver.set_angles.call_count, TRANSITION_STEPS + 1)

    def test_first_z_is_sit_z(self):
        self._run_stand_up()
        for rik in self.recording_iks:
            self.assertAlmostEqual(rik.calls[0][2], SIT_Z, places=10)

    def test_last_z_is_stand_z(self):
        self._run_stand_up()
        for rik in self.recording_iks:
            self.assertAlmostEqual(rik.calls[-1][2], STAND_Z, places=10)

    def test_z_monotonically_decreases(self):
        self._run_stand_up()
        z_seq = [c[2] for c in self.recording_iks[0].calls]
        for a, b in zip(z_seq, z_seq[1:]):
            self.assertGreaterEqual(a + 1e-12, b, "z should be non-increasing during stand_up")

    def test_all_legs_identical_z_per_step(self):
        self._run_stand_up()
        for step in range(TRANSITION_STEPS + 1):
            z_values = [rik.calls[step][2] for rik in self.recording_iks]
            self.assertAlmostEqual(max(z_values) - min(z_values), 0.0, places=10)

    def test_uses_hip_x_hip_y(self):
        self._run_stand_up()
        for rik in self.recording_iks:
            for (x, y, _) in rik.calls:
                self.assertAlmostEqual(x, HIP_X, places=10)
                self.assertAlmostEqual(y, HIP_Y, places=10)

    def test_no_hardware_required(self):
        try:
            self._run_stand_up()
        except Exception as e:
            self.fail(f"stand_up raised unexpectedly: {e}")

    def test_sleep_called_per_step(self):
        mock_sleep = MagicMock()
        stand_up(self.mock_drivers, self.recording_iks,
                 duration=TRANSITION_DURATION, steps=TRANSITION_STEPS,
                 sleep=mock_sleep)
        self.assertEqual(mock_sleep.call_count, TRANSITION_STEPS + 1)


class TestSitDown(unittest.TestCase):

    def setUp(self):
        self.mock_drivers = [MagicMock() for _ in range(6)]
        self.recording_iks = [RecordingIK(leg_index=i) for i in range(6)]
        self._no_sleep = lambda _t: None

    def _run_sit_down(self):
        sit_down(self.mock_drivers, self.recording_iks,
                 duration=TRANSITION_DURATION, steps=TRANSITION_STEPS,
                 sleep=self._no_sleep)

    def test_set_angles_count(self):
        self._run_sit_down()
        for driver in self.mock_drivers:
            self.assertEqual(driver.set_angles.call_count, TRANSITION_STEPS + 1)

    def test_first_z_is_stand_z(self):
        self._run_sit_down()
        for rik in self.recording_iks:
            self.assertAlmostEqual(rik.calls[0][2], STAND_Z, places=10)

    def test_last_z_is_sit_z(self):
        self._run_sit_down()
        for rik in self.recording_iks:
            self.assertAlmostEqual(rik.calls[-1][2], SIT_Z, places=10)

    def test_z_monotonically_increases(self):
        self._run_sit_down()
        z_seq = [c[2] for c in self.recording_iks[0].calls]
        for a, b in zip(z_seq, z_seq[1:]):
            self.assertLessEqual(a - 1e-12, b, "z should be non-decreasing during sit_down")

    def test_all_legs_identical_z_per_step(self):
        self._run_sit_down()
        for step in range(TRANSITION_STEPS + 1):
            z_values = [rik.calls[step][2] for rik in self.recording_iks]
            self.assertAlmostEqual(max(z_values) - min(z_values), 0.0, places=10)

    def test_uses_hip_x_hip_y(self):
        self._run_sit_down()
        for rik in self.recording_iks:
            for (x, y, _) in rik.calls:
                self.assertAlmostEqual(x, HIP_X, places=10)
                self.assertAlmostEqual(y, HIP_Y, places=10)

    def test_no_hardware_required(self):
        try:
            self._run_sit_down()
        except Exception as e:
            self.fail(f"sit_down raised unexpectedly: {e}")

    def test_sit_down_is_reverse_of_stand_up(self):
        stand_iks = [RecordingIK(leg_index=i) for i in range(6)]
        sit_iks   = [RecordingIK(leg_index=i) for i in range(6)]
        mock_drivers = [MagicMock() for _ in range(6)]
        no_sleep = lambda _t: None
        stand_up(mock_drivers, stand_iks, duration=TRANSITION_DURATION,
                 steps=TRANSITION_STEPS, sleep=no_sleep)
        sit_down(mock_drivers, sit_iks, duration=TRANSITION_DURATION,
                 steps=TRANSITION_STEPS, sleep=no_sleep)
        stand_z = [c[2] for c in stand_iks[0].calls]
        sit_z   = [c[2] for c in sit_iks[0].calls]
        for a, b in zip(stand_z, reversed(sit_z)):
            self.assertAlmostEqual(a, b, places=10)


class TestSitStandIKValidity(unittest.TestCase):

    def test_no_domain_error_across_full_transition(self):
        for leg_index in range(6):
            ik = LegIK(leg_index=leg_index)
            for s in range(TRANSITION_STEPS + 1):
                frac = s / TRANSITION_STEPS
                z = SIT_Z + (STAND_Z - SIT_Z) * frac
                result = ik.solve_ik(HIP_X, HIP_Y, z)
                for val in result:
                    self.assertTrue(math.isfinite(val),
                                    f"Non-finite at leg {leg_index}, step {s}, z={z:.4f}: {val}")

    def test_left_legs_valid_across_transition(self):
        for leg_index in [3, 4, 5]:
            ik = LegIK(leg_index=leg_index)
            for s in range(TRANSITION_STEPS + 1):
                frac = s / TRANSITION_STEPS
                z = SIT_Z + (STAND_Z - SIT_Z) * frac
                result = ik.solve_ik(HIP_X, HIP_Y, z)
                for val in result:
                    self.assertTrue(math.isfinite(val),
                                    f"Non-finite at left leg {leg_index}, step {s}, z={z:.4f}: {val}")


if __name__ == '__main__':
    unittest.main()
