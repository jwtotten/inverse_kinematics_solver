import unittest
from Scripts.animation_controller import (
    advance_frame,
    compute_body_offset,
    clamp_speed,
    translate_joints,
    build_ground_grid,
    apply_direction_to_controllers,
    apply_gait_to_controllers,
)
from Scripts.gait_controller import GaitController, GaitPattern
from Scripts.iksolver import IkSolver


class AnimationControllerTests(unittest.TestCase):
    """Tests for pure animation logic functions."""

    # advance_frame tests
    def test_advance_frame_normal(self):
        result = advance_frame(10.0, 1.0, 50)
        self.assertAlmostEqual(result, 11.0)

    def test_advance_frame_wraps_at_num_samples(self):
        result = advance_frame(49.8, 1.0, 50)
        self.assertAlmostEqual(result, 0.8)

    def test_advance_frame_fractional_speed(self):
        result = advance_frame(10.0, 0.5, 50)
        self.assertAlmostEqual(result, 10.5)

    # compute_body_offset tests
    def test_body_offset_increases_forward(self):
        result = compute_body_offset(0.0, 1, 4.0, 50)
        self.assertGreater(result, 0.0)

    def test_body_offset_unchanged_when_stopped(self):
        result = compute_body_offset(5.0, 0, 4.0, 50)
        self.assertAlmostEqual(result, 5.0)

    def test_body_offset_decreases_backward(self):
        result = compute_body_offset(0.0, -1, 4.0, 50)
        self.assertLess(result, 0.0)

    # clamp_speed tests
    def test_clamp_speed_min(self):
        self.assertAlmostEqual(clamp_speed(0.0), 0.2)

    def test_clamp_speed_max(self):
        self.assertAlmostEqual(clamp_speed(5.0), 3.0)

    def test_clamp_speed_in_range(self):
        self.assertAlmostEqual(clamp_speed(1.5), 1.5)

    # translate_joints tests
    def test_translate_joints_shifts_x(self):
        joints = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        result = translate_joints(joints, 10.0)
        self.assertAlmostEqual(result[0][0], 11.0)
        self.assertAlmostEqual(result[1][0], 14.0)

    def test_translate_joints_preserves_y_z(self):
        joints = [[0.0, 2.0, 3.0]]
        result = translate_joints(joints, 5.0)
        self.assertAlmostEqual(result[0][1], 2.0)
        self.assertAlmostEqual(result[0][2], 3.0)

    # build_ground_grid tests
    def test_ground_grid_z_is_zero(self):
        xs, ys, zs = build_ground_grid(body_offset=0.0)
        flat_z = [z for seg in zs for z in seg if z is not None]
        self.assertTrue(all(z == 0.0 for z in flat_z))

    def test_ground_grid_centered_on_offset(self):
        xs, ys, zs = build_ground_grid(body_offset=10.0, spacing=1.0, half_width=5.0)
        flat_x = [x for seg in xs for x in seg if x is not None]
        self.assertGreaterEqual(min(flat_x), 5.0 - 0.001)
        self.assertLessEqual(max(flat_x), 15.0 + 0.001)


class AnimationControllerIntegrationTests(unittest.TestCase):
    """Integration tests using real GaitController/IkSolver instances."""

    @classmethod
    def setUpClass(cls):
        # Use pre-existing IkSolver instances if available, or create new ones
        # IkSolver has a 6-instance limit — these tests use 2 instances max
        cls.ik_a = IkSolver(femur_length=3.5, tibia_length=3.5)
        cls.ik_b = IkSolver(femur_length=3.5, tibia_length=3.5)

    # apply_direction_to_controllers tests
    def test_direction_forward_sets_positive_step_length(self):
        gc = GaitController(iksolver=self.ik_a, number_samples=50)
        apply_direction_to_controllers([gc], direction=1, step_length_abs=4.0)
        self.assertAlmostEqual(gc.step_length, 4.0)

    def test_direction_backward_sets_negative_step_length(self):
        gc = GaitController(iksolver=self.ik_a, number_samples=50)
        apply_direction_to_controllers([gc], direction=-1, step_length_abs=4.0)
        self.assertAlmostEqual(gc.step_length, -4.0)

    def test_direction_stop_sets_zero_step_length(self):
        gc = GaitController(iksolver=self.ik_a, number_samples=50)
        apply_direction_to_controllers([gc], direction=0, step_length_abs=4.0)
        self.assertAlmostEqual(gc.step_length, 0.0)

    # apply_gait_to_controllers tests
    def test_gait_switch_tripod_phase_offsets(self):
        gc = GaitController(iksolver=self.ik_a, number_samples=50)
        apply_gait_to_controllers([gc], GaitPattern.TRIPOD)
        # After applying TRIPOD, phase_offset depends on ik_a's instance number
        # Just verify it's one of the valid TRIPOD values (0.0 or 0.5)
        self.assertIn(gc.phase_offset, [0.0, 0.5])

    def test_gait_switch_wave_phase_offsets(self):
        gc = GaitController(iksolver=self.ik_a, number_samples=50)
        apply_gait_to_controllers([gc], GaitPattern.WAVE)
        # WAVE: phase_offset = (instance_number * 0.167) % 1
        # Just verify it's a valid float in [0, 1)
        self.assertGreaterEqual(gc.phase_offset, 0.0)
        self.assertLess(gc.phase_offset, 1.0)


if __name__ == "__main__":
    unittest.main()
