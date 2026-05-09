import unittest
import numpy as np
from Scripts.geometry_utils import (
    segment_to_segment_distance,
    segments_are_crossing,
    build_segment_list,
)
from Scripts.leg_collision_checker import (
    check_all_leg_collisions,
    get_crossing_leg_indices,
    CollisionResult,
)


class GeometryUtilsTests(unittest.TestCase):

    def test_parallel_segments_perpendicular_distance(self):
        p0, p1 = np.array([0, 0, 0.0]), np.array([1, 0, 0.0])
        q0, q1 = np.array([0, 1, 0.0]), np.array([1, 1, 0.0])
        self.assertAlmostEqual(segment_to_segment_distance(p0, p1, q0, q1), 1.0, places=6)

    def test_crossing_segments_zero_distance(self):
        p0, p1 = np.array([-1, 0, 0.0]), np.array([1, 0, 0.0])
        q0, q1 = np.array([0, -1, 0.0]), np.array([0, 1, 0.0])
        self.assertAlmostEqual(segment_to_segment_distance(p0, p1, q0, q1), 0.0, places=6)

    def test_skew_segments_non_zero_distance(self):
        # Segment A along X at z=0, segment B along Y at z=1
        p0, p1 = np.array([0, 0, 0.0]), np.array([2, 0, 0.0])
        q0, q1 = np.array([1, -1, 1.0]), np.array([1, 1, 1.0])
        self.assertAlmostEqual(segment_to_segment_distance(p0, p1, q0, q1), 1.0, places=6)

    def test_shared_endpoint_zero_distance(self):
        p0, p1 = np.array([0, 0, 0.0]), np.array([1, 0, 0.0])
        q0, q1 = np.array([1, 0, 0.0]), np.array([2, 1, 0.0])
        self.assertAlmostEqual(segment_to_segment_distance(p0, p1, q0, q1), 0.0, places=6)

    def test_degenerate_point_segment(self):
        # Segment A is a single point at origin
        p0 = p1 = np.array([0, 0, 0.0])
        q0, q1 = np.array([1, 0, 0.0]), np.array([2, 0, 0.0])
        self.assertAlmostEqual(segment_to_segment_distance(p0, p1, q0, q1), 1.0, places=6)

    def test_segments_are_crossing_true(self):
        p0, p1 = np.array([-1, 0, 0.0]), np.array([1, 0, 0.0])
        q0, q1 = np.array([0, -1, 0.0]), np.array([0, 1, 0.0])
        self.assertTrue(segments_are_crossing(p0, p1, q0, q1, threshold=0.01))

    def test_segments_are_crossing_false(self):
        p0, p1 = np.array([0, 0, 0.0]), np.array([1, 0, 0.0])
        q0, q1 = np.array([0, 5, 0.0]), np.array([1, 5, 0.0])
        self.assertFalse(segments_are_crossing(p0, p1, q0, q1, threshold=0.05))

    def test_build_segment_list_length(self):
        coords = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.5, 0.0]]
        segs = build_segment_list(coords)
        self.assertEqual(len(segs), 2)

    def test_build_segment_list_values(self):
        coords = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.5, 0.0]]
        segs = build_segment_list(coords)
        np.testing.assert_array_equal(segs[0][0], np.array([0.0, 0.0, 0.0]))
        np.testing.assert_array_equal(segs[0][1], np.array([1.0, 0.0, 0.0]))
        np.testing.assert_array_equal(segs[1][0], np.array([1.0, 0.0, 0.0]))
        np.testing.assert_array_equal(segs[1][1], np.array([2.0, 0.5, 0.0]))

    def test_build_segment_list_array_shape(self):
        coords = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.5, 0.0]]
        segs = build_segment_list(coords)
        self.assertEqual(segs[0][0].shape, (3,))
        self.assertEqual(segs[1][1].shape, (3,))


class LegCollisionCheckerTests(unittest.TestCase):

    def _make_straight_leg(self, origin_x, origin_y, reach_x, reach_y):
        """Synthetic leg: hip at origin, knee at midpoint, foot at end."""
        hip  = [origin_x,                      origin_y,                      0.0]
        knee = [origin_x + reach_x * 0.5,      origin_y + reach_y * 0.5,    -1.75]
        foot = [origin_x + reach_x,             origin_y + reach_y,           -3.5]
        return [hip, knee, foot]

    def _spread_legs(self):
        """Six legs spread far apart — guaranteed no crossing."""
        return [
            self._make_straight_leg(0,   0,  1, 0),
            self._make_straight_leg(0,   3,  1, 0),
            self._make_straight_leg(0,   6,  1, 0),
            self._make_straight_leg(-50, 0, -1, 0),
            self._make_straight_leg(-50, 3, -1, 0),
            self._make_straight_leg(-50, 6, -1, 0),
        ]

    def test_no_collision_separated_legs(self):
        crossing = get_crossing_leg_indices(self._spread_legs(), threshold=0.05)
        self.assertEqual(len(crossing), 0)

    def test_collision_detected_when_legs_co_located(self):
        # All 6 legs at exactly the same position — every pair crosses
        colliding_leg = self._make_straight_leg(0, 0, 3, 0)
        all_coords = [colliding_leg] * 6
        crossing = get_crossing_leg_indices(all_coords, threshold=0.05)
        self.assertEqual(set(crossing), {0, 1, 2, 3, 4, 5})

    def test_collision_result_fields(self):
        colliding_leg = self._make_straight_leg(0, 0, 3, 0)
        results = check_all_leg_collisions([colliding_leg] * 6, threshold=0.05)
        self.assertIsInstance(results[0], CollisionResult)
        self.assertIsInstance(results[0].leg_a_index, int)
        self.assertIsInstance(results[0].leg_b_index, int)
        self.assertIsInstance(results[0].distance, float)
        self.assertTrue(results[0].is_crossing)

    def test_only_overlapping_legs_flagged(self):
        overlap_leg = self._make_straight_leg(0, 0, 3, 0)
        all_coords = [
            overlap_leg,                           # leg 0  ┐ crossing
            overlap_leg,                           # leg 1  ┘
            self._make_straight_leg(0,   20, 1, 0),  # leg 2
            self._make_straight_leg(0,   30, 1, 0),  # leg 3
            self._make_straight_leg(-50, 20, -1, 0), # leg 4
            self._make_straight_leg(-50, 30, -1, 0), # leg 5
        ]
        crossing = get_crossing_leg_indices(all_coords, threshold=0.05)
        self.assertIn(0, crossing)
        self.assertIn(1, crossing)
        self.assertNotIn(2, crossing)
        self.assertNotIn(3, crossing)
        self.assertNotIn(4, crossing)
        self.assertNotIn(5, crossing)

    def test_same_leg_segments_not_compared(self):
        # A leg's own femur and tibia share the knee endpoint (distance = 0),
        # so if self-comparisons were included every leg would flag. Verify they don't.
        leg = self._make_straight_leg(0, 0, 3, 0)
        all_coords = [leg] + self._spread_legs()[1:]
        results = check_all_leg_collisions(all_coords, threshold=0.05)
        for r in results:
            self.assertNotEqual(r.leg_a_index, r.leg_b_index)


if __name__ == "__main__":
    unittest.main()
