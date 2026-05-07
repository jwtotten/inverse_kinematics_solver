import unittest
import numpy as np
from Scripts.gait_controller import GaitController, GaitPattern
from Scripts.iksolver import IkSolver

FEMUR_LENGTH = 3.5
TIBIA_LENGTH = 3.5

class GaitControllerTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """
        Single IkSolver shared across all tests to avoid hitting the 6-instance class limit.
        """
        cls.ik_solver = IkSolver(femur_length=FEMUR_LENGTH, tibia_length=TIBIA_LENGTH)

    @classmethod
    def tearDownClass(cls):
        del cls.ik_solver

    # ------------------------------------------------------------------
    # test_01 — initialisation
    # ------------------------------------------------------------------

    def test_01_initialisation_no_args_raises(self):
        """GaitController() with no arguments must raise TypeError."""
        with self.assertRaises(TypeError):
            GaitController()

    def test_01_initialisation_none_raises(self):
        """GaitController(iksolver=None) must raise TypeError."""
        with self.assertRaises(TypeError):
            GaitController(iksolver=None)

    def test_01_initialisation_invalid_number_samples_raises(self):
        """B7: GaitController must reject a non-integer number_samples."""
        with self.assertRaises(TypeError):
            GaitController(iksolver=self.ik_solver, number_samples="not_a_number")

    def test_01_initialisation_valid(self):
        """GaitController with a valid IkSolver and default samples must construct cleanly."""
        gc = GaitController(iksolver=self.ik_solver)
        self.assertIsNotNone(gc)

    # ------------------------------------------------------------------
    # test_02 — gait cycle
    # ------------------------------------------------------------------

    def test_02_get_motion_returns_three_vectors(self):
        """get_motion() must return a list of three non-None numpy arrays."""
        gc = GaitController(iksolver=self.ik_solver)
        motion = gc.get_motion()
        self.assertEqual(len(motion), 3)
        for vec in motion:
            self.assertIsNotNone(vec)

    def test_02_gait_pattern_tripod(self):
        """Setting TRIPOD gait pattern must not raise and must produce motion vectors."""
        gc = GaitController(iksolver=self.ik_solver)
        gc.set_gait_pattern(GaitPattern.TRIPOD)
        x, y, z = gc.get_motion()
        self.assertEqual(len(x), gc.number_samples)
        self.assertEqual(len(z), gc.number_samples)

    # ------------------------------------------------------------------
    # test_03 — stationary leg equation
    # ------------------------------------------------------------------

    def test_03_stationary_leg_equation_shape(self):
        """B3: stationary_leg_equation() must return an array of length number_samples."""
        gc = GaitController(iksolver=self.ik_solver)
        result = gc.stationary_leg_equation()
        self.assertEqual(len(result), gc.number_samples)
        self.assertTrue(all(v == 0.0 for v in result))
