import unittest
from Scripts.iksolver import IkSolver

FEMUR_LENGTH = 3.5
TIBIA_LENGTH = 3.5

class IkSolverTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """
        Single IkSolver shared across all tests to avoid hitting the 6-instance class limit.
        """
        cls.ik_solver = IkSolver(femur_length=FEMUR_LENGTH, tibia_length=TIBIA_LENGTH)

    @classmethod
    def tearDownClass(cls):
        del cls.ik_solver

    def test_01_initialisation(self):
        self.assertTrue(True)

    def test_02_offsets_property_get(self):
        """B4: the 'offsets' property must be readable and return [x, y, z]."""
        offsets = self.ik_solver.offsets
        self.assertIsInstance(offsets, list)
        self.assertEqual(len(offsets), 3)

    def test_03_solve_inverse_kinematics_returns_three_angles(self):
        """IK solver must return exactly three joint angles as floats."""
        result = self.ik_solver.solve_inverse_kinematics(1.0, 1.2, 3.5)
        self.assertEqual(len(result), 3)
        for angle in result:
            self.assertIsInstance(angle, float)

    def test_04_forward_kinematics_returns_three_joint_positions(self):
        """FK must return three [x, y, z] joint positions."""
        angles = self.ik_solver.solve_inverse_kinematics(1.0, 1.2, 3.5)
        coords = self.ik_solver.solve_forward_kinematics(*angles)
        self.assertEqual(len(coords), 3)
        for joint in coords:
            self.assertEqual(len(joint), 3)
