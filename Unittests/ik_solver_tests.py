import unittest
from Scripts.iksolver import IkSolver

FEMUR_LENGTH = 3.5
TIBIA_LENGTH = 3.5

class IkSolverTests(unittest.TestCase):
    def setUp(self):
        """
        Set Up for the IK solver class.
        """
        self.ik_solver = IkSolver(femur_length=FEMUR_LENGTH, tibia_length=TIBIA_LENGTH)

    def test_ik_solver(self):
        # Your test code here
        self.assertTrue(True)
