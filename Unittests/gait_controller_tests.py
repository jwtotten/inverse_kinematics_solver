import unittest
from Scripts.gait_controller import GaitController
from Scripts.iksolver import IkSolver

FEMUR_LENGTH = 3.5
TIBIA_LENGTH = 3.5

class GaitControllerTests(unittest.TestCase):

    test_01 = False
    test_02 = False
    test_03 = False

    @classmethod
    def setUpClass(cls):
        """
        This class method is called at the instantiation of the class itself, meaning only one IkSolver class is created and used for repeat testing. If this was called in the setUp() method, this would be repeatedly reinstanciated.
        """
        cls.ik_solver = IkSolver(femur_length=FEMUR_LENGTH, tibia_length=TIBIA_LENGTH)

    def test_01_initialisation(self):
        """
        Test 01 covers the initialisation of the class with different parameters.
        """

        # This first case should fail, as no IkSolver class is provided.
        try:
            self.gait_controller = GaitController()
        except TypeError:
            assert True
        except Exception:
            assert False

        # This next case should also raise an error, as None as been passed instead of the IkSolver class.
        try:
            self.gait_controller = GaitController(iksolver=None)
        except TypeError:
            assert True
        except Exception:
            assert False

        # This next case should fail, as the correct IkSolver class has been passed but the number of instances is not of the correct type.
        try:
            self.gait_controller = GaitController(iksolver=self.ik_solver, num_instances="not_a_number")
        except TypeError:
            assert True
        except Exception:
            assert False

        # This next case should pass, as the correct IkSolver class has been given, and the number of instances to calculate has been left at default.
        try:
            self.gait_controller = GaitController(iksolver=self.ik_solver)
            assert True
        except Exception:
            assert False
        
        self.test_01 = True

    @unittest.skipIf(not test_01, "Initialisation Test was not passed.")
    def test_02_gait_cycle(self):
        """
        Test 02 covers the functionality of setting up the gait cycle.        
        """
        pass
