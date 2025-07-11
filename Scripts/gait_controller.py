"""
This script is for controlling the gait of the hexapod and 
for configuring the directionality of the leg based off of its leg instance.
"""

import numpy as np

class GaitController:
    _instances = []
    limit = 6

    def __init__(self, iksolver, number_samples):
        self.iksolver = iksolver
        self.number_samples = number_samples

    def __new__(cls, *args, **kwargs):
        if not len(cls._instances) < cls.limit:
            raise RuntimeError("Count not create instance. Limit %s reached" % cls.limit)    
        instance = super().__new__(cls)
        cls._instances.append(instance)
        return instance

    def lift_leg_equation(self) -> list:
        """
        This function is used to calculate the sample positons for the leg when the leg is to 
        be lifted directly up for half of the gait cycle.
        :return: leg_up, leg_down :: two lists with the leg in the up position for half of the gait cycle 
        and the leg in the down position for half of the gait cycle.
        :rtype: list
        """
        leg_up = np.zeros((self.number_samples/2, self.iksolver.z1))
        leg_down = np.zeros((self.number_samples/2, self.iksolver.z0))

        return leg_up, leg_down
    
    def stationary_leg_equation(self) -> list:
        """
        This function is used to calculate the sample positions for the leg when the leg is to 
        be held at a constant position for the full gait cycle.
        :return: leg_stationary :: a list with the leg at a fixed position for the full gait cycle.
        :rtype: list
        """
        leg_stationary = np.zeros((self.number_samples, self.iksolver.y0))
        
        return leg_stationary
    
    def carry_leg_equation(self) -> list:
        """
        This function is used to calculate the sample positions for the leg when the leg is to
        be moved horizontally for half of the gait cycle.
        :return: leg_forward, leg_backwards :: two lists with the leg moving horizontally forward and horizontally backwards
        for half of the gait cycle each.
        :rtype: list
        """
        leg_forward = np.zeros((self.number_samples/2, self.iksolver.x1))
        leg_backward = np.zeros((self.number_samples/2, self.iksolver.x0))

        return leg_forward, leg_backward
    