# This file is for solving the inverse kinematics of a hexapod robot.

import numpy as np
from math import atan2, sqrt, acos

class IkSolver:
    def __init__(self, coxa_length, femur_length, tibia_length, z_offset=0):
        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length
        self.z_offset = z_offset
    
    def solve_angles(self, x:float, y:float, z:float, verbose:bool = False) -> list:
        """
        Solves the angles required to move the leg to the given coordinate (x, y, z).
        :param x: X coordinate
        :type x: float
        :param y: Y coordinate
        :type y: float
        :param z: Z coordinate
        :type z: float
        :return: List of angles [alpha, beta, gamma]
        """
        # Calculate required lengths
        l1 = sqrt(x**2 + y**2)
        l = sqrt(self.z_offset**2 - (l1-self.coxa_length)**2)
        if verbose:
            print(f"l1: {l1}, l: {l}")

        # Calculate gamma
        gamma = atan2(y, x)
        if verbose:
            print(f"gamma: {gamma}")

        # Calculate alpha
        alpha1 = acos(self.z_offset/l)
        if verbose:
            print(f"alpha1: {alpha1}")
        alpha2 = acos((l**2 + self.femur_length**2 - self.tibia_length**2)/(2*l*self.tibia_length))
        if verbose:
            print(f"alpha2: {alpha2}")
        alpha  = alpha1 + alpha2
        if verbose:
            print(f"alpha: {alpha}")

        # Calculate beta
        beta = acos((self.femur_length**2 + self.tibia_length**2 - l**2)/(2*self.femur_length*self.tibia_length))
        if verbose:
            print(f"beta: {beta}")

        return [alpha, beta, gamma] 

if __name__ == "__main__":
    print('Inverse Kinematics Solver for Hexapod Robot')

    # Example parameters
    coxa_length: float = 1.0
    femur_length: float = 1.0
    tibia_length: float = 1.0
    z_offset: float = 2.0

    # Example usage
    x: float = 0.5
    y: float = 0.5
    z: float = 0.0

    ik_solver = IkSolver(coxa_length, femur_length, tibia_length, z_offset)
    angles = ik_solver.solve_angles(x, y, z, verbose=True)
    print(f"Angles: {angles}")
