import numpy as np
from math import atan2, sqrt, acos, sin, cos

class IkSolver:
    def __init__(self, coxa_length, femur_length, tibia_length, z_offset=0):
        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length
        self.z_offset = z_offset

    def solve_inverse_kinematics(self, x, y, z) -> list:
        """
        This function solves the inverse kinematic equations for an end effector with 3
        degrees of freedom.

        :param x: X coordinate
        :type x: float
        :param y: Y coordinate
        :type y: float
        :param z: Z coordinate
        :type z: float
        :return: List of angles [[q11, q21], [q21, q22]]
        """
        j1 = atan2(y, x) * (180 / np.pi)
        H = sqrt((y**2) + (x**2))
        L = sqrt((H**2) + (z**2))
        j3 = acos(((self.femur_length**2) + (self.tibia_length**2) - (L**2)) / (2 * self.femur_length * self.tibia_length)) * (180 / np.pi)
        b = acos(((L**2) + (self.femur_length**2) - (self.tibia_length**2)) / (2 * L * self.femur_length)) * (180 / np.pi)
        a = atan2(z, H) * (180 / np.pi)
        j2 = (b + a) 

        return [j1, j2, j3]


    def solve_forward_kinematics(self, q1, q2, q3) -> list:
        """
        This function solves the forward kinematic equations for an end effector with 3
        degrees of freedom.

        :param q1: Joint angle 1
        :type q1: float
        :param q2: Joint angle 2
        :type q2: float
        :param q3: Joint angle 3
        :type q3: float
        :return: List of coordinates [x, y, z]
        """
        
        x1 = self.coxa_length * cos(q1)
        y1 = self.coxa_length * sin(q1)
        z1 = 0
        x2 = x1 + self.femur_length * cos(q1) * cos(q2)
        y2 = y1 + self.femur_length * sin(q1) * cos(q2)
        z2 = self.femur_length * sin(q2)
        x3 = x2 + self.tibia_length * cos(q1) * cos(q2 + q3)
        y3 = y2 + self.tibia_length * sin(q1) * cos(q2 + q3)
        z3 = z2 + self.tibia_length * sin(q2 + q3)

        return [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]
    
if __name__ == "__main__":
    # Example usage
    ik_solver = IkSolver(coxa_length=1.0, femur_length=1.0, tibia_length=1.0)
    x, y, z = 1.0, 1.0, 1.0
    angles = ik_solver.solve_inverse_kinematics(x, y, z)
    print(f"Inverse Kinematics Angles: {angles}")
    
    q1, q2, q3 = angles[0][0], angles[1][0], angles[1][1]
    coordinates = ik_solver.solve_forward_kinematics(q1, q2, q3)
    print(f"Forward Kinematics Coordinates: {coordinates}")
