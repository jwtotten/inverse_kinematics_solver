import numpy as np
from math import atan2, sqrt, acos, sin, cos

class IkSolver(object):
    _instances = []  # Keep track of instance reference
    limit = 6

    def __init__(self, femur_length, tibia_length) -> None:
        """
        This function initializes the inverse kinematic solver.
        :param femur_length: Length of the femur
        :type femur_length: float
        :param tibia_length: Length of the tibia
        :type tibia_length: float
        :return: None
        """
        # Leg lengths
        self.femur_length = femur_length
        self.tibia_length = tibia_length
        
        # Leg offsets
        self.x_offset: float = -5
        self.y_offset:float = -2
        self.z_offset: float = -6

        # Body lengths
        self.x_length: float = 5
        self.y_length: float = 3
        self.z_length: float = 1
    
    def __new__(cls, *args, **kwargs):
        if not len(cls._instances) < cls.limit:
            raise RuntimeError("Count not create instance. Limit %s reached" % cls.limit)    
        instance = super().__new__(cls)
        cls._instances.append(instance)
        return instance
    
    def __repr__(self) -> str:
        return_string: str = (f"{type(self).__name__}"
                              f"(femur length={self.x}," 
                              f"tibia length={self.y},"
                              f"x offset={self.x_offset},"
                              f"y offset={self.y_offset},"
                              f"z offset={self.z_offset})")
        return return_string

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
        x, y, z = self.apply_offsets(x, y, z)

        j1 = atan2(y, x)
        H = sqrt((y**2) + (x**2))
        L = sqrt((H**2) + (z**2))
        j3 = acos(((self.femur_length**2) + (self.tibia_length**2) - (L**2)) / (2 * self.femur_length * self.tibia_length))
        b = acos(((L**2) + (self.femur_length**2) - (self.tibia_length**2)) / (2 * L * self.femur_length))
        a = atan2(z, H)
        j2 = (b + a) 

        return [j1, j2, j3]
    
    def apply_offsets(self, x, y, z) -> list:
        """
        This function applies the offsets to the coordinates.

        :param x: X coordinate
        :type x: float
        :param y: Y coordinate
        :type y: float
        :param z: Z coordinate
        :type z: float
        :return: List of coordinates [x, y, z]
        """
        # Apply offsets to the coordinates
        x += self.x_offset
        y += self.y_offset
        z += self.z_offset

        return [x, y, z]
    
    def solve_leg_position_from_target_coordinates(self, x: float, y: float, z: float, verbose:bool = False) -> list:
        """
        This fuction takes in the target coordinates and returns the leg position
        in the form of a list of coordinates.
        :param x_list: list of target X coordinate  
        :type x_list: list
        :param y_list: list of target Y coordinate
        :type y_list: list
        :param z_list: list of target Z coordinate
        :type z_list: list
        :param verbose: If True, print the coordinates
        :type verbose: bool
        :return: List of coordinates [x, y, z]
        """

        if verbose:
            print(f"Target Coordinates: "
                f"\nX: {x}"
                f"\nY: {y}"
                f"\nZ: {z}")

        solution = self.solve_inverse_kinematics(x, y, z)

        if verbose:
            print(f"Inverse Kinematics Solution: "
                f"\n{solution}")
            
        q1, q2, q3 = solution[0], solution[1], solution[2]
        coordinates = self.solve_forward_kinematics(q1, q2, q3)

        if verbose:
            print(f"Calculated leg coordinates:"
                f"\n{coordinates}")
            
        return coordinates

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
        
        x1 = 0
        y1 = 0
        z1 = 0
        x2 = x1 - self.femur_length * cos(q1) * cos(q2)
        y2 = y1 - self.femur_length * sin(q1) * cos(q2)
        z2 = self.femur_length * sin(q2)
        x3 = x2 - self.tibia_length * cos(q1) * cos(q2 + q3)
        y3 = y2 - self.tibia_length * sin(q1) * cos(q2 + q3)
        z3 = z2 - self.tibia_length * sin(q2 + q3)

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
