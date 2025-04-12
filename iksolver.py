# This file is for solving the inverse kinematics of a hexapod robot.

import numpy as np
from math import atan2, sqrt, acos
from matplotlib import pyplot as plt

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
        if verbose:
            print(f"Input coordinates: x={x}, y={y}, z={z}")
        # Check if the coordinates are reachable
        if sqrt(x**2 + y**2) > self.coxa_length + self.femur_length + self.tibia_length:
            raise ValueError("Coordinates are unreachable")

        # Calculate required lengths
        l1 = sqrt(x**2 + y**2)
        l = sqrt(self.z_offset**2 + (l1-self.coxa_length)**2)
        if verbose:
            print(f"l1: {l1}, l: {l}")

        # Check if the coordinates are reachable
        if l > self.femur_length + self.tibia_length:
            raise ValueError("Coordinates are unreachable")

        # Calculate gamma
        gamma = atan2(y, x)
        if verbose:
            print(f"gamma: {gamma}")

        # Calculate alpha
        alpha1 = acos(self.z_offset/l)
        if verbose:
            print(f"alpha1: {alpha1}")
        alpha2 = acos((-l**2 - self.femur_length**2 + self.tibia_length**2)/(-2*l*self.tibia_length))
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

    def get_leg_positions(self, angles: list) -> dict:
        """
        Calculates the coordinates of the coxa, femur, and tibia joints based on the given angles.
        :param angles: List of angles [alpha, beta, gamma]
        :type angles: list
        :return: Dictionary with coordinates of coxa, femur, and tibia joints
        """
        alpha, beta, gamma = angles

        # Coxa position
        coxa_x = self.coxa_length * np.cos(gamma)
        coxa_y = self.coxa_length * np.sin(gamma)
        coxa_z = 0

        # Femur position
        femur_x = coxa_x + self.femur_length * np.cos(alpha) * np.cos(gamma)
        femur_y = coxa_y + self.femur_length * np.cos(alpha) * np.sin(gamma)
        femur_z = self.femur_length * np.sin(alpha)

        # Tibia position (end effector)
        tibia_x = femur_x + self.tibia_length * np.cos(alpha + beta) * np.cos(gamma)
        tibia_y = femur_y + self.tibia_length * np.cos(alpha + beta) * np.sin(gamma)
        tibia_z = femur_z + self.tibia_length * np.sin(alpha + beta)

        return {
            "coxa": (coxa_x, coxa_y, coxa_z),
            "femur": (femur_x, femur_y, femur_z),
            "tibia": (tibia_x, tibia_y, tibia_z)
        }

if __name__ == "__main__":
    print('Inverse Kinematics Solver for Hexapod Robot')

    # Example parameters
    coxa_length: float = 1.0
    femur_length: float = 3.0
    tibia_length: float = 5.0
    z_offset: float = 2

    # Example usage
    x: float = 1.0
    y: float = 1.5
    z: float = 0.0

    ik_solver = IkSolver(coxa_length, femur_length, tibia_length, z_offset)
    angles = ik_solver.solve_angles(x, y, z, verbose=True)
    print(f"Angles: {angles}")
    print('')

    # Get leg positions
    leg_positions = ik_solver.get_leg_positions(angles)
    print(f"Leg positions: {leg_positions}")
    # Extracting the coordinates for plotting
    coxa_pos = leg_positions["coxa"]
    femur_pos = leg_positions["femur"]
    tibia_pos = leg_positions["tibia"]
    x = [coxa_pos[0], femur_pos[0], tibia_pos[0]]
    y = [coxa_pos[1], femur_pos[1], tibia_pos[1]]
    z = [coxa_pos[2], femur_pos[2], tibia_pos[2]]
    print(f"x: {x},\ny: {y},\nz: {z}")

    # Plotting the leg positions
    fig = plt.figure()
    ax_1 = fig.add_subplot(221)
    ax_1.set_xlabel('X axis')
    ax_1.set_ylabel('Y axis')
    ax_1.set_title('Leg Positions')
    ax_1.scatter(x, y, c='r', marker='o')
    ax_1.plot(x, y, c='b')
    ax_1.text(x[2], y[2], 'Tibia', size=10, zorder=1)
    ax_1.text(x[0], y[0], 'Coxa', size=10, zorder=1)
    ax_1.text(x[1], y[1], 'Femur', size=10, zorder=1)

    ax_2 = fig.add_subplot(222)
    ax_2.set_xlabel('X axis')
    ax_2.set_ylabel('Z axis')
    ax_2.set_title('Leg Positions')
    ax_2.scatter(x, z, c='r', marker='o')
    ax_2.plot(x, z, c='b')
    ax_2.text(x[2], z[2], 'Tibia', size=10, zorder=1)
    ax_2.text(x[0], z[0], 'Coxa', size=10, zorder=1)
    ax_2.text(x[1], z[1], 'Femur', size=10, zorder=1)

    ax_3 = fig.add_subplot(212, projection='3d')
    ax_3.set_xlabel('X axis')
    ax_3.set_ylabel('Y axis')
    ax_3.set_zlabel('Z axis')
    ax_3.set_title('Leg Positions')
    ax_3.scatter(x, y, z, c='r', marker='o')
    ax_3.plot(x, y, z, c='b')
    ax_3.text(x[2], y[2], z[2], 'Tibia', size=10, zorder=1)
    ax_3.text(x[0], y[0], z[0], 'Coxa', size=10, zorder=1)
    ax_3.text(x[1], y[1], z[1], 'Femur', size=10, zorder=1)
    plt.show()
