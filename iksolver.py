# This file is for solving the inverse kinematics of a hexapod robot.

import numpy as np
from math import atan2, sqrt, acos, sin, cos
from matplotlib import pyplot as plt

class IkSolver:
    def __init__(self, coxa_length, femur_length, tibia_length, z_offset=0):
        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length
        self.z_offset = z_offset

    def rik2(self, x:float, z:float, l1:float, l2:float, verbose:bool = False):
        """
        This function calculates the rik2 for a given x and y.
        :param x: X coordinate
        :type x: float
        :param z: Z coordinate
        :type z: float
        :return: List of angles [[q11, q21], [q21, q22]]
        """

        xd = sqrt((x**2 + z**2))

        c2 = (xd**2-l1**2-l2**2)/(2*l1*l2)

        if verbose:
            print(f"xd: {xd}, c2: {c2}")
        if abs(c2) > 1:
            raise ValueError("Coordinates are unreachable")
        elif c2 == 1:
            if verbose:
                print("Coordinates are reachable")  
            return [atan2(y, x), 0]
        elif c2 == -1:
            if verbose:
                print("Coordinates are reachable")
            return [atan2(y, x), np.pi]
        else:
            q21 = acos(c2)
            q22 = -acos(c2)
            theta = atan2(z, x)
            q11 = theta - atan2(l2*sin(q21), l1+l2*cos(q21))
            q12 = theta - atan2(l2*sin(q22), l1+l2*cos(q22))

        return [[q11, q21], [q12, q22]]
    
    def rik3(self, x:float, y:float, z:float, verbose:bool = False):
        """
        This function calculates the rik2 and the for a given z, calculates all 4 solutions for the rik3.
        :param x: X coordinate
        :type x: float
        :param y: Y coordinate
        :type y: float
        :param z: Z coordinate
        :type z: float
        :return: List of angles [[q11, q21], [q21, q22]]
        """

        l1 = self.coxa_length
        l2 = self.femur_length
        l3 = self.tibia_length

        xrot = sqrt(x**2 + y**2)
        zrot = self.z_offset

        [[q11, q21], [q12, q22]] = self.rik2(xrot, zrot, l2, l3, verbose)
        q31 = atan2(y, xrot)
        [[q11_1, q21_1], [q12_1, q22_1]] = self.rik2(-xrot, zrot, l2, l3, verbose)
        q32 = -atan2(y, xrot)

        return [[q11, q21, q31], [q12, q22, q31], [q11_1, q21_1, q32], [q12_1, q22_1, q32]]

    def apply_boundary_conditions(self, angles: list) -> list:
        """
        Apply the boundary conditions of the robot to the possible solutions of the inverse 
        kinematics.
        :param angles: List of angles [alpha, beta, gamma]
        :type angles: list
        :return: List of angles after applying the boundary conditions
        """
        raise NotImplementedError("Boundary conditions are not implemented yet")

    def forward_kinematics(self, angles: list) -> dict:
        """
        Calculates the x, y, and z coordinates of each segment of a 3-part end effector using forward kinematics.
        :param angles: List of angles [alpha, beta, gamma]
        :type angles: list
        :return: Dictionary with coordinates of coxa, femur, and tibia joints
        """
        alpha, beta, gamma = angles

        # Coxa position
        coxa_x = self.coxa_length * cos(alpha)
        coxa_y = self.coxa_length * sin(alpha)
        coxa_z = self.z_offset

        # Femur position
        femur_x = coxa_x + self.femur_length * cos(alpha + beta)
        femur_y = coxa_y + self.femur_length * sin(alpha + beta)
        femur_z = coxa_z

        # Tibia position (end effector)
        tibia_x = femur_x + self.tibia_length * cos(alpha + beta + gamma)
        tibia_y = femur_y + self.tibia_length * sin(alpha + beta + gamma)
        tibia_z = femur_z

        return {
            "coxa": (coxa_x, coxa_y, coxa_z),
            "femur": (femur_x, femur_y, femur_z),
            "tibia": (tibia_x, tibia_y, tibia_z)
        }
    
    def get_leg_positions(self, angles: list) -> dict:
        """
        Calculates the coordinates of the coxa, femur, and tibia joints based on the given angles.
        :param angles: List of angles [alpha, beta, gamma]
        :type angles: list
        :return: Dictionary with coordinates of coxa, femur, and tibia joints
        """
        alpha, beta, gamma = angles

        # Coxa position - should be pinned to the XY origin and adjusted by the z height.
        coxa_x = self.coxa_length * cos(alpha)
        coxa_y = self.coxa_length * sin(alpha)
        coxa_z = self.z_offset

        # Femur position
        femur_x = self.coxa_length * cos(alpha)
        femur_y = self.coxa_length * sin(alpha)
        femur_z = self.femur_length * sin(beta) + self.z_offset

        # Tibia position (end effector)
        tibia_x = femur_x + self.tibia_length * cos(alpha + beta) * cos(gamma)
        tibia_y = femur_y + self.tibia_length * cos(alpha + beta) * sin(gamma)
        tibia_z = femur_z + self.tibia_length * sin(alpha + beta)

        return {
            "coxa": (coxa_x, coxa_y, coxa_z),
            "femur": (femur_x, femur_y, femur_z),
            "tibia": (tibia_x, tibia_y, tibia_z)
        }

if __name__ == "__main__":
    print('Inverse Kinematics Solver for Hexapod Robot')

    # Example parameters
    coxa_length: float = 1.0
    femur_length: float = 3.5
    tibia_length: float = 7.0
    z_offset: float = 2

    #example usage of rik3
    x = 2.5
    y = 2.0
    z = 2.0
    ik_solver = IkSolver(coxa_length, femur_length, tibia_length, z_offset)
    all_angles = ik_solver.rik3(x, y, z, verbose=True)
    print(f"Angles: {all_angles}")
    print(" ")

    for index, angles in enumerate(all_angles):

        # Get leg positions
        leg_positions = ik_solver.forward_kinematics(angles)
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
        fig = plt.figure(index)
        ax_1 = fig.add_subplot(221)
        ax_1.set_xlabel('X axis')
        ax_1.set_ylabel('Y axis')
        ax_1.set_title('Leg Positions')
        ax_1.scatter(x, y, c='r', marker='o')
        ax_1.plot(x, y, c='b')
        ax_1.text(x[2], y[2], 'Tibia', size=10, zorder=1)
        ax_1.text(x[0], y[0], 'Coxa', size=10, zorder=1)
        ax_1.text(x[1], y[1], 'Femur', size=10, zorder=1)
        # plot the body of the robot
        ax_1.axhline(y=0, xmin=0, xmax=1, c='g')
        ax_1.axvline(x=0, ymin=0, ymax=1, c='g')

        ax_2 = fig.add_subplot(222)
        ax_2.set_xlabel('X axis')
        ax_2.set_ylabel('Z axis')
        ax_2.set_title('Leg Positions')
        ax_2.scatter(x, z, c='r', marker='o')
        ax_2.plot(x, z, c='b')
        ax_2.text(x[2], z[2], 'Tibia', size=10, zorder=1)
        ax_2.text(x[0], z[0], 'Coxa', size=10, zorder=1)
        ax_2.text(x[1], z[1], 'Femur', size=10, zorder=1)
        # plot the body of the robot
        ax_2.axhline(y=0, xmin=0, xmax=1, c='g')
        ax_2.axvline(x=0, ymin=0, ymax=1, c='g')

        ax_3 = fig.add_subplot(212, projection='3d')
        ax_3.set_xlabel('X axis')
        ax_3.set_ylabel('Y axis')
        ax_3.set_zlabel('Z axis')
        ax_3.set_title('Leg Positions')
        ax_3.scatter(x, y, z, c='r', marker='o')
        ax_3.plot(x, y, z, c='b')
        ax_3.plot([0, x[0]], [0, y[0]], [z_offset, z_offset], c='b')
        
        # plot the body of the robot
        ax_3.plot([-1, 1], [0, 0], [z_offset-1, z_offset-1], c='g')
        ax_3.plot([0, 0], [-1, 1], [z_offset-1, z_offset-1], c='g')
        ax_3.plot([0, 0], [0, 0], [z_offset-1, z_offset+1], c='g')
        ax_3.plot([-1, 1], [0, 0], [z_offset+1, z_offset+1], c='g')
        ax_3.plot([0, 0], [-1, 1], [z_offset+1, z_offset+1], c='g')

        ax_3.scatter(0, 0, z_offset, c='r', marker='o')
        ax_3.text(x[2], y[2], z[2], 'End Effector', size=10, zorder=1)
        ax_3.text(x[0], y[0], z[0], 'Femur', size=10, zorder=1)
        ax_3.text(x[1], y[1], z[1], 'Tibia', size=10, zorder=1)
        ax_3.text(0, 0, z_offset, 'Coxa', size=10, zorder=1)

    plt.show()
