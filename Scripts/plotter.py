from matplotlib import pyplot as plt
import matplotlib.animation as animation

class Plotter:

    def __init__(self):
        """
        Initialize the Plotter class.
        """
        self.plot_index = 0
    
    def plot_2d_projection(self, forward_kinematics_result: list) -> None:
        """
        Plot the 2D solution of the inverse kinematics.
        :param forward_kinematics_result: The result of the forward kinematics
        :type forward_kinematics_result: list
        :return: None
        """
        # Extracting the coordinates from the forward kinematics result
        x = [forward_kinematics_result[0][0], forward_kinematics_result[1][0], forward_kinematics_result[2][0]]
        y = [forward_kinematics_result[0][1], forward_kinematics_result[1][1], forward_kinematics_result[2][1]]
        z = [forward_kinematics_result[0][2], forward_kinematics_result[1][2], forward_kinematics_result[2][2]]

        # Plotting the leg positions
        fig = plt.figure(self.plot_index)
        ax_1 = fig.add_subplot(121)
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

        ax_2 = fig.add_subplot(122)
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

        self.plot_index += 1

    def plot_animated_2d_projection_xy(self, 
                                    x_positions: list,
                                    y_positions: list,
                                    z_positions: list, 
                                    ik_solver) -> None:
        """
        Plot the 2D solution of the inverse kinematics.
        :param x_positions: The x positions of the leg
        :type x_positions: list
        :param y_positions: the y positions of the leg
        :type y_positions: list
        :param z_positions: the z positions of the leg
        :type z_positions: list
        :param ik_solver: The class that the pyplot animator use to calculate the positions of the legs.
        :type ik_solver: Class
        :return: None
        """

        # Check that the lenght of the lists are equal
        if not (len(x_positions) == len(y_positions) == len(z_positions)):
            raise ValueError("The length of the x, y and z positions must be equal.")
        # Check that the lists are not empty
        if not (x_positions and y_positions and z_positions):
            raise ValueError("The x, y and z positions must not be empty.")
        
        
        coordinates = ik_solver.solve_leg_position_from_target_coordinates(x_positions[0], 
                                                                           y_positions[0], 
                                                                           z_positions[0], 
                                                                           verbose=True)
        # Extracting the coordinates from the forward kinematics result
        x = [coordinates[0][0], coordinates[1][0], coordinates[2][0]]
        y = [coordinates[0][1], coordinates[1][1], coordinates[2][1]]
        z = [coordinates[0][2], coordinates[1][2], coordinates[2][2]]
        
        # Plotting the leg positions
        fig, (ax_1, ax_2) = plt.subplots(ncols=2, figsize=(10, 8))

        ax_1.set_title('Leg Positions')
        ax_1.set_xlabel('X axis')
        ax_1.set_ylabel('Y axis')
        ax_1.set_title('Leg Positions')
        ax_1.scatter(x, y, c='r', marker='o')
        left_plot, = ax_1.plot(x, y, c='b')
        ax_1.text(x[2], y[2], 'Tibia', size=10, zorder=1)
        ax_1.text(x[0], y[0], 'Coxa', size=10, zorder=1)
        ax_1.text(x[1], y[1], 'Femur', size=10, zorder=1)

        ax_2.set_title('Leg Positions')
        ax_2.set_xlabel('X axis')
        ax_2.set_ylabel('Z axis')
        ax_2.set_title('Leg Positions')
        ax_2.scatter(x, z, c='r', marker='o')
        right_plot, = ax_2.plot(x, z, c='b')
        ax_2.text(x[2], z[2], 'Tibia', size=10, zorder=1)
        ax_2.text(x[0], z[0], 'Coxa', size=10, zorder=1)
        ax_2.text(x[1], z[1], 'Femur', size=10, zorder=1)

        def update_animated_plot(i):
            coordinates = ik_solver.solve_leg_position_from_target_coordinates(x_positions[i], 
                                                                           y_positions[i], 
                                                                           z_positions[i], 
                                                                           verbose=True)
            # Extracting the coordinates from the forward kinematics result
            x = [coordinates[0][0], coordinates[1][0], coordinates[2][0]]
            y = [coordinates[0][1], coordinates[1][1], coordinates[2][1]]
            z = [coordinates[0][2], coordinates[1][2], coordinates[2][2]]

            ax_1.scatter(x, y, c='r', marker='o')
            left_plot.set_data(x, y)
            ax_1.text(x[2], y[2], 'Tibia', size=10, zorder=1)
            ax_1.text(x[0], y[0], 'Coxa', size=10, zorder=1)
            ax_1.text(x[1], y[1], 'Femur', size=10, zorder=1)

            ax_2.scatter(x, z, c='r', marker='o')
            right_plot.set_data(x, z)
            ax_2.text(x[2], z[2], 'Tibia', size=10, zorder=1)
            ax_2.text(x[0], z[0], 'Coxa', size=10, zorder=1)
            ax_2.text(x[1], z[1], 'Femur', size=10, zorder=1)
            
            return left_plot, right_plot
        
        ani = animation.FuncAnimation(fig, update_animated_plot, frames=len(x_positions), interval=100)
        plt.show()
        self.plot_index += 1 

    
    def plot_animated_3d_projection(self, forward_kinematics_result: list) -> None:
        """
        Plot the 3D solution of the inverse kinematics.
        :param forward_kinematics_result: The result of the forward kinematics
        :type forward_kinematics_result: list
        :return: None
        """
        # Extracting the coordinates from the forward kinematics result
        x = [forward_kinematics_result[0][0], forward_kinematics_result[1][0], forward_kinematics_result[2][0]]
        y = [forward_kinematics_result[0][1], forward_kinematics_result[1][1], forward_kinematics_result[2][1]]
        z = [forward_kinematics_result[0][2], forward_kinematics_result[1][2], forward_kinematics_result[2][2]]

        # Plotting the leg positions
        fig = plt.figure(self.plot_index)
        ax = fig.add_subplot()

        # Setting the Axes properties
        ax.set(xlabel='X')
        ax.set(ylabel='Y')
        ax.set(zlabel='Z')
        
        ani = animation.FuncAnimation(fig, lambda i: ax.scatter(x, y, z, c='r', marker='o'), frames=10, interval=1000)
        ax.set_title('Leg Positions')
        ax.text(x[2], y[2], z[2], 'Tibia', size=10, zorder=1)
        ax.text(x[0], y[0], z[0], 'Coxa', size=10, zorder=1)
        ax.text(x[1], y[1], z[1], 'Femur', size=10, zorder=1)

        raise NotImplementedError("3D animation is not implemented yet.")

        self.plot_index += 1

    def show_all_plots(self, file_name=None) -> None:
        """
        Show all the plots.
        :param file_name: Name of the file to save the plot, defaults to None
        :type file_name: str, optional
        :return: None
        """
        if file_name is not None:
            plt.savefig(file_name)
        plt.show()
        plt.close()
        self.plot_index = 0


if __name__ == "__main__":
    # Example usage
    plotter = Plotter()
    plotter.plot_2d_solution()
