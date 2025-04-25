from Scripts.iksolver import IkSolver
from Scripts.plotter import Plotter

if __name__ == "__main__":

    femur_length = 3.5
    tibia_length = 3.5

    # Example usage
    ik = IkSolver(femur_length=femur_length, 
                  tibia_length=tibia_length)
    
    x=1.0
    y=1.2
    z=3.5

    # Assuming iksolver has a method to get the 2D solution
    solution = ik.solve_inverse_kinematics(x, y, z)
    print(f"Inverse Kinematics Solution: "
          f"\n{solution}")
    q1, q2, q3 = solution[0], solution[1], solution[2]
    coordinates = ik.solve_forward_kinematics(q1, q2, q3)
    print(f"Forward Kinematics Coordinates: "
          f"\n{coordinates[0]}"
          f"\n{coordinates[1]}"
          f"\n{coordinates[2]}")
    
    # Plot the 2D projection of the solution
    plotter = Plotter()
    plotter.plot_2d_projection(coordinates)  # Replace with actual method call
    plotter.show_all_plots()  # Show all plots

    # Plot an animated 2D projection of the solution
    x_targets = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0]
    y_targets = [1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.7, 1.6, 1.5, 1.4, 1.3, 1.2]
    z_targets = [3.5, 3.6, 3.7, 3.8, 3.7, 3.6, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5]

    plotter.plot_animated_2d_projection_xy(x_targets, y_targets, z_targets, ik_solver = ik)
    

    # plot the animated 3d projection of the solution
    plotter.plot_animated_3d_projection(x_targets, y_targets, z_targets, ik_solver = ik)


