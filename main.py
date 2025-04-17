from Scripts.iksolver import IkSolver
from Scripts.plotter import Plotter

if __name__ == "__main__":

    coxa_length = 1.0
    femur_length = 3.5
    tibia_length = 3.5

    # Example usage
    ik = IkSolver(coxa_length=coxa_length, 
                  femur_length=femur_length, 
                  tibia_length=tibia_length)
    
    x=1.0
    y=1.0
    z=1.0

    # Assuming iksolver has a method to get the 2D solution
    solution = ik.solve_inverse_kinematics(x, y, z)  # Replace with actual method call
    print(f"Inverse Kinematics Solution: "
          f"\n{solution}")
    q1, q2, q3 = solution[0], solution[1], solution[2]
    coordinates = ik.solve_forward_kinematics(q1, q2, q3)
    print(f"Forward Kinematics Coordinates: "
          f"\n{coordinates[0]}"
          f"\n{coordinates[1]}"
          f"\n{coordinates[2]}")
    # Assuming plotter has a method to plot the 2D solution
    # plotter = Plotter()
    # plotter.plot_2d_solution(solution)  # Replace with actual method call
