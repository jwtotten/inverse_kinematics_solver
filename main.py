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
    print(f"Inverse Kinematics Solution: {solution}")
    # Assuming plotter has a method to plot the 2D solution
    # plotter = Plotter()
    # plotter.plot_2d_solution(solution)  # Replace with actual method call
