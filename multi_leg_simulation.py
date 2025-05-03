from Scripts.iksolver import IkSolver
from Scripts.plotter import Plotter

if __name__ == "__main__":

    # Leg 1
    femur_length_1 = 3.5
    tibia_length_1 = 3.5

    # Leg 2
    femur_length_2 = 3.5
    tibia_length_2 = 3.5

    # Leg 3
    femur_length_3 = 3.5
    tibia_length_3 = 3.5

    # Leg 4
    femur_length_4 = 3.5
    tibia_length_4 = 3.5

    # Leg 5
    femur_length_5 = 3.5
    tibia_length_5 = 3.5

    # Leg 6
    femur_length_6 = 3.5
    tibia_length_6 = 3.5

    # instantiate the inverse kinematic solver for each leg
    ik_1 = IkSolver(femur_length=femur_length_1, 
                  tibia_length=tibia_length_1)
    
    ik_2 = IkSolver(femur_length=femur_length_2, 
                  tibia_length=tibia_length_2)

    ik_3 = IkSolver(femur_length=femur_length_3, 
                  tibia_length=tibia_length_3)
    
    ik_4 = IkSolver(femur_length=femur_length_4, 
                  tibia_length=tibia_length_4)
    
    ik_5 = IkSolver(femur_length=femur_length_5, 
                  tibia_length=tibia_length_5)
    
    ik_6 = IkSolver(femur_length=femur_length_6, 
                  tibia_length=tibia_length_6)
    
    try:
        ik_7 = IkSolver(femur_length=femur_length_6, 
                  tibia_length=tibia_length_6)
    except RuntimeError as e:
        print(f"Too many instances of the leg solver have been instanciated.")
        print(e)
    
    x=1.0
    y=1.2
    z=3.5

    # Plot an animated 3D projection of the solution
    x_targets = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0]
    y_targets = [1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.7, 1.6, 1.5, 1.4, 1.3, 1.2]
    z_targets = [3.5, 3.6, 3.7, 3.8, 3.7, 3.6, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5]

    # plot the animated 3d projection of the solution
    plotter = Plotter()
    plotter.plot_animated_3d_projection(x_targets, y_targets, z_targets, ik_solver = ik_1)


    # plot the animated 3d projection of the solution
    print(f"Leg 1: {ik_1}")
    print(f"Leg 2: {ik_2}")
    print(f"Leg 3: {ik_3}")
    print(f"Leg 4: {ik_4}")
    print(f"Leg 5: {ik_5}")
    print(f"Leg 6: {ik_6}")
    plotter.plot_animated_3d_projection(x_targets, y_targets, z_targets, ik_solver = [ik_1, ik_2, ik_3, ik_4, ik_5, ik_6])
