from Scripts.iksolver import IkSolver
from Scripts.plotter import Plotter
from Scripts.gait_controller import GaitController, GaitPattern

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

    print(f"Leg 1: {ik_1}")

    # plot the animated 3d projection of the solution
    plotter = Plotter()
    plotter.plot_animated_3d_projection(ik_solver = ik_1)


    # plot the animated 3d projection of the solution
    print(f"Leg 1: {ik_1}")
    print(f"Leg 2: {ik_2}")
    print(f"Leg 3: {ik_3}")
    print(f"Leg 4: {ik_4}")
    print(f"Leg 5: {ik_5}")
    print(f"Leg 6: {ik_6}")
    plotter.plot_animated_3d_projection(ik_solver = [ik_1, ik_2, ik_3, ik_4, ik_5, ik_6])

    del ik_1
    del ik_2
    del ik_3
    del ik_4
    del ik_5
    del ik_6

    # Create gait controllers directly with leg parameters
    gc_1 = GaitController(femur_length=3.5, tibia_length=3.5, number_samples=50)
    gc_2 = GaitController(femur_length=3.5, tibia_length=3.5, number_samples=50)
    gc_3 = GaitController(femur_length=3.5, tibia_length=3.5, number_samples=50)
    gc_4 = GaitController(femur_length=3.5, tibia_length=3.5, number_samples=50)
    gc_5 = GaitController(femur_length=3.5, tibia_length=3.5, number_samples=50)
    gc_6 = GaitController(femur_length=3.5, tibia_length=3.5, number_samples=50)

    # Set gait patterns
    for gc in [gc_1, gc_2, gc_3, gc_4, gc_5, gc_6]:
        gc.set_gait_pattern(GaitPattern.TRIPOD)

    # Plot using the gait controllers directly
    plotter.plot_animated_3d_projection([gc_1, gc_2, gc_3, gc_4, gc_5, gc_6])
