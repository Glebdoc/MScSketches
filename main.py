import numpy as np
import pyvista as pv
from bemUtils import*

from geometry import*
from solver import*

def plot(all_points, all_lines):
    plotter = pv.Plotter()
    poly_data = pv.PolyData(np.array(all_points))
    poly_data.lines = np.array(all_lines)
    plotter.add_mesh(poly_data, color="blue", line_width=2)

    # Plot coordinate axes as vectors
    axis_origins, axis_directions, axis_colors = get_axis_vectors()
    for i in range(3):
        plotter.add_arrows(
            np.array([axis_origins[i]]), 
            np.array([axis_directions[i]]), 
            color=axis_colors[i], 
            mag=0.1
        )
    plotter.show()

# Main propeller parameters
main_position = Point(0, 0, 0)
main_angles = (0, 0, 0)  # Initial rotation angles (e.g., Euler angles)
main_hub = 0.3
main_diameter = 3.
main_NB = 3  # Number of blades
main_pitch = 0.2
main_RPM = 350
main_chord = 0.3
main_n = 25

# Small propeller parameters (e.g., 4 small propellers)
small_props_angles = (0, 0, 0)  # Initial rotation angles (e.g., Euler angles)
small_props_diameter = 0.3
small_props_NB = 3
small_props_RPM = 10000
small_props_chord = 0.05
small_props_n = 7

# Create the drone

err = 1.0
iter = 0
max_iter = 100
weight = 0.5
main_U = 5.11
small_U = 61.69

runWhole = False

if runWhole:
    while (err > 1e-1 and iter<max_iter):
        

        drone = Drone(main_position, main_angles, main_hub, main_diameter, 
                    main_NB, main_pitch, main_RPM, main_chord, main_n,
                    small_props_angles, small_props_diameter, small_props_NB,
                    small_props_RPM, small_props_chord, small_props_n,
                    mainWakeLength=1, smallWakeLength=6, main_U=main_U, small_U = small_U, main_distribution='uniform', small_distribution='uniform')

        old_main_U = main_U
        old_small_U = small_U

        main_U, small_U, _,_= solve(drone)

        main_U = weight*main_U + (1-weight)*old_main_U
        small_U = weight*small_U + (1-weight)*old_small_U

        err = np.abs(main_U - old_main_U) + np.abs(small_U - old_small_U)
        if err < 1e-1:
            main_U, small_U, horses, Gammas = solve(drone,plotting=False, updateConfig=True)
        iter += 1
        print(f'Iteration: {iter}, Error: {err}, Main U: {main_U}, Small U: {small_U}')

    drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)
    #u, v, w = computeVelocityField(horses, Gammas)

else:
    drone = Drone(main_position, main_angles, main_hub, main_diameter, 
                    main_NB, main_pitch, main_RPM, main_chord, main_n,
                    small_props_angles, small_props_diameter, small_props_NB,
                    small_props_RPM, small_props_chord, small_props_n,
                    mainWakeLength=1, smallWakeLength=6, main_U=main_U, small_U = small_U, main_distribution='uniform', small_distribution='uniform')
    main_U, small_U, _, _ = solve(drone,plotting=True, updateConfig=True)
    drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)