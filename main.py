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

# bigPropPos = Point(0,0,0)
# smallPropPos = Point(0,0.5,0)

# bigProp = Propeller(bigPropPos, angles=(0,0,0), hub=0.1, diameter=1, NB=3, pitch=12, RPM=100, chord=0.1, n=10, U=2, wake_length=1)
# smallProp = Propeller(smallPropPos, angles=(0, -np.radians(90),0), hub=0.1, diameter=0.2, NB=3, pitch=12, RPM=1000, chord=0.03, n=10, U=4, wake_length=4)
# bigPropShoes = bigProp.horseShoes

# scene([bigProp, smallProp], ["blue", "red"])

# Main propeller parameters
main_position = Point(0, 0, 0)
main_angles = (0, 0, 0)  # Initial rotation angles (e.g., Euler angles)
main_hub = 0.1
main_diameter = 2.0
main_NB = 3  # Number of blades
main_pitch = 0.2
main_RPM = 100
main_chord = 0.1
main_n = 4

# Small propeller parameters (e.g., 4 small propellers)
small_props_angles = (0, 0, 0)  # Initial rotation angles (e.g., Euler angles)
small_props_diameter = 0.2
small_props_NB = 3
small_props_RPM = 1000
small_props_chord = 0.03
small_props_n = 3

# Create the drone
drone = Drone(main_position, main_angles, main_hub, main_diameter, 
              main_NB, main_pitch, main_RPM, main_chord, main_n,
              small_props_angles, small_props_diameter, small_props_NB,
              small_props_RPM, small_props_chord, small_props_n,
              mainWakeLength=1, smallWakeLength=4)



extra= solve(drone)

drone.display(color_main='blue', color_small='green', extra_points=extra)