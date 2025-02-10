import numpy as np
import pyvista as pv
from bemUtils import*

from geometry import*

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

bigPropPos = Point(0,0,0)

bigProp = Propeller(bigPropPos, hub=0.1, diameter=1, NB=1, pitch=12, RPM=100, chord=0.2, n=10)
bigPropShoes = bigProp.horseShoes
bigProp.display()