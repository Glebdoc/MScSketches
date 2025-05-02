import numpy as np
import matplotlib.pyplot as plt
import pyvista as pv

def rotation_matrix(angle_x=0, angle_y=0, angle_z=0):
        """ Compute the full 3D rotation matrix (XYZ order). """
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(angle_x), -np.sin(angle_x)],
                       [0, np.sin(angle_x), np.cos(angle_x)]])
        
        Ry = np.array([[np.cos(angle_y), 0, np.sin(angle_y)],
                       [0, 1, 0],
                       [-np.sin(angle_y), 0, np.cos(angle_y)]])
        
        Rz = np.array([[np.cos(angle_z), -np.sin(angle_z), 0],
                       [np.sin(angle_z), np.cos(angle_z), 0],
                       [0, 0, 1]])

        return Rz @ Ry @ Rx 


vector = np.array([0, 0, 2])

# rotate around y axis - 90 

angle_y = -np.pi/2
angle_z = 2*np.pi/3

R = rotation_matrix(angle_y=angle_y)
rotated_vector_y = R @ vector

R = rotation_matrix(angle_y = angle_y, angle_z=angle_z)
rotated_vector_yz = R @ vector

# Define origin
origin = np.array([0, 0, 0])

import pyvista as pv
import numpy as np




# Define vectors and origin
# vectors = np.array([
#     vector,
#     rotated_vector_y,
#     rotated_vector_yz
# ])
# origins = np.array([
#     [0, 0, 0],
#     [0, 0, 0],
#     [0, 0, 0]
# ])

# # Create PyVista point cloud
# point_cloud = pv.PolyData(origins)
# point_cloud["vectors"] = vectors  # attach vectors to each point

# # Use glyphs to draw arrows with actual magnitude
# arrows = point_cloud.glyph(orient="vectors", scale="vectors", factor=1.0)

# # Plot
# plotter = pv.Plotter()
# plotter.add_mesh(arrows, color="orange")
# plotter.add_point_labels(origins + vectors, ["v1", "v2", "v3"], font_size=36, text_color="black")
# plotter.add_axes_at_origin()
# plotter.show_bounds(xlabel='X', ylabel='Y', zlabel='Z')
# plotter.show()
