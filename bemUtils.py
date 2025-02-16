import numpy as np
import pyvista as pv

def get_axis_vectors(length=1.5):
    """
    Generates axis vectors for visualization.

    Returns:
        tuple: (axis_origins, axis_directions, axis_colors)
    """
    origins = np.array([
        [0, 0, 0],  # Origin for all axes
        [0, 0, 0],
        [0, 0, 0]
    ])
    
    directions = np.array([
        [length, 0, 0],  # X-axis direction
        [0, length, 0],  # Y-axis direction
        [0, 0, length]   # Z-axis direction
    ])
    
    colors = ["red", "green", "blue"]  # X (red), Y (green), Z (blue)

    return origins, directions, colors

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

def scene(bodies, colors, collocation_points, velocity_vectors, extra_points=None, extra_lines=None):
        plotter = pv.Plotter()
        plotter.set_background("black") 

        if velocity_vectors is not None:
            print("Plotting velocity vectors")
            print('velocity_vectors.shape',velocity_vectors.shape)
            print('collocation_points.shape',collocation_points.shape)
            poly_data = pv.PolyData(collocation_points)
            poly_data['vectors'] = velocity_vectors
            magnitude = np.linalg.norm(velocity_vectors, axis=1)
            poly_data['magnitude'] = magnitude
            glyphs = poly_data.glyph(orient='vectors', scale='magnitude', factor=0.003)
            plotter.add_mesh(glyphs, color="white")

        if extra_points is not None:
            poly_data = pv.PolyData(np.array(extra_points))
            poly_data.lines = np.array(extra_lines)
            plotter.add_mesh(poly_data, color="white", line_width=2)


        for j, body in enumerate(bodies):
            all_points = []
            all_lines = []
            coll_points = []
            start_index = 0

            for c in range(len(body.collocationPoints)):
                coll_points.extend(body.collocationPoints[c].T)
                 

            for i in body.horseShoes:
                points, lines = i.get_plot_data(start_index)
                all_points.extend(points)
                all_lines.extend(lines)
                start_index += len(points)

        
            poly_data = pv.PolyData(np.array(all_points))
            poly_data.lines = np.array(all_lines)
            plotter.add_mesh(poly_data, color=colors[j], line_width=2)
            poly_data = pv.PolyData(np.array(coll_points))
            plotter.add_mesh(poly_data, color="red")

        axis_origins, axis_directions, axis_colors = get_axis_vectors()
        for i in range(3):
            plotter.add_arrows(
                np.array([axis_origins[i]]), 
                np.array([axis_directions[i]]), 
                color=axis_colors[i], 
                mag=0.1
        )

        plotter.show()