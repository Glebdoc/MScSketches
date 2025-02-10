import numpy as np

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