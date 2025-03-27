import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
import os, json
from geometry import*

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

def contractionCoefficients(y_close, z_close, z_far, R):
    """
    Compute the contraction coefficients for a wake. 
    Assuming that contraction is represented by a parabola given by z = a*y^2 + b*y + c.
    We are looking for the coefficients a, b, and c based on the given points.
    
    Parameters:
        y_close (float): y value at z_close
        z_close (float): z value at y_close
        z_far (float): z value at y = 0.78R
        R (float): Reference radius of the rotor wake
        
    Returns:
        (a, b, c): Coefficients of the parabolic equation z = a*y^2 + b*y + c
    """
    # Define the system of equations
    A = np.array([[0, 0, 1],  # z = 0 at y = R
                  [z_close**2, z_close, 1],  # z_close at y_close
                  [z_far**2, z_far, 1]])  # z_far at y = 0.78R
                  
    b = np.array([R, y_close, 0.78*R])

    # Solve for coefficients
    coefs = np.linalg.solve(A, b)
    
    return coefs
def find_parabola(points):

    (x1, y1), (x2, y2), (x3, y3) = points
    
    # Construct the system of equations
    A = np.array([
        [x1**2, x1, 1],
        [x2**2, x2, 1],
        [x3**2, x3, 1]
    ])
    B = np.array([y1, y2, y3])
    
    # Solve for a, b, c
    a, b, c = np.linalg.solve(A, B)
    
    return a, b, c

def endurance(power_required, battery_capacity, battery_config='2P6S', battery_efficiency=1):
    """
    Calculate the endurance of a battery-powered system.

    Parameters
    ----------
    power_required : [whatt]
    battery_capacity : [mAh]

    Returns
    -------
    float
        The endurance of the system in hours.
    """
    if battery_config == '2P6S':
        #convert to Ah
        battery_capacity = battery_capacity / 1000
        capacity = battery_capacity * 2 #[Ah]
        voltage = 3.7 * 6
        energy = capacity * voltage * battery_efficiency#[Wh]
        time = energy / power_required #[h]
        #convert to minutes
        time = time * 60
        return time
    
    if battery_config == '1P6S':
        #convert to Ah
        battery_capacity = battery_capacity / 1000
        capacity = battery_capacity
        voltage = 3.7 * 6
        energy = capacity * voltage * battery_efficiency
        time = energy / power_required #[h]
        #convert to minutes
        time = time * 60
        return time
    else:
        raise ValueError('Invalid battery configuration')     

 
def scene(bodies, colors, collocation_points, total_velocity_vectors, axial_velocity_vectors, tangential_velocity_vectors, extra_points=None, extra_lines=None):
        plotter = pv.Plotter()
        plotter.set_background("black") 

        if total_velocity_vectors is not None:
            poly_data = pv.PolyData(collocation_points)
            poly_data['vectors'] = total_velocity_vectors
            magnitude = np.linalg.norm(total_velocity_vectors, axis=1)
            poly_data['magnitude'] = magnitude
            glyphs = poly_data.glyph(orient='vectors', scale='magnitude', factor=0.004)
            plotter.add_mesh(glyphs, color="white")

        if axial_velocity_vectors is not None:
            poly_data = pv.PolyData(collocation_points) 
            poly_data['vectors'] = axial_velocity_vectors
            magnitude = np.linalg.norm(axial_velocity_vectors, axis=1)
            poly_data['magnitude'] = magnitude
            glyphs = poly_data.glyph(orient='vectors', scale='magnitude', factor=0.004)
            plotter.add_mesh(glyphs, color="yellow")

        if tangential_velocity_vectors is not None:
            poly_data = pv.PolyData(collocation_points) 
            poly_data['vectors'] = tangential_velocity_vectors
            magnitude = np.linalg.norm(tangential_velocity_vectors, axis=1)
            poly_data['magnitude'] = magnitude
            glyphs = poly_data.glyph(orient='vectors', scale='magnitude', factor=0.004)
            plotter.add_mesh(glyphs, color="red")

        


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

def twistGen(in_hub, in_tip, r, AoA):
    in_hub = in_hub + AoA
    in_tip = in_tip + AoA 

    if r[0] == 0:
        r[0] = 1e-6

    k = (in_tip - in_hub)/(1/(r[-1]) - 1/r[0])
    m = in_hub - k/(r[0])

    twist = k/r + m
    return twist


def contractionCoefficients(y_close, z_close, z_far, R):
    """
    Compute the contraction coefficients for a wake. 
    Assuming that contraction is represented by a parabola given by z = a*y^2 + b*y + c.
    We are looking for the coefficients a, b, and c based on the given points.
    
    Parameters:
        y_close (float): y value at z_close
        z_close (float): z value at y_close
        z_far (float): z value at y = 0.78R
        R (float): Reference radius of the rotor wake
        
    Returns:
        (a, b, c): Coefficients of the parabolic equation z = a*y^2 + b*y + c
    """
    # Define the system of equations
    A = np.array([[0, 0, 1],  # z = 0 at y = R
                  [z_close**2, z_close, 1],  # z_close at y_close
                  [z_far**2, z_far, 1]])  # z_far at y = 0.78R
                  
    b = np.array([R, y_close, 0.78*R])

    # Solve for coefficients
    coefs = np.linalg.solve(A, b)
    
    return coefs


def get_contraction_func(coefs, zw):
    y = lambda z: coefs[0]*z**2 + coefs[1]*z + coefs[2]
    z = np.linspace(0, zw[-1], len(zw))
    return y(z)

def contraction_sigmoid(z, contraction=0.78):
    # Sigmoid func: y(z) = 1 / (1 + exp(-k*(z))) +0.5 (so that it is 1 at z = 0)
    # at z_far, y(z) = contraction 
    # solve for k 
    z = abs(z)
    k = -np.log(1/(contraction-0.5) - 1) /z[-1]
    y = 1 / (1 + np.exp(-k*z)) + 0.5
    
    return y

