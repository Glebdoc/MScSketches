import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
import os, json
import time 
import ctypes
from geometry import*
from matplotlib.colors import ListedColormap, BoundaryNorm

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

def computeVelocityField(plane='YZ', shift=0, discretization=50, plotting=False):
    vortexTable = np.loadtxt('table_final.txt')

    if plane == 'YZ':
        y_range = np.linspace(-1.5, 1.5, discretization)
        z_range = np.linspace(-5, .5, discretization)

        Y, Z = np.meshgrid(y_range, z_range)
        N_points = len(Y.flatten())
        points = np.column_stack((np.ones(N_points)*shift, Y.flatten(), Z.flatten()))

    elif plane == 'XZ':
        x_range = np.linspace(-1, 1, discretization)
        z_range = np.linspace(-.5, .5, discretization)

        X, Z = np.meshgrid(x_range, z_range)
        N_points = len(X.flatten())
        points = np.column_stack((X.flatten(), np.ones(N_points)*shift, Z.flatten()))

    elif plane == 'XY':
        x_range = np.linspace(-1., 1., discretization)
        y_range = np.linspace(-1., 1., discretization)

        X, Y = np.meshgrid(x_range, y_range)
        N_points = len(X.flatten())
        points = np.column_stack((X.flatten(), Y.flatten(), np.ones(N_points)*shift))

    u = np.zeros(N_points)
    v = np.zeros(N_points)
    w = np.zeros(N_points)

    if os.name == 'nt':
            mylib = ctypes.CDLL("./mylib.dll")  
    else:
        # Assuming Linux or MacOS
        mylib = ctypes.CDLL("./mylib.so")

    T = len(vortexTable) 
    table_ptr = vortexTable.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    points_ptr = points.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    u_ptr = u.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    v_ptr = v.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    w_ptr = w.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

    start = time.time()
    mylib.computeVelocityField(N_points, T, points_ptr, table_ptr, u_ptr, v_ptr, w_ptr)

    time2 = time.time() - start
    print("C Fields function execution time:", time2, "seconds")


    magnitude = np.sqrt(u**2 + v**2 + w**2)
    magnitude[magnitude > 25] = 25  # Cap the magnitude at 25 m/s
    magnitude = magnitude.reshape((discretization, discretization))

    # Create PyVista mesh for visualization
    mesh = pv.PolyData(points)  # Create the points from the grid
    mesh['u'] = u  # Add u velocity component to mesh (use dictionary assignment)
    mesh['v'] = v  # Add v velocity component to mesh
    mesh['w'] = w  # Add w velocity component to mesh
    mesh['magnitude'] = magnitude.flatten()  # Add magnitude of velocity to mesh

    # Set bounds for colormap
    min_mag = 0
    max_mag = 25
    n_bands = 64  
    cmap = plt.get_cmap("jet", n_bands)  # Creates discrete colors

    grid = pv.StructuredGrid()
    grid.points = points  # Keep it (N,3), do NOT reshape!
    grid.dimensions = [discretization, discretization, 1]  # Needed for 2D slices
    grid["magnitude"] = magnitude.flatten()
    grid['u'] = u
    grid['v'] = v
    grid['w'] = w

    discrete_cmap = ListedColormap(cmap(np.linspace(0, 1, n_bands)))  # Apply discrete colors

    if plotting:
        pl = pv.Plotter(shape=(2, 2))
        pl.subplot(0, 0)
        pl.add_mesh(grid, scalars='u', cmap='jet')
        pl.add_text("u", color='k')
        pl.subplot(0, 1)
        pl.add_mesh(grid.copy(),  scalars='v', cmap='jet')
        pl.add_text("v", color='k')
        pl.subplot(1, 0)
        pl.add_mesh(grid.copy(),  scalars='w', cmap='jet')
        pl.add_text("w", color='k')
        pl.subplot(1, 1)
        pl.add_mesh(grid.copy(),  scalars='magnitude', cmap=discrete_cmap, clim=[min_mag, max_mag], show_scalar_bar=True)
        pl.add_text("magnitude", color='k')
        pl.link_views()
        pl.camera_position = 'iso'
        pl.background_color = 'white'
        pl.show()

    return u, v, w

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

