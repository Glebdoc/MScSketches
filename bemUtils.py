import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
import os, json
import time 
import ctypes
#from geometry import*
from matplotlib.colors import ListedColormap, BoundaryNorm
import seaborn as sns

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

 
def scene(bodies, colors, collocation_points, total_velocity_vectors, axial_velocity_vectors, tangential_velocity_vectors, extra_points=None, extra_lines=None, savefig = False):
        plotter = pv.Plotter()
        plotter.set_background("white") 

        camera = pv.Camera()
        camera.position = (-5, -5, 3)
        camera.focal_point = (0, 0, 0)
        camera.up = (0, 0, 1)
        plotter.camera = camera


        if total_velocity_vectors is not None:
            poly_data = pv.PolyData(collocation_points)
            poly_data['vectors'] = total_velocity_vectors
            magnitude = np.linalg.norm(total_velocity_vectors, axis=1)
            poly_data['magnitude'] = magnitude
            glyphs = poly_data.glyph(orient='vectors', scale='magnitude', factor=0.004)
            plotter.add_mesh(glyphs, color="white", opacity=0.5)

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
                 

            for i in range(body.vortexTABLE.shape[0]):
                all_points.append(body.vortexTABLE[i,:3].tolist())
                all_points.append(body.vortexTABLE[i,3:-2].tolist())
                all_lines.append([2, start_index, start_index + 1])
                start_index += 2
        
            poly_data = pv.PolyData(np.array(all_points))
            poly_data.lines = np.array(all_lines)
            plotter.add_mesh(poly_data, color=colors[j], line_width=2, opacity=0.5)
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
        # if display:
        #     plotter.save_graphic("scene.png")

def twistGen(in_hub, in_tip, r, AoA):
    in_hub = in_hub + AoA
    in_tip = in_tip + AoA 

    if r[0] == 0:
        r[0] = 1e-6

    k = (in_tip - in_hub)/(1/(r[-1]) - 1/r[0])
    m = in_hub - k/(r[0])

    twist = k/r + m
    return twist

def computeVelocityField(data, points, plane='YZ', shift=0, discretization=50, plotting=False):
    vortexTable = data

    # --- Decide the sampling mode ---
    if points is not None:
        points = np.asarray(points)
        N_points = len(points)
        use_structured = False
    else:
        use_structured = True
        if plane == 'YZ':
            y_range = np.linspace(-1.5, 1.5, discretization)
            z_range = np.linspace(-2.0, 0.5, discretization)
            Y, Z = np.meshgrid(y_range, z_range)
            N_points = Y.size
            points = np.column_stack((np.full(N_points, shift), Y.ravel(), Z.ravel()))
        elif plane == 'XZ':
            x_range = np.linspace(-1.5, 1.5, discretization)
            z_range = np.linspace(-2.0, 0.5, discretization)
            X, Z = np.meshgrid(x_range, z_range)
            N_points = X.size
            points = np.column_stack((X.ravel(), np.full(N_points, shift), Z.ravel()))
        elif plane == 'XY':
            x_range = np.linspace(-0.5, 1.0, discretization)
            y_range = np.linspace(0.5, 1.3, discretization)
            X, Y = np.meshgrid(x_range, y_range)
            N_points = X.size
            points = np.column_stack((X.ravel(), Y.ravel(), np.full(N_points, shift)))
        else:
            raise ValueError("plane must be 'YZ', 'XZ', or 'XY'")

    # --- Allocate and call C library ---
    u = np.zeros(N_points)
    v = np.zeros(N_points)
    w = np.zeros(N_points)

    if os.name == 'nt':
        mylib = ctypes.CDLL("./mylib.dll")
    else:
        mylib = ctypes.CDLL("./mylib.so")

    T = len(vortexTable)
    table_ptr  = vortexTable.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    points_ptr = points.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    u_ptr = u.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    v_ptr = v.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    w_ptr = w.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

    start = time.time()
    mylib.computeVelocityField(N_points, T, points_ptr, table_ptr, u_ptr, v_ptr, w_ptr)
    print("C Fields function execution time:", time.time() - start, "seconds")

    # --- Only build structured arrays when we actually used a structured grid ---
    if use_structured:
        mag = np.sqrt(u**2 + v**2 + w**2)
        mag = np.clip(mag, 0, 25).reshape((discretization, discretization))
        # (any PyVista grid construction would go here)
        # Return both flat and structured if you want:
        return u, v, w, mag
    else:
        return u, v, w


    # if plotting:
    #     pl = pv.Plotter(shape=(2, 2))
    #     pl.subplot(0, 0)
    #     pl.add_mesh(grid, scalars='u', cmap='jet')
    #     pl.add_text("u", color='k')
    #     pl.subplot(0, 1)
    #     pl.add_mesh(grid.copy(),  scalars='v', cmap='jet')
    #     pl.add_text("v", color='k')
    #     pl.subplot(1, 0)
    #     pl.add_mesh(grid.copy(),  scalars='w', cmap='jet')
    #     pl.add_text("w", color='k')
    #     pl.subplot(1, 1)
    #     pl.add_mesh(grid.copy(),  scalars='magnitude', cmap=discrete_cmap, clim=[min_mag, max_mag], show_scalar_bar=True)
    #     pl.add_text("magnitude", color='k')
    #     pl.link_views()
    #     pl.camera_position = 'iso'
    #     pl.background_color = 'white'
    #     pl.show()
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


# def cRotate(horse, R):
#     points = []
#     for i in range(len(horse.leftset)):
#         points.append(horse.leftset[i].p1.x, horse.leftset[i].p1.y, horse.leftset[i].p1.z)
#         points.append(horse.leftset[i].p2.x, horse.leftset[i].p2.y, horse.leftset[i].p2.z)
#         points.append(horse.rightset[i].p1.x, horse.rightset[i].p1.y, horse.rightset[i].p1.z)
#         points.append(horse.rightset[i].p2.x, horse.rightset[i].p2.y, horse.rightset[i].p2.z)
    
#     points = np.array(points)

#     points_ptr = points.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
#     N = len(points)
#     points_rotated = np.zeros((N, 3))
#     points_rotated_ptr = points_rotated.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
#     R_ptr = R.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

#     mylib = ctypes.CDLL("./rotate.so")

#     mylib.rotate(points_ptr, points_rotated_ptr, R_ptr, N)

#     # assemble the horseshoe again 

#     leftset = []
#     rightset = []

#     for i in range((N-2)):
        

def set_bw_design():
    """
    Configure matplotlib + seaborn for consistent black & white (grayscale) plotting style.
    """
    # Seaborn & matplotlib styles
    sns.set_style("whitegrid")
    plt.style.use("seaborn-v0_8")
    sns.set_palette("Greys_r")  # grayscale palette
    
    # Global rcParams for consistent looks
    plt.rcParams.update({
        "figure.figsize": (6, 4),
        "figure.dpi": 200,
        "axes.edgecolor": "black",
        "axes.labelweight": "bold",
        "axes.grid": True,
        "grid.alpha": 0.4,
        "grid.color": "black",
        "grid.linewidth": 0.5,
        "legend.frameon": True,
        "legend.fancybox": False,
        "legend.shadow": False,  # cleaner for B&W
        "lines.linewidth": 1.5,
        "lines.markersize": 3,
    })
    
    # Line styles / markers / colors for B&W distinction
    design = {
        "line_styles": ['-', '--', '-.'],
        "markers": ['o', 's', '^'],
        "colors": ['black', '0.4', '0.7']  # black, dark gray, light gray
    }
    return design