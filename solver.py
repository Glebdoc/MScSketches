from tqdm import tqdm
import numpy as np
import time, os
from geometry import Point, median_filter
import matplotlib.pyplot as plt
import pyvista as pv
import xfoilUtil as xf
import ctypes
from matplotlib.colors import BoundaryNorm
from mpl_toolkits.axes_grid1 import make_axes_locatable
from scipy.interpolate import make_interp_spline
import glob



def preload_airfoil_data(data_folder="./airfoil/data/numpy_smoothed"):
    """
    Preload all airfoil npz files into a dictionary.
    """
    file_paths = glob.glob(os.path.join(data_folder, "*.npz"))
    preloaded_data = {}
    
    for file_path in file_paths:
        data = np.load(file_path, allow_pickle=True)
        airfoil_name = os.path.splitext(os.path.basename(file_path))[0].replace("numpy_", "")
        preloaded_data[airfoil_name] = {key: data[key] for key in data.files}
    
    return preloaded_data

def conditional_median_filter_1d(signal, kernel_size=3, std_thresh=1.0):
    """Applies a conditional median filter to a 1D signal.

    Each point is replaced with the median of its local window
    only if it lies outside 1 standard deviation of the window.

    Args:
        signal (array-like): Input 1D array.
        kernel_size (int): Size of the sliding window (must be odd).
        std_thresh (float): Threshold in standard deviations.

    Returns:
        ndarray: Filtered 1D signal.
    """
    assert kernel_size % 2 == 1, "Kernel size must be odd."

    signal = np.asarray(signal)
    pad_size = kernel_size // 2
    padded = np.pad(signal, pad_size, mode='edge')

    from numpy.lib.stride_tricks import sliding_window_view
    windows = sliding_window_view(padded, kernel_size)

    medians = np.median(windows, axis=1)
    stds = np.std(windows, axis=1)

    deviations = np.abs(signal - medians)

    # Replace only if deviation exceeds threshold
    result = np.where(deviations > std_thresh * stds, medians, signal)

    return result

def computeNumberOfPoints(drone, type):
    """
    Compute the number of collocation points for the main and small propellers.

    """
    if type == 'quadcopter':
        main_NB = drone.props[0].NB
        main_n = drone.props[0].n
        npM = main_NB*(main_n-1)
        collocN = npM*4
        return npM, 0, collocN, main_NB, 0, main_n, 0

    else:
        main_NB = drone.main_prop.NB
        main_n = drone.main_prop.n
        npM = main_NB*(main_n-1) # number of collocation points for main prop

        if type == 'drone':
            small_NB = drone.small_props[0].NB
            small_n = drone.small_props[0].n
            npS = small_NB*(small_n-1) # number of collocation points for small prop
            collocN = (npM + npS*main_NB) # total number of collocation points
            return npM, npS, collocN, main_NB, small_NB, main_n, small_n
        
        else:
            collocN = npM
            return npM, 0, collocN, main_NB, 0, main_n, 0
   
def computeAzimuthAndOrigin(drone, npM, npS, main_NB, collocN, type):
    n_azimuth = np.zeros((collocN, 3))
    n_origin = np.zeros((collocN, 3))
    if type=='quadcopter':
        for i in range(4):
            n_azimuth [i*npM:(i+1)*npM] = np.tile(drone.props[i].azimuth, (npM, 1))
            n_origin [i*npM:(i+1)*npM] = np.array([drone.props[i].origin.x, 
                                                   drone.props[i].origin.y, 
                                                   drone.props[i].origin.z])
        return n_azimuth, n_origin
    else:
        
        n_azimuth[:npM] = np.tile(drone.main_prop.azimuth, (npM, 1))
        n_origin[:npM] = np.array([drone.main_prop.origin.x, drone.main_prop.origin.y, drone.main_prop.origin.z])

        if type=='helicopter':
            return n_azimuth, n_origin

        for i in range(main_NB):
            n_local_prop = drone.small_props[i].azimuth
            n_azimuth[npM + i * npS: npM + (i + 1) * npS] = np.tile(n_local_prop, (npS, 1))
            n_origin[npM + i * npS: npM + (i + 1) * npS] = np.array([drone.small_props[i].origin.x, 
                                                                    drone.small_props[i].origin.y, 
                                                                    drone.small_props[i].origin.z])
        return n_azimuth, n_origin

def computeCollocationPoints(drone, npM, npS, main_NB, small_NB, main_n, small_n, drone_type):
    
    if drone_type == 'quadcopter':
        colloc_points = np.array(drone.props[0].collocationPoints)
        print('colloc_points shape', colloc_points.shape)

        return 0
        return total_colloc_points
        # If
    main_collocPoints = np.array(drone.main_prop.collocationPoints)
    if drone_type=='helicopter':
            return mainCollocPoints
    else:
        for i in range(main_NB):
            mainCollocPoints[i * (main_n - 1):(i + 1) * (main_n - 1)] = main_colloc[i].T
            smallCollocPoints = np.zeros((npS*main_NB, 3))
            for i in range(main_NB):
                small_colloc = np.array(drone.small_props[i].collocationPoints)
                for j in range(small_NB):
                    smallCollocPoints[i * small_NB * (small_n - 1) + j * (small_n - 1): i * small_NB * (small_n - 1) + (j + 1) * (small_n - 1)] = small_colloc[j].T
            total_colloc_points = np.concatenate((mainCollocPoints, smallCollocPoints))
            return total_colloc_points

def computeChords(drone, npM, collocN, main_NB, small_NB, main_n, small_n, type):
    
    if type =='quadcopter':
        chords = drone.props[0].chord
        chords = 0.5*(chords[1:] + chords[:-1])
        return np.tile(chords, 4)
    else:
        chords = np.zeros((collocN, 1)) 
        ch_mid = 0.5*(drone.main_prop.chord[1:main_n] + drone.main_prop.chord[:main_n-1]) # Chords corresponding to the control points of the main prop
        chords[:npM, 0] = np.tile(ch_mid, (main_NB))
        if type == 'helicopter':
            return chords
        # Chords for small prop
        chords_small = drone.small_props[0].chord
        chords_small = 0.5*(chords_small[1:] + chords_small[:-1])
        chords[npM: , 0] = np.tile(chords_small, (main_NB*small_NB))
        return chords

def computeVrotational(drone, total_colloc_points, Omega, n_azimuth, n_origin, npM, npS, main_NB, type):
    if type =='quadcopter':
        v_rotational = np.cross(total_colloc_points-n_origin, n_azimuth*Omega[0])
        return v_rotational
    else:
        v_rotational_small = np.zeros((npS*main_NB, 3))
        azimuthVector = drone.main_prop.azimuth
        v_rotational_main = np.cross(total_colloc_points[:npM], azimuthVector*Omega[0])
        if type=='helicopter':
            return v_rotational_main

        # V rotational small props 
        v_rotational_small = np.cross(total_colloc_points[npM:] - n_origin[npM:], n_azimuth[npM:]*Omega[npM:]) + np.cross(total_colloc_points[npM:], azimuthVector*Omega[0]) 

        v_rotational = np.concatenate( (v_rotational_main, v_rotational_small))

        return v_rotational

def computeVaxial(v_rotational, n_azimuth, total_colloc_points, n_origin):

    # assume that induced are 0
    u = np.zeros((total_colloc_points.shape[0], 1))
    v = np.zeros((total_colloc_points.shape[0], 1))
    w = -np.ones((total_colloc_points.shape[0], 1))

    vel_total_x = u.flatten() + v_rotational[:, 0].flatten() 
    vel_total_y = v.flatten() + v_rotational[:, 1].flatten()
    vel_total_z = w.flatten() + v_rotational[:, 2].flatten()
    vel_total  = np.column_stack((vel_total_x, vel_total_y, vel_total_z))

    v_axial = np.sum(vel_total * n_azimuth, axis=1)
    return v_axial

def computeVtangential(v_rotational, n_azimuth, total_colloc_points, n_origin):
    vel_total = v_rotational
    tan_direction = np.cross(total_colloc_points - n_origin, n_azimuth)
    tan_direction = tan_direction / np.linalg.norm(tan_direction, axis=1)[:, np.newaxis]
    v_tangential = np.sum(vel_total * tan_direction, axis=1)
    return v_tangential

def computeInflowAndAlpha(v_tangential, v_axial, twist):
    inflowangle = np.arctan(-v_axial/v_tangential)
    alpha = twist.flatten() -  (inflowangle*180/np.pi).flatten()
    alpha = np.reshape(alpha, (-1, 1))
    return inflowangle, alpha

def plotInfluenceMatrices(u_influences, v_influences, w_influences):
    cmap = plt.get_cmap('binary')

    fig, ax = plt.subplots(1, 3, figsize=(15, 5))

    data_list = [u_influences, v_influences, w_influences]
    titles = ["u_influences", "v_influences", "w_influences"]

    for i in range(3):
        im = ax[i].imshow(data_list[i], cmap=cmap)
        ax[i].set_title(titles[i])
        ax[i].set_xticks(np.arange(data_list[i].shape[1] + 1) - 0.5, minor=True)
        ax[i].set_yticks(np.arange(data_list[i].shape[0] + 1) - 0.5, minor=True)
        ax[i].grid(which='minor', color='w', linestyle='-', linewidth=1)

        # Attach colorbar with same height
        divider = make_axes_locatable(ax[i])
        cax = divider.append_axes("right", size="5%", pad=0.05)
        plt.colorbar(im, cax=cax)

    plt.tight_layout()
    plt.show()

def computeForces(drone, v_tangential, v_axial, alpha, npM, npS, chords, main_NB, small_NB, main_n, small_n):
    # compute v_mag 
    v_mag = np.sqrt(v_axial**2 + v_tangential**2)

    # assume that 
    Cl = 2*np.pi*(alpha*np.pi/180)

    r_main =  drone.main_prop.r
    r_small = drone.small_props[0].r
    r_steps = (r_main[1:] - r_main[:-1])
    r_steps = np.tile(r_steps, (main_NB, 1))
    r_small_steps = (r_small[1:] - r_small[:-1])
    r_small_steps = np.tile(r_small_steps, (main_NB, 1))

    Lift = 0.5 * 1.225 * Cl[:npM].flatten() * (v_mag[:npM].flatten()**2) * chords[:npM].flatten() * r_steps.flatten()
    Cd = np.ones((npM, 1)) * 0.01

    Drag = 0.5 * 1.225 * Cd[:npM].flatten() * (v_mag[:npM].flatten()**2) * chords[:npM].flatten() * r_steps.flatten()

    return Cl, Lift, Drag

def smoothSpline(data, step=2):
    spline_cl = make_interp_spline(data[::step, 0], data[::step, 1], k=3)
    spline_cd = make_interp_spline(data[::step, 0], data[::step, 2], k=3)
    alpha_smooth = np.linspace(data[::step, 0].min(), data[::step, 0].max(), 300)
    cl_smooth = spline_cl(alpha_smooth)
    cd_smooth = spline_cd(alpha_smooth)
    return alpha_smooth, cl_smooth, cd_smooth

def solve(drone, updateConfig=True, case='main', save=False, savePath=False):

    preloaded_data = preload_airfoil_data()
    if drone.type == 'quadcopter':
        print('Solving for Quadcopter')
        type = 'quadcopter'
    elif (drone.type == 'helicopter'):
        print('Solving for Helicopter')
        type = 'helicopter'
    else:
        print('Solving for Drone with small propellers')
        type = 'drone'
    #helicopter = drone.helicopter
    ReInfluence = drone.reynolds
    if type=='drone':
        main_airfoil = drone.main_prop.airfoil
        small_airfoil = drone.small_airfoil
    else:
        main_airfoil = drone.airfoil

    npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(drone, type)

    v_axial = np.zeros((collocN, 1))
    v_tangential = np.zeros((collocN, 1))
    chords = np.zeros((collocN, 1)) 
    twist = np.zeros((collocN, 1))
    Cl, Cd = np.zeros((collocN, 1)), np.zeros((collocN, 1))
    Omega = np.zeros((collocN, 1))

    u_influences = np.zeros((collocN, collocN))
    v_influences = np.zeros((collocN, collocN))
    w_influences = np.zeros((collocN, collocN))

    Gammas = np.ones((collocN, 1))

    if ReInfluence is False:
        data_main = np.loadtxt('./A18_(smoothed)_Re0.191_M0.00_N9.0.txt', skiprows=12)
        data_small = np.loadtxt('./A18 (smoothed)_A18_(smoothed)_Re0.050_M0.00_N9.0.txt',skiprows=12)
        alpha_main, clP_main, cdP_main = smoothSpline(data_main, step=2)

        alpha_small, clP_small, cdP_small = smoothSpline(data_small, step=3)

    total_colloc_points = computeCollocationPoints(drone, npM, npS, main_NB, small_NB, main_n, small_n, type)
    table = np.array(drone.vortexTABLE)

    if os.name == 'nt':
        mylib = ctypes.CDLL("./mylib.dll")  
    else:
        mylib = ctypes.CDLL("./mylib.so")
    
    N = len(total_colloc_points)  # Number of collocation points
    T = len(table)  # Number of vortices
    core_size = ctypes.c_float(drone.core_size)

    collocationPoints_ptr = total_colloc_points.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    vortexTable_ptr = table.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    uInfluence_ptr = u_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    vInfluence_ptr = v_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    wInfluence_ptr = w_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

    # Call the C function
    res = mylib.computeInfluenceMatrices(N, T, collocationPoints_ptr, vortexTable_ptr, uInfluence_ptr, vInfluence_ptr, wInfluence_ptr, core_size)

    # n_azimuth, n_origin 
    n_azimuth, n_origin = computeAzimuthAndOrigin(drone, npM, npS, main_NB, collocN, type)

    # Omega
    if type == 'quadcopter':
        Omega = np.tile(drone.props[0].RPM*2*np.pi/60, (collocN, 1))
        twist = np.tile(drone.props[0].pitch, (collocN, 1))
    else:
        Omega[:npM] = drone.main_prop.RPM*2*np.pi/60
        twist[:npM, 0] = drone.main_prop.pitch
    # Chords
    chords = computeChords(drone, npM, collocN, main_NB, small_NB, main_n, small_n, type)


    if type=='drone':
        Omega[npM:] = drone.small_props[0].RPM*2*np.pi/60
        twist[npM:, 0] = np.tile(drone.small_props[0].pitch, (main_NB*small_NB))
    if type=='quadcopter':
        twist  = np.tile(twist[:npM, 0], (4, 1))
    # V rotational main prop 
    v_rotational = computeVrotational(drone, total_colloc_points, Omega, n_azimuth, n_origin, npM, npS, main_NB, type)
    # angle_small = drone.small_props[0].angles[1]
    # twist[npM:] = -np.rad2deg(angle_small)- twist[npM:]  # for small props, the twist is shifted by 90 degrees


    weight = 0.05
    err = 1.0
    iter = 0
    while (err > 1e-8 and iter<1_000):
        iter+=1
        if iter % 200 == 0:
            print(f'Weight: {weight} Iter:{iter} Error:{err:.4f}')
            weight -= weight*0.01
            weight = max(weight, 0.03)
        u = u_influences@Gammas
        v = v_influences@Gammas
        w = w_influences@Gammas

        vel_total_x = u.flatten() + v_rotational[:, 0].flatten() 
        vel_total_y = v.flatten() + v_rotational[:, 1].flatten()
        vel_total_z = w.flatten() + v_rotational[:, 2].flatten()
        #     vel_total_z += -U
        vel_total  = np.column_stack((vel_total_x, vel_total_y, vel_total_z))

        v_axial = np.sum(vel_total * n_azimuth, axis=1)

        tan_direction = np.cross(total_colloc_points - n_origin, n_azimuth)
        tan_direction = tan_direction / np.linalg.norm(tan_direction, axis=1)[:, np.newaxis]
        v_tangential = np.sum(vel_total * tan_direction, axis=1)
        v_mag = np.sqrt(v_axial**2 + v_tangential**2)
        Re = 1.225*v_mag.flatten()*chords.flatten()/1.81e-5
        inflowangle = np.arctan(-v_axial/v_tangential)

        if type=='drone':
            for i in range(main_NB):
                for j in range(small_NB):
                    start = npM +i*npS + j*(small_n-1)
                    end = start + (small_n-1)
                    inflowangle[start: end] = median_filter(inflowangle[start: end].copy(), kernel_size=5)

        # ################
        if type=='drone':
            # Adjust inflow angle for small props
            small_prop_angle = -drone.small_props[0].angles[1]
            inflowangle[npM:] = small_prop_angle - inflowangle[npM:] 
        
        # ################

        alpha = twist.flatten() -  (inflowangle*180/np.pi).flatten()
        # Try catching stall

        alpha[alpha>15] = 15
        alpha[alpha<-5] = -5
        #alpha[alpha>alpha_cl_max_main] = alpha_cl_max_main

        if type=='drone':
            alpha_small_temp = inflowangle[npM:]*180/np.pi - twist[npM:].flatten()# for small props, the alpha is shifted by 90 degrees
            #alpha_small_temp[alpha_small_temp>alpha_cl_max_small] = alpha_cl_max_small
            alpha_small_temp[alpha_small_temp>15] = 15
            alpha_small_temp[alpha_small_temp<-5] = -5
            alpha[npM:] = alpha_small_temp

        alpha = np.reshape(alpha, (-1, 1))

        # count number of alpha > 10 
        STALL_MAIN_UPPER = np.sum(alpha[:npM]>10)/npM
        STALL_MAIN_LOWER = np.sum(alpha[:npM]<-4)/npM

        if type=='drone':

            STALL_SMALL_UPPER = np.sum(alpha[npM:npM+npS]>10)/npS
            STALL_SMALL_LOWER = np.sum(alpha[npM:npM+npS]<-4)/npS
        else:
            STALL_SMALL_UPPER = 0
            STALL_SMALL_LOWER = 0

        STALL_TUPLE = (STALL_MAIN_UPPER, STALL_MAIN_LOWER, STALL_SMALL_UPPER, STALL_SMALL_LOWER)


        # stall_count = np.sum(alpha[:npM]>10) + np.sum(alpha[:npM]<-4) + np.sum(alpha[npM:npM+npS]>10) + np.sum(alpha[npM:npM+npS]<-4)
        # STALL_FACTOR = stall_count/(npM+npS)

        if ReInfluence:
            #airfoil_array = np.array([main_airfoil]*collocN)
            if type == 'drone':
                airfoil_array = np.array([main_airfoil]*npM + [small_airfoil]*(collocN - npM))
            else:
                airfoil_array = np.array([main_airfoil]*collocN)

            #cl, cd = xf.getPolar_batch(Re, alpha, airfoil_array, npz_dir="./airfoil/data/numpy")
            cl, cd = xf.getPolar_batch(Re, alpha, airfoil_array, preloaded_data)
            Cl[:] = cl.reshape(-1, 1)
            Cd[:] = cd.reshape(-1, 1)
            #Cl, Cd = db.get_cl_cd(main_airfoil, Re, alpha.flatten())
        else:
            #Cl = np.interp(alpha, alphaPolar, clPolar)
            Cl[:npM] = np.interp(alpha[:npM], alpha_main, clP_main)
            Cl[npM:] = np.interp(alpha[npM:], alpha_small, clP_small)

        



        Gammas_old = Gammas
        Gammas  = weight * Cl.flatten() * 0.5 * chords.flatten()* v_mag.flatten() + (1-weight)*Gammas_old.flatten()

        # reshape Gammas to (collocN, 1)
        Gammas[Gammas<-1] = -1
        Gammas[Gammas>3] = 3
        Gammas = np.reshape(Gammas, (collocN, 1))


        err = np.linalg.norm(Gammas - Gammas_old)
        if err > 1e15:
            Gammas[:, 0] = 1
            Gammas_old[:,0] = 0
            weight /= 1.1
            print('Error too high, resetting Gammas to zero')

    if iter == 1000:
        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        print('Max iterations reached')
        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    print(f'Iteration: {iter}, Error: {err:.4f}, Weight: {weight}')

    if not ReInfluence:
        #Cd = np.interp(alpha, alphaPolar, cdPolar)
        Cd[:npM] = np.interp(alpha[:npM], alpha_main, cdP_main)
        Cd[npM:] = np.interp(alpha[npM:], alpha_small, cdP_small)

    if type=='quadcopter':
        r_main = drone.props[0].r
    else:
        r_main =  drone.main_prop.r

    r_main = (r_main[1:] - r_main[:-1])
    r_main = np.tile(r_main, (main_NB))
    

    if type=='drone':
        r_small = drone.small_props[0].r
        r_small = (r_small[1:] - r_small[:-1])
        r_small= np.tile(r_small, (main_NB*small_NB, 1))

        r_steps = np.concatenate((r_main, r_small), axis=None)
    elif type=='quadcopter':
        r_steps = np.tile(r_main, 4)
    else:
        r_steps = r_main


    Lift = 0.5 * 1.225 * Cl.flatten() * (v_mag.flatten()**2) * chords.flatten() * r_steps.flatten()
    Drag = 0.5 * 1.225 * Cd.flatten() * (v_mag.flatten()**2) * chords.flatten() * r_steps.flatten()
    print('----------------------------------')


    LD= (Lift[:npM].sum())/(Drag[:npM].sum())

    Faxial = Lift * np.cos(inflowangle.flatten()) - Drag * np.sin(inflowangle.flatten())
    Faxial[:npM] = conditional_median_filter_1d(Faxial[:npM], kernel_size=5, std_thresh=1.0)
    Faxial[npM:] = Lift[npM:] * np.cos(-(inflowangle[npM:]-np.pi/2).flatten()) - Drag[npM:] * np.sin(-(inflowangle[npM:]-np.pi/2).flatten())
    Ftan = Lift * np.sin(inflowangle.flatten()) + Drag * np.cos(inflowangle.flatten())

    # r_main =  drone.main_prop.r
    # r_main = (r_main[1:] + r_main[:-1]) * 0.5
    # r_main = np.tile(r_main, (main_NB, 1))

    if type=='drone':
        r_small = drone.small_props[0].r
        r_small = (r_small[1:] + r_small[:-1]) * 0.5
        r_small = np.tile(r_small, (main_NB*small_NB, 1))
        r = np.concatenate((r_main, r_small), axis=None)

    else:
        r = r_main

    Torque = np.sum(Ftan[:npM] * r[:npM].flatten())

    print(f"Main RPM: {Omega[0,0]/0.1047:.2f} Torque:{Torque:.2f}")
    if type=='drone':
        angle_small = drone.small_props[0].angles[1]
        Thrust = Faxial[:npM].sum() + Faxial[npM:].sum()*np.sin(np.pi/2 + angle_small)
    else:
        Thrust = Faxial.sum()
    print('----------------------------------')

    computed_power = Torque*Omega[0,0]  
    
    # Compute small propeller thrust and torque
    if type=='drone':
        
        Torque_small = np.sum(Ftan[npM:] * r[npM:].flatten())
        Thrust_small = Faxial[npM:].sum() 
        created_moment = Thrust_small*np.cos(np.pi/2 + angle_small)*drone.main_prop.diameter/2
        power_required = Torque_small*drone.small_props[0].RPM*2*np.pi/60
        print(f"Small RPM: {drone.small_props[0].RPM:.2f} Torque_small:{Torque_small:.2f} Thrust_small: {Thrust_small:.2f} created_moment: {created_moment:.2f} power_required:{power_required:.2f}")
    else:
        Torque_small = 0
        Thrust_small = 0
        created_moment = 0
        power_required = 0

    print(f"Small engines Thrust {Thrust_small:.2f}")
    

    

    # Compute the induced power for the main rotor
    induced_power = (-v_axial[:npM].flatten() * Faxial[:npM].flatten()).sum()
    print(f"Induced power main rotor {induced_power:.2f}")    

    # # Compute the profile power for the main rotor
    #Cd0 = xf.getCd0_batch(Re, airfoil_array, npz_dir="./airfoil/data/numpy")
    if ReInfluence:
        _, Cd0 = xf.getPolar_batch(Re, np.zeros((alpha.shape[0], 1)), airfoil_array, preloaded_data)
    else:
        Cd0 = np.ones((collocN)) * 0.01
    NB_arr = np.ones((collocN))
    R_arr = np.zeros((collocN))
    R_arr[:npM] = drone.main_prop.diameter*0.5
    if type=='drone':
        R_arr[npM:] = drone.small_props[0].diameter*0.5
        NB_arr[npM:] = small_NB

    NB_arr[:npM] = main_NB
    

    solidity = NB_arr * chords.flatten() / (np.pi*R_arr*0.5)
    coefs = solidity*Cd0/(4*np.pi)

    profile_power_coefs = coefs.flatten()*v_mag.flatten()*r.flatten()  /  ((Omega.flatten()*R_arr.flatten())**3)

    #profile_power = np.sum(profile_power_coefs[:npM]*1.225*np.pi*(drone.main_prop.diameter*0.5)**2*(Omega[:npM].flatten()*R_arr[:npM].flatten())**3)
    profile_power = np.sum(0.5*1.225*(Omega[:npM].flatten()*r[:npM].flatten())**3 * Cd0[:npM].flatten()*r_steps[:npM].flatten()*chords[:npM].flatten())
    #profile_power_small = np.sum(profile_power_coefs[npM:])



    print(f"Profile power main rotor {profile_power:.2f}, main RPM: {drone.main_prop.RPM:.2f}")
    total_power = induced_power + profile_power
    print(f"Total Power: {total_power:.2f}")

    p_ideal = Thrust * np.sqrt(Thrust/(2*(drone.main_prop.diameter**2)*np.pi*1.225/4))
    FM = p_ideal/total_power
    L_unitspan = Lift.flatten() / (chords.flatten() * r_steps.flatten())

    print(f"Main Blade Thrust {Thrust:.2f} Main Blade Torque {Torque:.2f} Main Blade Power from Q {computed_power:.2f} T/Q: {Thrust/Torque:.2f} FM:{FM:.2f} Preq/Protor: {power_required/total_power:.2f}")
    print('----------------------------------')
    R = drone.main_prop.diameter*0.5
    Ct = Thrust/(1.225*np.pi*R**2*(R * drone.main_prop.RPM*2*np.pi/60)**2)
    drone.main_prop.Ct = Ct
    # save results to csv 
    if savePath is not None:
        #plotInfluenceMatrices(u_influences, v_influences, w_influences)
        table_final = table.copy()
        hsN = 0 
        for i in range(len(table)):
            if i==0:
                table[i][-1] = Gammas[0]
                continue
        if table[i][-1] != table[i-1][-1]:
            hsN += 1
            table[i][-1] = Gammas[hsN]
        else:
            table[i][-1] = Gammas[hsN]
            
        
        #np.savetxt(f'{savePath}/table_final.txt', table_final)

        misc  = np.zeros((collocN, 1))
        misc[0] = Thrust
        misc[1] = Torque
        misc[2] = LD
        misc[3] = FM
        misc[4] = power_required/total_power
        misc[5] = induced_power 
        misc[6] = profile_power
        misc[7] = induced_power + profile_power
        misc[8] = computed_power
        misc[9] = npM
        misc[11] = main_NB
        misc[13] = drone.main_prop.RPM
        if not helicopter:
            misc[10] = npS
            misc[12] = small_NB
            misc[14] = drone.small_props[0].RPM

        results = np.column_stack((r.flatten(), 
                                v_axial.flatten(), 
                                v_tangential.flatten(), 
                                inflowangle.flatten(), 
                                alpha.flatten(),
                                Faxial.flatten(), 
                                Ftan.flatten(),
                                Gammas.flatten(),
                                np.linalg.norm(v_rotational, axis=1),
                                np.sum(v_rotational * n_azimuth, axis=1), # axial rotational velocity
                                np.sum(v_rotational * tan_direction, axis=1), # tangential rotational velocity
                                u.flatten(),
                                v.flatten(),
                                w.flatten(),
                                Cl.flatten(),
                                Cd.flatten(),
                                Re.flatten(),
                                L_unitspan,
                                chords.flatten(),
                                twist.flatten(),
                                misc))

        header = "r, v_axial, v_tangential, inflowangle, alpha, Faxial, Ftan, Gammas, v_rot_norm, v_rot_a, v_rot_t, u, v, w, Cl, Cd,  misc"
        case = case.replace('.json', '')
        np.savetxt(f'./{savePath}/_res.csv', results, delimiter=',', header=header, comments='')

    np.savetxt('./auxx/v_axial.txt', v_axial)
    drone.main_prop.v_axial = v_axial[:main_n]

    return Gammas.flatten(), FM, created_moment, Torque, Thrust, power_required, induced_power, profile_power, v_axial, STALL_TUPLE