from tqdm import tqdm
import numpy as np
import time, os
from geometry import Point
import matplotlib.pyplot as plt
import pyvista as pv
import xfoilUtil as xf
import ctypes

def computeNumberOfPoints(drone):
    """
    Compute the number of collocation points for the main and small propellers.
    """
    main_NB = drone.main_prop.NB
    small_NB = drone.small_props[0].NB
    main_n = drone.main_prop.n
    small_n = drone.small_props[0].n

    npM = main_NB*(main_n-1) # number of collocation points for main prop
    npS = small_NB*(small_n-1) # number of collocation points for small prop
    
    collocN = (npM + npS*main_NB) # total number of collocation points
    
    return npM, npS, collocN, main_NB, small_NB, main_n, small_n

def computeAzimuthAndOrigin(drone, npM, npS, main_NB, collocN):
    n_azimuth = np.zeros((collocN, 3))
    n_origin = np.zeros((collocN, 3))
    n_azimuth[:npM] = np.tile(drone.main_prop.azimuth, (npM, 1))
    for i in range(main_NB):
        n_local_prop = drone.small_props[i].azimuth
        n_azimuth[npM + i * npS: npM + (i + 1) * npS] = np.tile(n_local_prop, (npS, 1))

        n_origin[:npM] = np.array([drone.main_prop.origin.x, drone.main_prop.origin.y, drone.main_prop.origin.z])
        n_origin[npM + i * npS: npM + (i + 1) * npS] = np.array([drone.small_props[i].origin.x, drone.small_props[i].origin.y, drone.small_props[i].origin.z])

    return n_azimuth, n_origin

def computeCollocationPoints(drone, npM, npS, main_NB, small_NB, main_n, small_n):
    mainCollocPoints = np.zeros((npM, 3))
    smallCollocPoints = np.zeros((npS*main_NB, 3))
    main_colloc = np.array(drone.main_prop.collocationPoints)
  
    for i in range(main_NB):
        mainCollocPoints[i * (main_n - 1):(i + 1) * (main_n - 1)] = main_colloc[i].T
    
    for i in range(main_NB):
        small_colloc = np.array(drone.small_props[i].collocationPoints)
        for j in range(small_NB):
            smallCollocPoints[i * small_NB * (small_n - 1) + j * (small_n - 1): i * small_NB * (small_n - 1) + (j + 1) * (small_n - 1)] = small_colloc[j].T
    total_colloc_points = np.concatenate((mainCollocPoints, smallCollocPoints))

    return total_colloc_points

def computeChords(drone, npM, collocN, main_NB, small_NB, main_n, small_n):
    chords = np.zeros((collocN, 1)) 
    ch_mid = 0.5*(drone.main_prop.chord[1:main_n] + drone.main_prop.chord[:main_n-1]) # Chords corresponding to the control points of the main prop
    chords[:npM, 0] = np.tile(ch_mid, (main_NB))

    # Chords for small prop
    chords_small = drone.small_props[0].chord
    chords_small = 0.5*(chords_small[1:] + chords_small[:-1])
    chords[npM: , 0] = np.tile(chords_small, (main_NB*small_NB))
    return chords

def computeVrotational(drone, total_colloc_points, Omega, n_azimuth, n_origin, npM, npS, main_NB):
    v_rotational_small = np.zeros((npS*main_NB, 3))
    azimuthVector = drone.main_prop.azimuth
    v_rotational_main = np.cross(total_colloc_points[:npM], azimuthVector*Omega[0])

    # V rotational small props 
    v_rotational_small = np.cross(total_colloc_points[npM:] - n_origin[npM:], n_azimuth[npM:]*Omega[npM:]) + np.cross(total_colloc_points[npM:], azimuthVector*Omega[0])

    v_rotational = np.concatenate( (v_rotational_main, v_rotational_small))

    # collocs are fine


   # return v_rotational, v_rotational_main, v_rotational_small
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


    # Lift = 0.5 * 1.225 * Cl[:main_n-1].flatten() * (v_mag[:main_n-1].flatten()**2) * chords[:main_n-1].flatten() * r_steps.flatten()
    # Drag = 0.5 * 1.225 * Cd[:main_n-1].flatten() * (v_mag[:main_n-1].flatten()**2) * chords[:main_n-1].flatten() * r_steps.flatten()
    # LD= (Lift.sum())/(Drag.sum())

    # Faxial = Lift * np.cos(inflowangle[:main_n-1].flatten()) - Drag * np.sin(inflowangle[:main_n-1].flatten())
    # Ftan = Lift * np.sin(inflowangle[:main_n-1].flatten()) + Drag * np.cos(inflowangle[:main_n-1].flatten())



def solve(drone, updateConfig=True, case='main', save=False):

    updateReynolds = False  
    ReInfluence = drone.reynolds
    wind = drone.wind
    main_airfoil = drone.main_prop.airfoil

    npM, npS, collocN, main_NB, small_NB, main_n, small_n = computeNumberOfPoints(drone)
    
    v_axial = np.zeros((collocN, 1))
    v_tangential = np.zeros((collocN, 1))
    chords = np.zeros((collocN, 1)) 
    twist = np.zeros((collocN, 1))
    v_rotational_main = np.zeros((npM, 3))
    v_rotational_small = np.zeros((npS*main_NB, 3))
    
    Omega = np.zeros((collocN, 1))

    u_influences = np.zeros((collocN, collocN))
    v_influences = np.zeros((collocN, collocN))
    w_influences = np.zeros((collocN, collocN))

    Gammas = np.ones((collocN, 1))

    mainCollocPoints = np.zeros((npM, 3))
    smallCollocPoints = np.zeros((npS*main_NB, 3))

    if ReInfluence:
        db = xf.PolarDatabase("./airfoil/data")

    else:
        data = np.loadtxt('./A18.txt', skiprows=12)
        alphaPolar = data[:, 0]
        clPolar = data[:, 1]
        cdPolar = data[:, 2]  


    total_colloc_points = computeCollocationPoints(drone, npM, npS, main_NB, small_NB, main_n, small_n)

    table = np.array(drone.vortexTABLE)

    if updateConfig:
        print('Updating config')
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

        np.savetxt('./auxx/u_influences.txt', u_influences)
        np.savetxt('./auxx/v_influences.txt', v_influences)
        np.savetxt('./auxx/w_influences.txt', w_influences)
    else:
        u_influences = np.loadtxt('./auxx/u_influences.txt')
        v_influences = np.loadtxt('./auxx/v_influences.txt')
        w_influences = np.loadtxt('./auxx/w_influences.txt')

    fig, ax = plt.subplots(1, 3, figsize=(15, 5))

    colormap = plt.get_cmap('viridis')

    # u_influences
    im0 = ax[0].imshow(u_influences, cmap=colormap)
    ax[0].set_title("u_influences")
    ax[0].set_xticks(np.arange(u_influences.shape[1] + 1) - 0.5, minor=True)
    ax[0].set_yticks(np.arange(u_influences.shape[0] + 1) - 0.5, minor=True)
    ax[0].grid(which='minor', color='w', linestyle='-', linewidth=1)
    fig.colorbar(im0, ax=ax[0])

    # v_influences
    im1 = ax[1].imshow(v_influences, cmap=colormap)
    ax[1].set_title("v_influences")
    ax[1].set_xticks(np.arange(v_influences.shape[1] + 1) - 0.5, minor=True)
    ax[1].set_yticks(np.arange(v_influences.shape[0] + 1) - 0.5, minor=True)
    ax[1].grid(which='minor', color='w', linestyle='-', linewidth=1)
    fig.colorbar(im1, ax=ax[1])

    # w_influences
    im2 = ax[2].imshow(w_influences, cmap=colormap)
    ax[2].set_title("w_influences")
    ax[2].set_xticks(np.arange(w_influences.shape[1] + 1) - 0.5, minor=True)
    ax[2].set_yticks(np.arange(w_influences.shape[0] + 1) - 0.5, minor=True)
    ax[2].grid(which='minor', color='w', linestyle='-', linewidth=1)
    fig.colorbar(im2, ax=ax[2])

    plt.tight_layout()
    plt.show()

    # add color bar


    # # # Optional: rotate for better visibility
    # cbar = plt.colorbar(im, ax=ax[0, 2])



    # # Grid lines to simulate borders
    # ax.set_xticks(np.arange(w_influences.shape[1]+1) - 0.5, minor=True)
    # ax.set_yticks(np.arange(w_influences.shape[0]+1) - 0.5, minor=True)
    # ax.grid(which='minor', color='w', linestyle='-', linewidth=1)

    # # Major ticks centered in cells
    # ax.set_xticks(np.arange(w_influences.shape[1]))
    # ax.set_yticks(np.arange(w_influences.shape[0]))
    # ax.set_xticklabels(np.arange(w_influences.shape[1]))
    # ax.set_yticklabels(np.arange(w_influences.shape[0]))

    # # Optional: rotate for better visibility
    # plt.setp(ax.get_xticklabels(), rotation=0, ha="center")

    # # Remove minor tick marks (we only want the grid)
    # ax.tick_params(which='minor', bottom=False, left=False)

    # # Add color bar
    # cbar = plt.colorbar(im, ax=ax)
    # cbar.set_label('Influence Coefficient', rotation=270, labelpad=15)

    # plt.title("w_influences Heatmap with Borders and Indices")
    # plt.show()

    # n_azimuth, n_origin 
    n_azimuth, n_origin = computeAzimuthAndOrigin(drone, npM, npS, main_NB, collocN)

    # Omega
    Omega[:npM] = drone.main_prop.RPM*2*np.pi/60
    Omega[npM:] = drone.small_props[0].RPM*2*np.pi/60

    # Chords
    chords = computeChords(drone, npM, collocN, main_NB, small_NB, main_n, small_n)

    # Twist
    twist[:npM, 0] = drone.main_prop.pitch
    twist[npM:, 0] = np.tile(drone.small_props[0].pitch, (main_NB*small_NB))
    
    # V rotational main prop 
    v_rotational = computeVrotational(drone, total_colloc_points, Omega, n_azimuth, n_origin, npM, npS, main_NB)


    weight = 0.2
    err = 1.0
    iter = 0
    while (err > 1e-8 and iter<1_000):
        iter+=1
        if iter % 200 == 0:
            print('Weight:', weight, 'Iter:', iter, 'Error:', err)
            # weight -= weight*0.1
            # weight = max(weight, 0.1)
        u = u_influences@Gammas
        v = v_influences@Gammas
        w = w_influences@Gammas

        vel_total_x = u.flatten() + v_rotational[:, 0].flatten() 
        vel_total_y = v.flatten() + v_rotational[:, 1].flatten()
        vel_total_z = w.flatten() + v_rotational[:, 2].flatten()
        vel_total  = np.column_stack((vel_total_x, vel_total_y, vel_total_z))

        v_axial = np.sum(vel_total * n_azimuth, axis=1)
        tan_direction = np.cross(total_colloc_points - n_origin, n_azimuth)
        tan_direction = tan_direction / np.linalg.norm(tan_direction, axis=1)[:, np.newaxis]
        v_tangential = np.sum(vel_total * tan_direction, axis=1)

        v_mag = np.sqrt(v_axial**2 + v_tangential**2)
        Re = 1.225*v_mag.flatten()*chords.flatten()/1.81e-5


        inflowangle = np.arctan(-v_axial/v_tangential)
        alpha = twist.flatten() -  (inflowangle*180/np.pi).flatten()
        #alpha[npM:] = 5
        alpha = np.reshape(alpha, (-1, 1))

        if ReInfluence:

            Cl, Cd = db.get_cl_cd(main_airfoil, Re, alpha.flatten())
        else:
            Cl = np.interp(alpha, alphaPolar, clPolar)

        Gammas_old = Gammas
        Gammas  = weight * Cl.flatten() * 0.5 * chords.flatten()* v_mag.flatten() + (1-weight)*Gammas_old.flatten()
        Gammas = np.reshape(Gammas, (collocN, 1))

        err = np.linalg.norm(Gammas - Gammas_old)
        #print(f'Iteration: {iter}, Error: {err}')
    if iter == 1000:
        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        print('Max iterations reached')
        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    print(f'Iteration: {iter}, Error: {err}, Weight: {weight}')


    mean_axial_main = v_axial[:main_NB*(main_n-1)].mean()
    mean_axial_small = v_axial[main_NB*(main_n-1):].mean()


    if not ReInfluence:
        Cd = np.interp(alpha, alphaPolar, cdPolar)

    r_main =  drone.main_prop.r
    r_small = drone.small_props[0].r

    # steps (width of an element)
    r_main = (r_main[1:] - r_main[:-1])
    r_main = np.tile(r_main, (main_NB))

    r_small = (r_small[1:] - r_small[:-1])
    r_small= np.tile(r_small, (main_NB*small_NB, 1))

    r_steps = np.concatenate((r_main, r_small), axis=None)

    Lift = 0.5 * 1.225 * Cl.flatten() * (v_mag.flatten()**2) * chords.flatten() * r_steps.flatten()

    Drag = 0.5 * 1.225 * Cd.flatten() * (v_mag.flatten()**2) * chords.flatten() * r_steps.flatten()
    print('----------------------------------')
    # print('Lift small:', Lift[npM:npM + npS])
    # print('Drag small:', Drag[npM:npM + npS])

    LD= (Lift[:npM].sum())/(Drag[:npM].sum())



    Faxial = Lift * np.cos(inflowangle.flatten()) - Drag * np.sin(inflowangle.flatten())
    Faxial[npM:] = Lift[npM:] * np.cos(-(inflowangle[npM:]-np.pi/2).flatten()) - Drag[npM:] * np.sin(-(inflowangle[npM:]-np.pi/2).flatten())
    Ftan = Lift * np.sin(inflowangle.flatten()) + Drag * np.cos(inflowangle.flatten())

    # print('V_axial small:', v_axial[npM:npM + npS])
    # print('V_tangential small:', v_tangential[npM:npM + npS])
    # print('V_rotational small:', v_rotational[npM:npM + npS, :])
    # print('inflowangle small:', inflowangle[npM:npM + npS]*180/np.pi)
    # print('alpha small:', alpha[npM:npM + npS])
    # print('Faxial small:', Faxial[npM:npM + npS])
    # print('Ftan small:', Ftan[npM:npM + npS])
    # print('----------------------------------')


    r_main =  drone.main_prop.r
    r_small = drone.small_props[0].r
    r_main = (r_main[1:] + r_main[:-1]) * 0.5

    r_main = np.tile(r_main, (main_NB, 1))
    r_small = (r_small[1:] + r_small[:-1]) * 0.5
    r_small = np.tile(r_small, (main_NB*small_NB, 1))
    r = np.concatenate((r_main, r_small), axis=None)

    Torque = np.sum(Ftan[:npM] * r[:npM].flatten())
    Thrust = Faxial[:npM].sum() 
    print('----------------------------------')

    computed_power = Torque*drone.main_prop.RPM*2*np.pi/60
    
    # Compute small propeller thrust and torque
    Torque_small = np.sum(Ftan[npM:] * r[npM:].flatten())
    Thrust_small = Faxial[npM:].sum() 

    print("Small engines Thrust", Thrust_small)

    created_moment = Thrust_small*drone.main_prop.diameter/2
    power_required = Torque_small*drone.small_props[0].RPM*2*np.pi/60
    print("computing power required, used RPM: ", drone.small_props[0].RPM)

    

    # Compute the induced power for the main rotor
    induced_power = (-v_axial[:npM].flatten() * Faxial[:npM].flatten()).sum()
    print("Induced power main rotor", induced_power)    

    # Compute the profile power for the main rotor
    cd0 = 0.02
    solidity = drone.main_prop.NB * drone.main_prop.chord[:main_n-1].flatten() / (np.pi*drone.main_prop.diameter/2)
    omega = drone.main_prop.RPM * 2 * np.pi / 60
    R = drone.main_prop.diameter/2
    coefs = solidity*cd0/(4*np.pi)

    profile_power_coefs = coefs*((v_mag[:main_n-1].flatten()*r[:main_n-1]/(omega*R))**3)
    profile_power = profile_power_coefs*1.225*np.pi*(drone.main_prop.diameter/2)**2*(omega*R)**3
    profile_power = profile_power.sum()
    print("Profile power main rotor", profile_power)
    total_power = induced_power + profile_power

    p_ideal = Thrust * np.sqrt(Thrust/(2*(drone.main_prop.diameter**2)*np.pi*1.225/4))
    FM = p_ideal/total_power

    print(f"Main Blade Thrust {Thrust:.2f} Main Blade Torque {Torque:.2f} Main Blade Power {computed_power:.2f} T/Q: {Thrust/Torque:.2f} FM:{FM:.2f} Preq/Protor: {power_required/total_power:.2f}")
    print('----------------------------------')
    
    # save results to csv 
    if save:
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
            
        np.savetxt('table_final.txt', table_final)

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
        misc[10] = npS
        misc[11] = main_NB
        misc[12] = small_NB
        misc[13] = drone.main_prop.RPM
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
                                misc))

        header = "r, v_axial, v_tangential, inflowangle, alpha, Faxial, Ftan, Gammas, v_rot_norm, v_rot_a, v_rot_t, u, v, w, Cl, Cd,  misc"
        case = case.replace('.json', '')
        np.savetxt(f'./results/{case}_res.csv', results, delimiter=',', header=header, comments='')

    
    return abs(mean_axial_main), abs(mean_axial_small), None, Gammas, FM, created_moment, Torque, Thrust, power_required, induced_power, profile_power