from tqdm import tqdm
import numpy as np
import time, os
from geometry import Point
import matplotlib.pyplot as plt
import pyvista as pv
import xfoilUtil as xf
import ctypes


def solve(drone, updateConfig=True, case='main', save=False):

    updateReynolds = False  
    ReInfluence = drone.reynolds
    wind = drone.wind
    main_airfoil = drone.main_prop.airfoil
    main_NB = drone.main_prop.NB
    small_NB = drone.small_props[0].NB
    main_n = drone.main_prop.n
    small_n = drone.small_props[0].n
    
    collocN = (main_NB * (main_n - 1) + main_NB * small_NB * (small_n - 1))

    u_influences = np.zeros((collocN, collocN))
    v_influences = np.zeros((collocN, collocN))
    w_influences = np.zeros((collocN, collocN))

    Gammas = np.ones((collocN, 1))

    mainCollocPoints = np.zeros((main_NB * (main_n - 1), 3))
    smallCollocPoints = np.zeros((main_NB * small_NB * (small_n - 1), 3))

    if ReInfluence:
        db = xf.PolarDatabase("./airfoil/data")

    else:
        data = np.loadtxt('./A18.txt', skiprows=12)
        alphaPolar = data[:, 0]
        clPolar = data[:, 1]
        cdPolar = data[:, 2]  


    main_colloc = np.array(drone.main_prop.collocationPoints)
  
    for i in range(main_NB):
        mainCollocPoints[i * (main_n - 1):(i + 1) * (main_n - 1)] = main_colloc[i].T

    for i in range(main_NB):
        small_colloc = np.array(drone.small_props[i].collocationPoints)
        for j in range(small_NB):
            smallCollocPoints[i * small_NB * (small_n - 1) + j * (small_n - 1): i * small_NB * (small_n - 1) + (j + 1) * (small_n - 1)] = small_colloc[j].T

    total_colloc_points = np.concatenate((mainCollocPoints, smallCollocPoints))
    table = np.array(drone.vortexTABLE)

    if updateConfig:
        # check if windows 
        if os.name == 'nt':
            mylib = ctypes.CDLL("./mylib.dll")  
        else:
            # Assuming Linux or MacOS
            mylib = ctypes.CDLL("./mylib.so")
        
        N = len(total_colloc_points)  # Number of collocation points
        T = len(table)  # Number of vortices

        collocationPoints_ptr = total_colloc_points.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        vortexTable_ptr = table.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        uInfluence_ptr = u_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        vInfluence_ptr = v_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        wInfluence_ptr = w_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        start = time.time()
        res = mylib.computeInfluenceMatrices(N, T, collocationPoints_ptr, vortexTable_ptr, uInfluence_ptr, vInfluence_ptr, wInfluence_ptr)
        time2 = time.time() - start
        print("C function execution time:", time2, "seconds", res)
        np.savetxt('./auxx/u_influences.txt', u_influences)
        np.savetxt('./auxx/v_influences.txt', v_influences)
        np.savetxt('./auxx/w_influences.txt', w_influences)
    else:
        u_influences = np.loadtxt('./auxx/u_influences.txt')
        v_influences = np.loadtxt('./auxx/v_influences.txt')
        w_influences = np.loadtxt('./auxx/w_influences.txt')
    

    v_axial = np.zeros((collocN, 1))
    v_tangential = np.zeros((collocN, 1))
    chords = np.zeros((collocN, 1)) 
    twist = np.zeros((collocN, 1))
    twist[:main_NB*(main_n-1), 0] = drone.main_prop.pitch
    chords[:main_NB* (main_n - 1), 0] = drone.main_prop.chord

    main_Omega = drone.main_prop.RPM * 2 * np.pi / 60
    v_rotational_main = np.zeros((main_NB* (main_n - 1), 3))
    azimuthVector = drone.main_prop.azimuth
    for i in range(main_NB* (main_n - 1)):
        v_rotational_main[i] = np.cross(total_colloc_points[i], azimuthVector*main_Omega)

    v_rotational_small = np.zeros((main_NB*small_NB* (small_n - 1), 3))
    n_azimuth = np.zeros((collocN, 3))
    n_azimuth[:main_NB* (main_n - 1)] = azimuthVector
    n_origin = np.zeros((collocN, 3))
    n_origin[:main_NB* (main_n - 1)] = np.array([drone.main_prop.origin.x, drone.main_prop.origin.y, drone.main_prop.origin.z])

    count = 0

    chords_small = drone.small_props[0].chord
    chords_small = 0.5*(chords_small[1:] + chords_small[:-1])
    for i in range(main_NB*small_NB):
        twist[main_NB*(main_n-1)+i*(small_n-1): main_NB*(main_n-1)+i*(small_n-1) + small_n-1, 0] = drone.small_props[count].pitch
        chords[main_NB*(main_n-1)+i*(small_n-1): main_NB*(main_n-1)+i*(small_n-1) + small_n-1, 0] = chords_small

    for i in range(main_NB*small_NB* (small_n - 1)):
        if i == 0:
            origin = np.array([drone.small_props[count].origin.x, drone.small_props[count].origin.y, drone.small_props[count].origin.z])
        if i % (small_NB*(small_n - 1)) == 0 and i != 0:
            count += 1
            origin = np.array([drone.small_props[count].origin.x, drone.small_props[count].origin.y, drone.small_props[count].origin.z])
        azimuthVector = drone.small_props[count].azimuth
        #chords[main_NB*(main_n-1)+i] = drone.small_props[count].chord
        n_azimuth[main_NB*(main_n-1)+i] = azimuthVector
        n_origin[main_NB*(main_n-1)+i] = origin
        Omega = drone.small_props[count].RPM * 2 * np.pi / 60
        # due to its own rotation
        v_rotational_small[i] = np.cross(total_colloc_points[(main_NB*(main_n-1))+i] - origin, azimuthVector*(Omega)) + total_colloc_points[(main_NB*(main_n-1))+i] # Switched order
        # due to main rotor rotation
        v_rotational_small[i] -= np.cross(drone.main_prop.azimuth*main_Omega, total_colloc_points[(main_NB*(main_n-1))+i]) #perhpas -origin
    v_rotational = np.concatenate((v_rotational_main, v_rotational_small))
    
    vel_total_output = np.zeros((collocN, 3))
    vel_axial_output = np.zeros((collocN, 3))
    vel_tangential_output = np.zeros((collocN, 3))



    weight = 0.15
    err = 1.0
    iter = 0
    while (err > 1e-5 and iter<10_000):
        iter+=1
        u = u_influences@Gammas
        v = v_influences@Gammas
        w = w_influences@Gammas

        for i in range(collocN):
            vel_total = np.array([
                v_rotational[i][0] + u[i] + wind[0], 
                v_rotational[i][1] + v[i] + wind[1], 
                v_rotational[i][2] + w[i] + wind[2]
            ])
            vel_total_output[i] = vel_total.flatten()

            v_axial[i] = np.dot(n_azimuth[i], vel_total.flatten()) #removed - sign
            vel_axial_output[i] = n_azimuth[i]*v_axial[i]

            r_vector = total_colloc_points[i] - n_origin[i]

            tangential_direction = np.cross(n_azimuth[i], r_vector)
            tangential_direction /= np.linalg.norm(tangential_direction)  # Normalize

            v_tangential[i] = np.dot(vel_total.flatten(), tangential_direction)
            vel_tangential_output[i] = v_tangential[i]*tangential_direction.flatten()

        
        v_mag = np.sqrt(v_axial**2 + v_tangential**2)
        Re = 1.225*v_mag.flatten()*chords.flatten()/1.81e-5

        inflowangle = np.arctan(v_axial/v_tangential)
        twist[main_NB*(main_n-1):] = inflowangle[main_NB*(main_n-1):]*180/np.pi + 5  # THSI IS JUST A TRICK FIX IT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        alpha = twist -  inflowangle*180/np.pi
        if ReInfluence:

            Cl, Cd = db.get_cl_cd(main_airfoil, Re, alpha.flatten())
        else:
            Cl = np.interp(alpha, alphaPolar, clPolar)

        Gammas_old = Gammas
        Gammas  = weight * Cl * 0.5 * chords* v_mag + (1-weight)*Gammas_old

        err = np.linalg.norm(Gammas - Gammas_old)
        #print(f'Iteration: {iter}, Error: {err}')
    if iter == 1000:
        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        print('Max iterations reached')
        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    print(f'Iteration: {iter}, Error: {err}')


    mean_axial_main = v_axial[:main_NB*(main_n-1)].mean()
    mean_axial_small = v_axial[main_NB*(main_n-1):].mean()

    r_main =  drone.main_prop.r
    r_small = drone.small_props[0].r
    r_steps = (r_main[1:] - r_main[:-1])
    r_small_steps = (r_small[1:] - r_small[:-1])

    if not ReInfluence:
        Cd = np.interp(alpha, alphaPolar, cdPolar)


    Lift = 0.5 * 1.225 * Cl[:main_n-1].flatten() * (v_mag[:main_n-1].flatten()**2) * chords[:main_n-1].flatten() * r_steps.flatten()
    Drag = 0.5 * 1.225 * Cd[:main_n-1].flatten() * (v_mag[:main_n-1].flatten()**2) * chords[:main_n-1].flatten() * r_steps.flatten()
    LD= (Lift.sum())/(Drag.sum())

    Faxial = Lift * np.cos(inflowangle[:main_n-1].flatten()) - Drag * np.sin(inflowangle[:main_n-1].flatten())
    Ftan = Lift * np.sin(inflowangle[:main_n-1].flatten()) + Drag * np.cos(inflowangle[:main_n-1].flatten())

    r = (r_main[1:] + r_main[:-1]) * 0.5
    Torque = np.sum(Ftan * r)*main_NB
    Thrust = Faxial.sum() * main_NB
    print('----------------------------------')

    computed_power = Torque.sum()*drone.main_prop.RPM*2*np.pi/60
    
    # Compute small propeller thrust and torque
    start = main_NB*(main_n-1)
    stop = main_NB*(main_n-1) + (small_n-1)
    lift = 0.5 * 1.225 * Cl[start:stop].flatten() * (v_mag[start:stop].flatten()**2) * chords[start:stop].flatten() * r_small_steps.flatten()
    drag = 0.5 * 1.225 * Cd[start:stop].flatten() * (v_mag[start:stop].flatten()**2) * chords[start:stop].flatten() * r_small_steps.flatten()

    Faxial_small = lift * np.cos(inflowangle[start:stop].flatten()) - drag * np.sin(inflowangle[start:stop].flatten())
    Ftan_small = lift * np.sin(inflowangle[start:stop].flatten()) + drag * np.cos(inflowangle[start:stop].flatten())

    r_small = (r_small[1:] + r_small[:-1]) * 0.5
    Torque_small = np.sum(Ftan_small * r_small)*small_NB
    Thrust_small = Faxial_small.sum() * small_NB

    #print("Small Blade Thrust", Thrust_small, "Combined: ", main_NB*Thrust_small)

    created_moment = main_NB*Thrust_small*drone.main_prop.diameter/2
    #print("Small Blade Torque", Torque_small.sum(), "Combined torque (to create moment): ", created_moment)  
    power_required = main_NB*Torque_small*drone.small_props[0].RPM*2*np.pi/60
    print("computing power required, used RPM: ", drone.small_props[0].RPM)
    #print("Small Blade Power", Torque_small.sum()*drone.small_props[0].RPM*2*np.pi/60, "Power required: ", power_required)

    

    # Compute the induced power for the main rotor
    induced_power = main_NB*(-v_axial[:main_n-1].flatten() * Faxial.flatten()).sum()
    print("Induced power main rotor", induced_power)    

    # Compute the profile power for the main rotor
    cd0 = 0.02
    solidity = drone.main_prop.NB * drone.main_prop.chord[:main_n-1].flatten() / (np.pi*drone.main_prop.diameter/2)
    omega = drone.main_prop.RPM * 2 * np.pi / 60
    R = drone.main_prop.diameter/2
    coefs = solidity*cd0/(4*np.pi)

    profile_power_coefs = coefs*((v_mag[:main_n-1].flatten()*r/(omega*R))**3)
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

        misc  = np.zeros((main_n-1))
        misc[0] = Thrust
        misc[1] = Torque
        misc[2] = LD
        misc[3] = FM
        misc[4] = power_required/total_power
        misc[5] = induced_power 
        misc[6] = profile_power
        misc[7] = induced_power + profile_power
        misc[8] = computed_power
        results = np.column_stack((r, 
                                v_axial[:main_n-1].flatten(), 
                                v_tangential[:main_n-1].flatten(), 
                                inflowangle[:main_n-1].flatten(), 
                                alpha[:main_n-1].flatten(),
                                Faxial.flatten(), 
                                Ftan.flatten(),
                                misc))
        header = "r, v_axial, v_tangential, inflowangle, alpha, Faxial, Ftan, misc"
        case = case.replace('.json', '')
        np.savetxt(f'./results/{case}_res.csv', results, delimiter=',', header=header, comments='')


    drone.total_collocation_points = total_colloc_points
    drone.total_velocity_vectors = vel_total_output
    drone.axial_velocity = vel_axial_output  
    drone.tangential_velocity = vel_tangential_output
    
    return abs(mean_axial_main), abs(mean_axial_small), None, Gammas, FM, created_moment, Torque, Thrust, power_required, induced_power, profile_power