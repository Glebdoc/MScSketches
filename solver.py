from tqdm import tqdm
import numpy as np
import time
from geometry import Point

def solve(drone):
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

    main_colloc = np.array(drone.main_prop.collocationPoints)
  
    for i in range(main_NB):
        mainCollocPoints[i * (main_n - 1):(i + 1) * (main_n - 1)] = main_colloc[i].T

    for i in range(main_NB):
        for j in range(small_NB):
            small_colloc = np.array(drone.small_props[j].collocationPoints)
            smallCollocPoints[i * small_NB * (small_n - 1) + j * (small_n - 1):i * small_NB * (small_n - 1) + (j + 1) * (small_n - 1)] = small_colloc[i].T

    # Outer loop with progress bar
    total_colloc_points = np.concatenate((mainCollocPoints, smallCollocPoints))
    print("collocation points", total_colloc_points)
    total_horses = drone.main_prop.horseShoes + [horse for prop in drone.small_props for horse in prop.horseShoes]

    start_time = time.time()
    
    for i, colloc in tqdm(enumerate(total_colloc_points), total=len(total_colloc_points), desc="Influence calculation"):
        for j, horse in enumerate(total_horses):
            x = colloc[0]
            y = colloc[1]
            z = colloc[2]
            u_vector = horse.velocity(Point(x, y, z))
            u_influences[i, j] = u_vector[0]
            v_influences[i, j] = u_vector[1]
            w_influences[i, j] = u_vector[2]


    err = 1.0
    iter = 0

    v_axial = np.zeros((collocN, 1))
    v_tangential = np.zeros((collocN, 1))

    #testing
    Omega = drone.main_prop.RPM * 2 * np.pi / 60
    v_rotational_main = np.zeros((main_NB* (main_n - 1), 3))
    for i in range(main_NB* (main_n - 1)):
        v_rotational_main[i] = np.cross(np.array([0, 0, Omega]), total_colloc_points[i])  

    v_rotational_small = np.zeros((main_NB*small_NB* (small_n - 1), 3))

    count = -1
    for i in range(main_NB*small_NB* (small_n - 1)):
        if i % (small_NB*(main_n - 1)) == 0:
            count += 1
        azimuthVector = drone.small_props[count].azimuth
        Omega = drone.small_props[count].RPM * 2 * np.pi / 60

        v_rotational_small[i] = np.cross(azimuthVector*Omega, total_colloc_points[(main_NB*(main_n-1))+i])

    
    v_rotational = np.concatenate((v_rotational_main, v_rotational_small))
    return v_rotational


    # while (err > 1e-4 and iter<1200):
    #     iter+=1
    #     u = u_influences@Gammas
    #     v = v_influences@Gammas
    #     w = w_influences@Gammas

    #     # First do the main propeller
    #     Omega = drone.main_prop.RPM * 2 * np.pi / 60
    #     for i in range(main_NB):
    #         for j in range(main_n - 1):
    #             k = i * (main_n - 1) + j
    #             v_axial[k] = u_influences[k]
    #             v_tangential[k] = np.cro

    #     v_axial = u 




