from tqdm import tqdm
import numpy as np
import time
from geometry import Point
import matplotlib.pyplot as plt
import pyvista as pv


def computeVelocityField(horses, Gammas):
    # Update horses with the new Gammas
    for horse, gamma in zip(horses, Gammas):
        horse.Gamma = gamma

    # Define the plane dimensions
    y_range = np.linspace(-2.5, 2.5, 200)
    z_range = np.linspace(-5, .5, 200)

    Y, Z = np.meshgrid(y_range, z_range)

    N_points = len(Y.flatten())

    points = np.column_stack((np.zeros_like(Y.flatten()), Y.flatten(), Z.flatten()))
    u = np.zeros_like(Y.flatten())
    v = np.zeros_like(Y.flatten())
    w = np.zeros_like(Y.flatten())

    # Calculate velocity field for each horse
    for i, horse in tqdm(enumerate(horses), total=len(horses), desc="Velocity field calculation"):
        u_vector = horse.velocity(points, vectorized=True)
        u += u_vector[:, 0]
        v += u_vector[:, 1]
        w += u_vector[:, 2]

    # Calculate the magnitude of the velocity field
    magnitude = np.sqrt(u**2 + v**2 + w**2)
    magnitude[magnitude > 25] = 25  # Cap the magnitude at 25 m/s
    magnitude = magnitude.reshape(Y.shape)
    print(magnitude.shape)

    # Save the magnitude to a file
    np.savetxt('magnitude.txt', magnitude)

    # Create PyVista mesh for visualization
    mesh = pv.PolyData(points)  # Create the points from the grid
    mesh['u'] = u  # Add u velocity component to mesh (use dictionary assignment)
    mesh['v'] = v  # Add v velocity component to mesh
    mesh['w'] = w  # Add w velocity component to mesh
    mesh['magnitude'] = magnitude.flatten()  # Add magnitude of velocity to mesh

    # Set bounds for colormap
    min_mag = 0
    max_mag = 25
    print(f"Colormap bounds: {min_mag} to {max_mag}")

    # Plot the velocity magnitude using PyVista
    plotter = pv.Plotter()
    boring_cmap = plt.get_cmap("plasma", 15)

    vectors = np.c_[np.zeros(len(u)), np.zeros(len(v)), w]

    mesh['vectors'] = vectors
    glyphs = mesh.glyph(orient='vectors', scale='magnitude', factor=0.05)

    plotter.add_mesh(glyphs, scalars='magnitude', cmap=boring_cmap, show_edges=False, clim=[min_mag, max_mag])


    # plotter.add_mesh(mesh, scalars='v', cmap=boring_cmap, show_edges=False, clim=[min_mag, max_mag])

    # # Add scalar bar for the magnitude with customized levels
    # plotter.add_scalar_bar(title="Velocity Magnitude", n_labels=5)

    # Optional: Add axes to the plot for better context
    plotter.add_axes()

    # Show the plot
    plotter.show()

    # Return the velocity components
    return u, v, w




def solve(drone, plotting=False, updateConfig=True):
    # Polar data
    data = np.loadtxt('./A18.txt', skiprows=12)

    alphaPolar = data[:, 0]
    clPolar = data[:, 1]
    cdPolar = data[:, 2]    


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
        small_colloc = np.array(drone.small_props[i].collocationPoints)
        for j in range(small_NB):
            smallCollocPoints[i * small_NB * (small_n - 1) + j * (small_n - 1): i * small_NB * (small_n - 1) + (j + 1) * (small_n - 1)] = small_colloc[j].T

    # Outer loop with progress bar
    total_colloc_points = np.concatenate((mainCollocPoints, smallCollocPoints))
    total_horses = drone.main_prop.horseShoes + [horse for prop in drone.small_props for horse in prop.horseShoes]

    start_time = time.time()
    

    #a =input("Do you want to calculate the influence matrix again? (y/n): ") 
    if updateConfig:
        for i, horse in tqdm(enumerate(total_horses), total=len(total_horses), desc="Influence calculation"):
            u_vector = horse.velocity(total_colloc_points, vectorized=True)
            u_influences[:, i] = u_vector[:, 0]
            v_influences[:, i] = u_vector[:, 1]
            w_influences[:, i] = u_vector[:, 2]
        np.savetxt('u_influences.txt', u_influences)
        np.savetxt('v_influences.txt', v_influences)
        np.savetxt('w_influences.txt', w_influences)
    else:
        u_influences = np.loadtxt('u_influences.txt')
        v_influences = np.loadtxt('v_influences.txt')
        w_influences = np.loadtxt('w_influences.txt')

    # u_influences2 = np.zeros((collocN, collocN))
    # v_influences2 = np.zeros((collocN, collocN))
    # w_influences2 = np.zeros((collocN, collocN))

    # if updateConfig:
    #     for i, colloc in tqdm(enumerate(total_colloc_points), total=len(total_colloc_points), desc="Influence calculation"):
    #         for j, horse in enumerate(total_horses):
    #             x = colloc[0]
    #             y = colloc[1]
    #             z = colloc[2]
    #             u_vector = horse.velocity(Point(x, y, z))
    #             u_influences2[i, j] = u_vector[0]
    #             v_influences2[i, j] = u_vector[1]
    #             w_influences2[i, j] = u_vector[2]
    #     np.savetxt('u_influences2.txt', u_influences2)
    #     np.savetxt('v_influences2.txt', v_influences2)
    #     np.savetxt('w_influences2.txt', w_influences2)
    # else:
    

    v_axial = np.zeros((collocN, 1))
    v_tangential = np.zeros((collocN, 1))
    chords = np.zeros((collocN, 1)) 

    main_Omega = drone.main_prop.RPM * 2 * np.pi / 60
    v_rotational_main = np.zeros((main_NB* (main_n - 1), 3))
    azimuthVector = drone.main_prop.azimuth
    for i in range(main_NB* (main_n - 1)):
        v_rotational_main[i] = np.cross(azimuthVector*main_Omega, total_colloc_points[i])  


    v_rotational_small = np.zeros((main_NB*small_NB* (small_n - 1), 3))
    n_azimuth = np.zeros((collocN, 3))
    n_azimuth[:main_NB* (main_n - 1)] = azimuthVector
    n_origin = np.zeros((collocN, 3))
    chords[:main_NB* (main_n - 1)] = drone.main_prop.chord
    n_origin[:main_NB* (main_n - 1)] = np.array([drone.main_prop.origin.x, drone.main_prop.origin.y, drone.main_prop.origin.z])

    count = 0

    for i in range(main_NB*small_NB* (small_n - 1)):
        if i == 0:
            origin = np.array([drone.small_props[count].origin.x, drone.small_props[count].origin.y, drone.small_props[count].origin.z])
        if i % (small_NB*(small_n - 1)) == 0 and i != 0:
            count += 1
            origin = np.array([drone.small_props[count].origin.x, drone.small_props[count].origin.y, drone.small_props[count].origin.z])
        azimuthVector = drone.small_props[count].azimuth
        chords[main_NB*(main_n-1)+i] = drone.small_props[count].chord
        n_azimuth[main_NB*(main_n-1)+i] = azimuthVector
        n_origin[main_NB*(main_n-1)+i] = origin
        Omega = drone.small_props[count].RPM * 2 * np.pi / 60
        # due to its own rotation
        v_rotational_small[i] = np.cross(azimuthVector*(-Omega), total_colloc_points[(main_NB*(main_n-1))+i] - origin) + total_colloc_points[(main_NB*(main_n-1))+i] # removed - sign from Omega
        # due to main rotor rotation
        v_rotational_small[i] -= np.cross(drone.main_prop.azimuth*main_Omega, total_colloc_points[(main_NB*(main_n-1))+i]) #perhpas -origin
    v_rotational = np.concatenate((v_rotational_main, v_rotational_small))
    


    #due to main rotor rotation 

    weight = 0.2
    err = 1.0
    iter = 0
    while (err > 1e-5 and iter<200):
        iter+=1
        u = u_influences@Gammas
        v = v_influences@Gammas
        w = w_influences@Gammas

        for i in range(collocN):
            vel_total = np.array([
                v_rotational[i][0] + u[i], 
                v_rotational[i][1] + v[i], 
                v_rotational[i][2] + w[i]
            ])

            v_axial[i] = -np.dot(n_azimuth[i], vel_total.flatten())

            r_vector = total_colloc_points[i] - n_origin[i]

            tangential_direction = np.cross(n_azimuth[i], r_vector)
            tangential_direction /= np.linalg.norm(tangential_direction)  # Normalize

            v_tangential[i] = np.dot(vel_total.flatten(), tangential_direction)

        # print("v_axial", v_axial)
        # print("v_tangential", v_tangential)
        v_mag = np.sqrt(v_axial**2 + v_tangential**2)
        inflowangle = np.arctan2(v_axial  ,v_tangential)
        twist = 5
        alpha = np.rad2deg(inflowangle) + twist
        Cl = np.interp(alpha, alphaPolar, clPolar)
        Gammas_old = Gammas
        Gammas  = weight * Cl * 0.5 * chords* v_mag + (1-weight)*Gammas_old

        err = np.linalg.norm(Gammas - Gammas_old)
    print('iter:', iter, 'err',err)


    mean_axial_main = v_axial[:main_NB*(main_n-1)].mean()
    mean_axial_small = v_axial[main_NB*(main_n-1):].mean()

    print("Mean axial velocity main rotor", mean_axial_main)
    print("Mean axial velocity small rotor", mean_axial_small)

    r_main =  drone.main_prop.r
    r_steps = (r_main[1:] - r_main[:-1])

    Cd = np.interp(alpha, alphaPolar, cdPolar)


    Lift = 0.5 * 1.225 * Cl[:main_n-1].flatten() * (v_mag[:main_n-1].flatten()**2) * chords[:main_n-1].flatten() * r_steps.flatten()
    Drag = 0.5 * 1.225 * Cd[:main_n-1].flatten() * (v_mag[:main_n-1].flatten()**2) * chords[:main_n-1].flatten() * r_steps.flatten()

    print("Main Blade L/D", Lift.sum()/Drag.sum())

    Faxial = Lift * np.cos(inflowangle[:main_n-1].flatten()) - Drag * np.sin(inflowangle[:main_n-1].flatten())
    Ftan = Lift * np.sin(inflowangle[:main_n-1].flatten()) + Drag * np.cos(inflowangle[:main_n-1].flatten())

    r = (r_main[1:] + r_main[:-1]) * 0.5
    Torque = np.sum(Ftan * r)*main_NB
    Thrust = Faxial.sum() * main_NB

    print("Main Blade Thrust", Thrust)
    print("Main Blade Torque", Torque.sum())
        
    if plotting:
        r_small = drone.small_props[0].r
        r_plotting = (r_main[:-1] + r_main[1:])*0.5
        r_plotting_small = (r_small[:-1] + r_small[1:])*0.5

        fig, axs = plt.subplots(2, 3, figsize=(10, 15))

        # Plot inflow angle
        axs[0, 0].plot(r_plotting, np.rad2deg(inflowangle[:main_n-1]))
        axs[0, 0].set_xlabel('Collocation Points')
        axs[0, 0].set_ylabel('Inflow Angle (degrees)')

        # Plot axial velocity
        axs[0, 1].plot(r_plotting, v_axial[:main_n-1])
        axs[0, 1].set_xlabel('Collocation Points')
        axs[0, 1].set_ylabel('Axial Velocity (m/s)')

        # Plot Faxial 
        axs[0, 2].plot(r_plotting, Faxial)
        axs[0, 2].set_xlabel('Collocation Points')
        axs[0, 2].set_ylabel('Axial Force (N)')


        # Plot tangential velocity
        axs[1, 0].plot(r_plotting, v_tangential[:main_n-1])
        axs[1, 0].set_xlabel('Collocation Points')
        axs[1, 0].set_ylabel('Tangential Velocity (m/s)')

        # Plot circulation (Gamma)
        axs[1, 1].plot(r_plotting, Gammas[:main_n-1])
        axs[1, 1].set_xlabel('Collocation Points')
        axs[1, 1].set_ylabel('Circulation (Gamma)')

        # Plot Ftan
        axs[1, 2].plot(r_plotting, Ftan)
        axs[1, 2].set_xlabel('Collocation Points')
        axs[1, 2].set_ylabel('Tangential Force (N)')


        plt.tight_layout()
        plt.show()

    return mean_axial_main, mean_axial_small, total_horses, Gammas