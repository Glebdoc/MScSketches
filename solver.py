from tqdm import tqdm
import numpy as np
import time
from geometry import Point
import matplotlib.pyplot as plt
import pyvista as pv

# I believe this whole function is wrong.
def computeVelocityField(horses, Gammas, plane='YZ', shift=0, discretization=50):
    # Update horses with the new Gammas
    for horse, gamma in zip(horses, Gammas):
        horse.Gamma = gamma

    # Define the plane dimensions
    if plane == 'YZ':
        y_range = np.linspace(-2.5, 2.5, discretization)
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
        x_range = np.linspace(-2.5, 2.5, discretization)
        y_range = np.linspace(-2.5, 2.5, discretization)

        X, Y = np.meshgrid(x_range, y_range)
        N_points = len(X.flatten())
        points = np.column_stack((X.flatten(), Y.flatten(), np.ones(N_points)*shift))

    u = np.zeros(N_points)
    v = np.zeros(N_points)
    w = np.zeros(N_points)

    # Calculate velocity field for each horse
    for i, horse in tqdm(enumerate(horses), total=len(horses), desc="Velocity field calculation"):
        u_vector = horse.velocity(points, vectorized=True)
        u += u_vector[:, 0]
        v += u_vector[:, 1]
        w += u_vector[:, 2]

    # Calculate the magnitude of the velocity field
    magnitude = np.sqrt(u**2 + v**2 + w**2)
    magnitude[magnitude > 25] = 25  # Cap the magnitude at 25 m/s
    magnitude = magnitude.reshape((discretization, discretization))
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

    vectors = np.c_[u, v, w]

    mesh['vectors'] = vectors
    glyphs = mesh.glyph(orient='vectors', scale='magnitude', factor=0.05)

    plotter.add_mesh(glyphs, scalars='magnitude', cmap=boring_cmap, show_edges=False, clim=[min_mag, max_mag])


    # plotter.add_mesh(mesh, scalars='v', cmap=boring_cmap, show_edges=False, clim=[min_mag, max_mag])

    # # Add scalar bar for the magnitude with customized levels
    # plotter.add_scalar_bar(title="Velocity Magnitude", n_labels=5)

    # Optional: Add axes to the plot for better context
    plotter.add_axes()
    plotter.show_grid()

    # Show the plot
    plotter.show()

    # Return the velocity components
    return u, v, w




def solve(drone, plotting=False, updateConfig=True, case='main', wind=np.array([0,0,0])):
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

    total_colloc_points = np.concatenate((mainCollocPoints, smallCollocPoints))
    total_horses = drone.main_prop.horseShoes + [horse for prop in drone.small_props for horse in prop.horseShoes]
    start_time = time.time()
    
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



    weight = 0.3
    err = 1.0
    iter = 0
    while (err > 1e-6 and iter<500):
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
        inflowangle = np.arctan(v_axial/v_tangential)
        twist[main_NB*(main_n-1):] = inflowangle[main_NB*(main_n-1):]*180/np.pi + 5
        alpha = twist -  inflowangle*180/np.pi
        Cl = np.interp(alpha, alphaPolar, clPolar)
        Gammas_old = Gammas
        Gammas  = weight * Cl * 0.5 * chords* v_mag + (1-weight)*Gammas_old

        err = np.linalg.norm(Gammas - Gammas_old)


    mean_axial_main = v_axial[:main_NB*(main_n-1)].mean()
    mean_axial_small = v_axial[main_NB*(main_n-1):].mean()

    r_main =  drone.main_prop.r
    r_small = drone.small_props[0].r
    r_steps = (r_main[1:] - r_main[:-1])
    r_small_steps = (r_small[1:] - r_small[:-1])

    Cd = np.interp(alpha, alphaPolar, cdPolar)


    Lift = 0.5 * 1.225 * Cl[:main_n-1].flatten() * (v_mag[:main_n-1].flatten()**2) * chords[:main_n-1].flatten() * r_steps.flatten()
    Drag = 0.5 * 1.225 * Cd[:main_n-1].flatten() * (v_mag[:main_n-1].flatten()**2) * chords[:main_n-1].flatten() * r_steps.flatten()

    LD= Lift.sum()/Drag.sum()

    Faxial = Lift * np.cos(inflowangle[:main_n-1].flatten()) - Drag * np.sin(inflowangle[:main_n-1].flatten())
    Ftan = Lift * np.sin(inflowangle[:main_n-1].flatten()) + Drag * np.cos(inflowangle[:main_n-1].flatten())

    r = (r_main[1:] + r_main[:-1]) * 0.5
    Torque = np.sum(Ftan * r)*main_NB
    Thrust = Faxial.sum() * main_NB

    print("Main Blade Thrust", Thrust)
    print("Main Blade Torque", Torque.sum())
    print('Tip thrust required:', Torque.sum()/(drone.main_prop.diameter/2)/main_NB)
    computed_power = Torque.sum()*drone.main_prop.RPM*2*np.pi/60
    print("Main Blade Power", computed_power)
    p_ideal = Thrust * np.sqrt(Thrust/(2*(drone.main_prop.diameter**2)*np.pi*1.225/4))
    FM = p_ideal/computed_power

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
    print("Small Blade Torque", Torque_small.sum(), "Combined torque (to create moment): ", created_moment)  
    power_required = main_NB*Torque_small.sum()*drone.small_props[0].RPM*2*np.pi/60
    print("Small Blade Power", Torque_small.sum()*drone.small_props[0].RPM*2*np.pi/60, "Power required: ", power_required)

    

    # Compute the induced power for the main rotor
    induced_power = main_NB*(abs(v_axial[:main_n-1].flatten()) * Faxial.flatten()).sum()
    #print("Induced power main rotor", induced_power)    



    # save results to csv 
    misc  = np.zeros((main_n-1))
    misc[0] = Thrust
    misc[1] = Torque
    misc[2] = LD
    results = np.column_stack((r, 
                               v_axial[:main_n-1].flatten(), 
                               v_tangential[:main_n-1].flatten(), 
                               inflowangle[:main_n-1].flatten(), 
                               alpha[:main_n-1].flatten(),
                               Faxial.flatten(), 
                               Ftan.flatten(),
                               misc))
    header = "r, v_axial, v_tangential, inflowangle, alpha, Faxial, Ftan, misc"

    np.savetxt(f'./results/results_{case}.csv', results, delimiter=',', header=header, comments='')

        
    if plotting:
        r_small = drone.small_props[0].r
        r_plotting = (r_main[:-1] + r_main[1:])*0.5
        r_plotting_small = (r_small[:-1] + r_small[1:])*0.5

        fig, axs = plt.subplots(2, 3, figsize=(10, 15))

        # Plot inflow angle
        axs[0, 0].plot(r_plotting, np.rad2deg(inflowangle[:main_n-1]))
        axs[0, 0].set_xlabel('Collocation Points')
        axs[0, 0].set_ylabel('Inflow Angle (degrees)')
        axs[0, 0].plot(np.linspace(r_plotting[0], r_plotting[-1], len(r_small)-1), np.rad2deg(inflowangle[start:stop]), label='small')
        axs[0, 0].legend()

        # # Plot axial vel30ocity
        # axs[0, 1].plot(r_plotting, v_axial[:main_n-1])
        # axs[0, 1].set_xlabel('Collocation Points')
        # axs[0, 1].set_ylabel('Axial Velocity (m/s)')

        # Plort alpha 
        axs[0, 1].plot(r_plotting, alpha[:main_n-1])
        axs[0, 1].plot(np.linspace(r_plotting[0], r_plotting[-1], len(r_small)-1), alpha[start:stop], label='small')
        axs[0, 1].set_xlabel('Collocation Points')
        axs[0, 1].set_ylabel('Alpha (degrees)')
        axs[0, 1].set_title('Alpha')
        axs[0, 1].legend()

        # Plot Faxial 
        axs[0, 2].plot(r_plotting, Faxial)
        axs[0, 2].set_xlabel('Collocation Points')
        axs[0, 2].set_ylabel('Axial Force (N)')


        # Plot tangential velocity
        axs[1, 0].plot(r_plotting, -v_tangential[:main_n-1])
        axs[1, 0].set_xlabel('Collocation Points')
        axs[1, 0].set_ylabel('Tangential Velocity (m/s)')

        # Plot circulation (Gamma)
        axs[1, 1].plot(r_plotting, Gammas[:main_n-1])
        axs[1, 1].plot(np.linspace(r_plotting[0], r_plotting[-1], len(r_small)-1), Gammas[start:stop], label='small')
        axs[1, 1].set_xlabel('Collocation Points')
        axs[1, 1].set_ylabel('Circulation (Gamma)')
        axs[1, 1].legend()

        # Plot Ftan
        axs[1, 2].plot(r_plotting, Ftan)
        axs[1, 2].set_xlabel('Collocation Points')
        axs[1, 2].set_ylabel('Tangential Force (N)')


        plt.tight_layout()
        plt.show()
    drone.total_collocation_points = total_colloc_points
    drone.total_velocity_vectors = vel_total_output
    drone.axial_velocity = vel_axial_output  
    drone.tangential_velocity = vel_tangential_output
    return abs(mean_axial_main), abs(mean_axial_small), total_horses, Gammas, FM, created_moment, Torque, Thrust, power_required