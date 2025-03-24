import numpy as np
import pyvista as pv
import os, json
from bemUtils import*
import time
import matplotlib.pyplot as plt
from scipy.optimize import minimize

from geometry import*
from solver import*


err = 1.0
err_desired = 1e-3
iter = 0
max_iter = 100
weight = 0.5
main_U = 4.2


runWhole = True
multipleConfigs = False

# for every .json file in the configs folder
# load the configuration

if multipleConfigs:
    configuration_files = [f for f in os.listdir('./configs') if f.endswith('.json')]
    for file in configuration_files:
        with open(f'./configs/{file}', 'r') as f:
            config = json.load(f)
            main_position = Point(*config['main_propeller']['position'])
            main_angles = config['main_propeller']['angles']
            main_hub = config['main_propeller']['hub']
            main_diameter = config['main_propeller']['diameter']
            #main_NB = config['main_propeller']['number_of_blades']
            main_NB = 3
            #main_pitch = config['main_propeller']['pitch']
            main_pitch = 2
            main_RPM = config['main_propeller']['rpm']
            #main_chord = config['main_propeller']['chord']
            main_chord = 0.2
            #main_n = config['main_propeller']['n']
            main_n = 30

            #small_props_angle = config['small_propellers']['angle']
            small_props_angle = 90
            small_props_diameter = config['small_propellers']['diameter']
            small_props_NB = config['small_propellers']['number_of_blades']
            small_props_RPM = config['small_propellers']['rpm']
            small_props_chord = config['small_propellers']['chord']
            #small_props_n = config['small_propellers']['n']
            small_props_n = 10

            wind_speed = 30
            wind_angle = 20

            wind = np.array([wind_speed*np.cos(wind_angle*np.pi/180),
                            0,
                            wind_speed*np.sin(wind_angle*np.pi/180)])

            if runWhole:
                while (err > err_desired and iter<max_iter):
                    

                    drone = Drone(main_position, main_angles, main_hub, main_diameter, 
                                main_NB, main_pitch, main_RPM, main_chord, main_n,
                                small_props_angle, small_props_diameter, small_props_NB,
                                small_props_RPM, small_props_chord, small_props_n,
                                mainWakeLength=1, smallWakeLength=6, main_U=main_U, small_U = small_U, main_distribution='uniform', small_distribution='uniform')

                    old_main_U = main_U
                    old_small_U = small_U

                    main_U, small_U, _,_= solve(drone, case=f'{file.replace(".json", "")}', plotting=False, updateConfig=True, wind=wind)

                    main_U = weight*main_U + (1-weight)*old_main_U 
                    small_U = weight*small_U + (1-weight)*old_small_U

                    err = np.abs(main_U - old_main_U) + np.abs(small_U - old_small_U)
                    if err < err_desired:
                        main_U, small_U, horses, Gammas = solve(drone,plotting=True, updateConfig=False, wind=wind)
                    iter += 1
                    x_data.append(iter)
                    y_data.append(err)
                    line.set_xdata(x_data)
                    line.set_ydata(y_data)
                    ax.relim()
                    ax.autoscale_view()
                    plt.draw()
                    plt.pause(0.01)
                    print(f'Iteration: {iter}, Error: {err}, Main U: {main_U}, Small U: {small_U}')
                plt.ioff()  # Disable interactive mode when done
                plt.show()  # Keep the final plot visible
                drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)
                #np.savetxt('table.txt', drone.vortexTABLE, delimiter=',', fmt=['%1.4e', '%1.4e','%1.4e','%1.4e','%1.4e','%1.4e','%i'])
                #np.savetxt('collocation_points.txt', drone.total_collocation_points, delimiter=',', fmt=['%1.4e', '%1.4e','%1.4e'])
                #u, v, w = computeVelocityField(horses, Gammas, plane='XZ', shift=0.5)

            else:
                drone = Drone(main_position, main_angles, main_hub, main_diameter, 
                                main_NB, main_pitch, main_RPM, main_chord, main_n,
                                small_props_angle, small_props_diameter, small_props_NB,
                                small_props_RPM, small_props_chord, small_props_n,
                                mainWakeLength=1, smallWakeLength=6, main_U=main_U, small_U = small_U, main_distribution='uniform', small_distribution='uniform')
                #main_U, small_U, _, _ = solve(drone,plotting=True, updateConfig=True)
                drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)

            err = 1.0
            iter = 0
            max_iter = 100

# Drone parameters 
MTOW = 30 #[N]

def twistGen(in_hub, in_tip, r, AoA):
    in_hub = in_hub + AoA
    in_tip = in_tip + AoA 

    if r[0] == 0:
        r[0] = 1e-6

    k = (in_tip - in_hub)/(1/(r[-1]) - 1/r[0])
    m = in_hub - k/(r[0])

    twist = k/r + m
    return twist

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

# Main propeller parameters
main_n = 80
main_position = Point(0, 0, 0)
main_angles = (0, 0, 0)  # Initial rotation angles (e.g., Euler angles)
main_hub = 0.1
main_diameter = 1.4*2
main_NB = 3  # Number of blades
main_RPM = 350
main_chord = np.linspace(0.13, 0.09, main_n-1)
main_chord = np.concatenate([main_chord]*main_NB)

main_r = np.linspace(main_hub, main_diameter/2, main_n-1)
#main_pitch = twistGen(22, 7, main_r, 5)
main_pitch = np.linspace(15, 15, main_n-1) 
main_pitch = np.concatenate([main_pitch]*main_NB)


# Create the drone

max_iter = 100
weight = 0.5
main_U = 5.16

# Wind conditions 
wind_speed = 0
wind_angle = 0

wind = np.array([wind_speed*np.cos(wind_angle*np.pi/180),
                0,
                wind_speed*np.sin(wind_angle*np.pi/180)])



def objective(x_norm):
    err = 1.0
    iter = 0
    global main_U, small_U
    main_RPM, small_props_RPM, main_diameter, small_props_diameter = denormalize(x_norm, bounds)
    while (err > err_desired and iter<max_iter):
        drone = Drone(main_position, main_angles, main_hub, main_diameter, 
                    main_NB, main_pitch, main_RPM, main_chord, main_n,
                    small_props_angle, small_props_diameter, small_props_NB, small_props_hub,
                    small_props_RPM, small_props_chord, small_props_n, small_props_pitch,
                    mainWakeLength=1, smallWakeLength=6, main_U=main_U, small_U = small_U, main_distribution='uniform', small_distribution='uniform')

        old_main_U = main_U
        old_small_U = small_U
        main_U, small_U, _,_,_, created_moment, Torque, Thrust, power_required= solve(drone, case=f' ', plotting=False, updateConfig=True, wind=wind)
                
        main_U = weight*main_U + (1-weight)*old_main_U 
        small_U = weight*small_U + (1-weight)*old_small_U
        err = np.abs(main_U - old_main_U) + np.abs(small_U - old_small_U)
        # if err < err_desired:
        #     main_U, small_U, horses, Gammas, fm, created_moment, Torque, Thrust = solve(drone,plotting=True, updateConfig=False, wind=wind)
        iter += 1

        print(f'Iteration: {iter}, Error: {err}, Main U: {main_U}, Small U: {small_U}')
    endurance_1 = endurance(power_required, 5000, battery_config='2P6S', battery_efficiency=0.9)
    thrust_error = (Thrust - MTOW)**2 
    endurance_error = (endurance_1 - 60)**2
    moment_error = (created_moment - Torque)**2
    #drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)
    print(':::::::::::::::::::::::::::::::::::::::::::::::::::::::::')
    print(f'Thrust Error: {thrust_error}, Moment Error: {moment_error}, Endurance Error: {endurance_error}')
    print(f'Endurance: {endurance_1}')
    print(f'Main RPM: {main_RPM}, Small Props RPM: {small_props_RPM}', f'Main Diameter: {main_diameter}, Small Props Diameter: {small_props_diameter}')
    print(':::::::::::::::::::::::::::::::::::::::::::::::::::::::::')
    return thrust_error + moment_error + endurance_error




############################################
OPTIMIZATION = False
############################################
def normalize(x, bounds):
    return [(x[i] - bounds[i][0]) / (bounds[i][1] - bounds[i][0]) for i in range(len(x))]

def denormalize(x, bounds):
    return [x[i] * (bounds[i][1] - bounds[i][0]) + bounds[i][0] for i in range(len(x))]

if OPTIMIZATION:
    bounds = [(350, 550), (15000, 20000), (0.8, 1.4), (0.08, 0.12)]
    x0 = normalize([main_RPM, small_props_RPM, main_diameter, small_props_diameter], bounds)
   
    res = minimize(objective, x0, bounds=[(0,1)]*len(x0), method='Powell', tol=0.1)
    optimal_x = denormalize(res.x, bounds)
    optimal_main_RPM, optimal_small_RPM, optimal_main_diameter, optimal_small_diameter = optimal_x

    print(f'Optimized Main RPM: {optimal_main_RPM}')
    print(f'Optimized Small Props RPM: {optimal_small_RPM}')
    print(f'Optimized Main Diameter: {optimal_main_diameter}')
    print(f'Optimized Small Props Diameter: {optimal_small_diameter}')
else:
    main_RPM = 350

    RPM = np.linspace(350, 550, 10)
    induced_powers = []
    parasitic_powers = []

    err = 1.0
    iter = 0
    print(':::::::::::::::::::::::::::::::::::::::::::::::::::::::::')
    while (err > err_desired and iter<max_iter):
            drone = Drone(main_position, main_angles, main_hub, main_diameter, 
                        main_NB, main_pitch, main_RPM, main_chord, main_n,
                        small_props_angle=None, small_props_diameter=None, small_props_NB=None, small_props_hub=None,
                        small_props_RPM=None, small_props_chord=None, small_props_n=None, small_props_pitch=None,
                        mainWakeLength=1, smallWakeLength=6, main_U=main_U, small_U = None, main_distribution='uniform', small_distribution='uniform', helicopter=True)

            old_main_U = main_U
            # abs(mean_axial_main), total_horses, Gammas, FM, Torque, Thrust, induced_power, profile_power
            main_U, _,_,_, Torque, Thrust, _,_= solve(drone, case=f' ', plotting=False, updateConfig=True, wind=wind)
                    
            main_U = weight*main_U + (1-weight)*old_main_U 

            err = np.abs(main_U - old_main_U) 
            if err < err_desired:
                main_U, horses, Gammas, fm, Torque, Thrust, induced_power, parasitic_power= solve(drone,plotting=True, updateConfig=False, wind=wind)
                induced_powers.append(induced_power)
                parasitic_powers.append(parasitic_power)
            iter += 1

            print(f'Iteration: {iter}, Error: {err}, Main U: {main_U}')
    # endurance_1 = endurance(power_required, 5000, battery_config='2P6S', battery_efficiency=0.9)
    # endurance_2 = endurance(power_required, 5000, battery_config='1P6S', battery_efficiency=0.9)
    # print(f'Endurance 2P6S: {endurance_1} minutes')
    # print(f'Endurance 1P6S: {endurance_2} minutes')
    # drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)
