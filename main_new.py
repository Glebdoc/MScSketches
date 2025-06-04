import numpy as np
import pyvista as pv
import os, json
from bemUtils import*
import time
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from bemUtils import*
from geometry import*
import plotter as myPlt
from solver import*
from generator import *

#############################
# ______ FLAGS ______
#############################
JUST_DISPLAY = False
PLOT_RESULTS = True
SAVE_RESULTS = True
OPTIMIZATION = False
THRUST_OPTIMIZATION = False

#############################

# _______ INPUTS _______
MTOW = 100 # N
with open('configs/base.json', 'r') as f:
#with open('configs/base_helicopter.json', 'r') as f:
    base_config = json.load(f)

variables_to_test = {
    "main_propeller.chord_tip": [0.1],

}
config_files = generate_flexible_configs(base_config, variables_to_test, case="exp_drone")
output_title = 'desidrone'

# _______ Solver _______

print("""
 .----------------. .----------------. .----------------. .-----------------..-----------------..----------------. .-----------------..----------------.   .----------------. .----------------. .----------------. .-----------------..----------------. 
| .--------------. | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. | | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. |
| |    _______   | | |   ______     | | |     _____    | | | ____  _____  | | | ____  _____  | | |     _____    | | | ____  _____  | | |    ______    | | | |  ________    | | |  _______     | | |     ____     | | | ____  _____  | | |  _________   | |
| |   /  ___  |  | | |  |_   __ \   | | |    |_   _|   | | ||_   \|_   _| | | ||_   \|_   _| | | |    |_   _|   | | ||_   \|_   _| | | |  .' ___  |   | | | | |_   ___ `.  | | | |_   __ \    | | |   .'    `.   | | ||_   \|_   _| | | | |_   ___  |  | |
| |  |  (__ \_|  | | |    | |__) |  | | |      | |     | | |  |   \ | |   | | |  |   \ | |   | | |      | |     | | |  |   \ | |   | | | / .'   \_|   | | | |   | |   `. \ | | |   | |__) |   | | |  /  .--.  \  | | |  |   \ | |   | | |   | |_  \_|  | |
| |   '.___`-.   | | |    |  ___/   | | |      | |     | | |  | |\ \| |   | | |  | |\ \| |   | | |      | |     | | |  | |\ \| |   | | | | |    ____  | | | |   | |    | | | | |   |  __ /    | | |  | |    | |  | | |  | |\ \| |   | | |   |  _|  _   | |
| |  |`\____) |  | | |   _| |_      | | |     _| |_    | | | _| |_\   |_  | | | _| |_\   |_  | | |     _| |_    | | | _| |_\   |_  | | | \ `.___]  _| | | | |  _| |___.' / | | |  _| |  \ \_  | | |  \  `--'  /  | | | _| |_\   |_  | | |  _| |___/ |  | |
| |  |_______.'  | | |  |_____|     | | |    |_____|   | | ||_____|\____| | | ||_____|\____| | | |    |_____|   | | ||_____|\____| | | |  `._____.'   | | | | |________.'  | | | |____| |___| | | |   `.____.'   | | ||_____|\____| | | | |_________|  | |
| |              | | |              | | |              | | |              | | |              | | |              | | |              | | |              | | | |              | | |              | | |              | | |              | | |              | |
| '--------------' | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' | | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' |
 '----------------' '----------------' '----------------' '----------------' '----------------' '----------------' '----------------' '----------------'   '----------------' '----------------' '----------------' '----------------' '----------------' 
      """)

for config in config_files:
    err_desired = 1e-1
    err_inner = 1e-2
    max_iter = 10
    max_iter_thrust = 50
    iter_outer =0
    RPM_main = 400
    upper_bound_main = 800 
    lower_bound_main = 200
    small_U = 60
    main_U = 1
    weight = 0.1

    if JUST_DISPLAY:
        drone = defineDrone(config)
        drone.display(color_main='gray', color_small='green', extra_points=None, extra_lines=None)
        break

    err = 1
    iter = 0
    err_moment = 1
    err_thrust = 10
    lr = 1.1
    
    # load config file

    

    while err_thrust > 1 and iter<max_iter_thrust:

        err = 1
        err_moment = 1
        iter_outer = 0
        iter = 0
        upper_bound = 25_000
        RPM_small = upper_bound
        lower_bound = 1_000

        data = np.random.rand(10,10)

        while err_moment > err_desired and iter_outer<max_iter:
            err = 1
            iter = 0
            weight = 0.2
            while (err > err_inner and iter<max_iter): 
                drone = defineDrone(config, main_U, small_U, RPM_main, RPM_small)
                #drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)
                print('Drone defined')
                print('_______________________________')
                helicopter = drone.helicopter
                if iter == 0:
                    
                    main_prop_n = drone.main_prop.n
                    main_prop_NB = drone.main_prop.NB
                    if not helicopter:
                        small_prop_n = drone.small_props[0].n
                        nsm = (main_prop_n-1)*main_prop_NB
                        v_axial_old_main = np.linspace(0, -1, nsm)
                        v_axial_old_small = np.linspace(0, -1, int(drone.nPoints+1 - nsm))
                    else:
                        v_axial_old_main = np.linspace(0, -1, int(drone.nPoints)+1)

                    plt.close()
                    fig, ax = plt.subplots()
                    #im = ax.imshow(np.random.rand(10,10), cmap='viridis')
                    
                    im = ax.imshow(v_axial_old_main.reshape(main_prop_NB, -1), cmap='viridis')
                    cbar = plt.colorbar(im, ax=ax)
                old_main_U = main_U 
                old_small_U = small_U
                main_U_new, small_U_new, _,_,_, created_moment, Torque, Thrust, power_required, _,_, v_axial= solve(drone, case=f'{config}', updateConfig=True)
                print('Solved')
                print('_______________________________')
                # error of the main rotor 
                if not helicopter:
                    v_axial_main = v_axial[:nsm]
                    v_axial_small = v_axial[nsm:]

                    v_axial_main_normalized = v_axial_main/np.linalg.norm(v_axial_main)
                    v_axial_small_normalized = v_axial_small/np.linalg.norm(v_axial_small)

                    im.set_data((v_axial_old_main - v_axial_main_normalized).reshape(main_prop_NB, -1))

                    err_main = np.linalg.norm(v_axial_old_main - v_axial_main_normalized)
                    err_small = np.linalg.norm(v_axial_old_small - v_axial_small_normalized)

                    v_axial_old_main = v_axial_main_normalized
                    v_axial_old_small = v_axial_small_normalized

                    v_axial_main_new = (1 - weight) * v_axial_main + weight * v_axial_old_main * np.linalg.norm(v_axial_main)
                    v_axial_small_new = (1 - weight) * v_axial_small + weight * v_axial_old_small * np.linalg.norm(v_axial_small)

                    err = err_main + err_small
                    v_axial = np.concatenate((v_axial_main_new, v_axial_small_new))
                else:
                    v_axial_main = v_axial
                    # normalize the axial velocity
                    v_axial_main_normalized = v_axial_main/np.linalg.norm(v_axial_main)
                    
                    err_main = np.linalg.norm(v_axial_old_main - v_axial_main_normalized)
                    
                    v_axial_old_main = v_axial_main_normalized
                
                    # update the axial velocity
                    v_axial_main_new = (1 - weight) * v_axial_main + weight * v_axial_old_main * np.linalg.norm(v_axial_main)

                
                    err = err_main
                    v_axial =v_axial_main_new
                #im.set_data(np.random.rand(10,10))
                

    
                # Optionally update the color scale
                cbar.update_normal(im)
                plt.pause(0.5)
                #np.savetxt('./auxx/v_axial.txt', v_axial*weight + v_axial_old*(1-weight))
                np.savetxt('./auxx/v_axial.txt', v_axial)
                iter += 1
                print(f'Iteration: {iter}, Error: {err}, Main U: {main_U}, Small U: {small_U}, weight: {weight}')

            print('__________________________________')
            print('Inner loop finished')
            err_moment = np.abs(abs(created_moment) - Torque)
            #drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)

            # Let's try bisection method 
            print(f'Created moment: {created_moment}, Torque: {Torque}, RPM_small: {RPM_small} , err_moment: {err_moment}, lr: {lr} iteration: {iter_outer}')
            if iter_outer == 0 and abs(created_moment) < Torque:
                print('MAX Created moment is less than Torque, adjust boundaries')
                if helicopter:
                    print('Helicopter, no need to adjust boundaries')
                    break
                continue
            Torque = abs(Torque)
            created_moment = abs(created_moment)
            if created_moment > Torque:
                upper_bound = RPM_small
                midpoint = (upper_bound + lower_bound)/2
                RPM_small = midpoint
            if created_moment < Torque:
                lower_bound = RPM_small
                midpoint = (upper_bound + lower_bound)/2
                RPM_small = midpoint
            iter_outer += 1

        if  THRUST_OPTIMIZATION:
            err_thrust = np.abs(Thrust - MTOW)
            if Thrust > MTOW:
                upper_bound_main = RPM_main
                midpoint = (upper_bound_main + lower_bound_main)/2
                RPM_main = midpoint
            if Thrust < MTOW:
                lower_bound_main = RPM_main
                midpoint = (upper_bound_main + lower_bound_main)/2
                RPM_main = midpoint
            print(f'RPM_main: {RPM_main}, Thrust: {Thrust}, MTOW: {MTOW}, err_thrust: {err_thrust}')
        else:
            err_thrust = 0
        
        

    if SAVE_RESULTS:
        drone = defineDrone(config, main_U, small_U, RPM_main, RPM_small)
        main_U, small_U, horses, Gammas, FM, created_moment, Torque, Thrust, power_required, _,_, _= solve(drone, case=f'{config}', updateConfig=False, save=True)
        #u, v , w = computeVelocityField(plane='XY', shift=-0, discretization=350, plotting=True)
        with open(f'configs/{config}', 'r') as f:
            config_data = json.load(f)
        
        # Modify the JSON data
        config_data["results"]["Thrust"] = Thrust
        config_data["results"]["Torque"] = Torque
        config_data["results"]["FM"] = FM

        # Write the updated data back to the file
        with open(f'configs/{config}', 'w') as f:
            json.dump(config_data, f, indent=4)

if PLOT_RESULTS:
    for i in range(len(config_files)):
        config_files[i] = config_files[i].replace('.json', '')
    myPlt.plot(config_files, show = True, title=output_title, helicopter=helicopter, QBlade=False)
    drone.display(color_main='gray', color_small='green', extra_points=None, extra_lines=None)

# clean up 
# delete the auxx files
os.remove('./auxx/v_axial.txt')