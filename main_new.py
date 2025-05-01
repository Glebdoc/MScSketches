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

#############################

# _______ INPUTS _______
#config_files = ['Um10Us3.json', 'Um10Us5.json','Um10Us7.json']
# read base config 
with open('configs/base.json', 'r') as f:
    base_config = json.load(f)

variables_to_test = {
    "main_propeller.n":[30], 
}
config_files = generate_flexible_configs(base_config, variables_to_test, case="nS")
output_title = 'core'

# _______ Solver _______



for config in config_files:
    err_desired = 1e-1
    err_inner = 1e-1
    max_iter = 50
    iter_outer =0
    RPM_main = 400
    small_U = 60
    main_U = 5
    weight = 0.5
    upper_bound = 35_000
    lower_bound = 2_000
    if JUST_DISPLAY:
        drone = defineDrone(config)
        drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)
        break

    err = 1
    iter = 0
    err_moment = 1
    lr = 1.1
    
    RPM_small = upper_bound

    # load config file
    while err_moment > err_desired and iter_outer<max_iter:
        err = 1
        iter = 0
        weight = 0.3
        while (err > err_inner and iter<max_iter): 
            drone = defineDrone(config, main_U, small_U, RPM_main, RPM_small)

            old_main_U = main_U 
            old_small_U = small_U
            main_U_new, small_U_new, _,_,_, created_moment, Torque, Thrust, power_required, _,_= solve(drone, case=f'{config}', updateConfig=True)

            # update main_U and small_U using gradient descent
            main_U = main_U_new*(1-weight) + weight * main_U
            small_U = small_U_new*(1-weight) + weight * small_U
            err_old = err
            err = np.abs(main_U - old_main_U) + np.abs(small_U - old_small_U)
            if err_old < err:
                new_weight = weight + weight*0.05
                weight = min(0.95, new_weight)

            iter += 1

            print(f'Iteration: {iter}, Error: {err}, Main U: {main_U}, Small U: {small_U}, weight: {weight}')

        # RPM_small, err_moment, lr = adaptive_rpm_controller(Torque, created_moment, RPM_small, err_moment_old, lr)
        err_moment = np.abs(created_moment - Torque)

        # Let's try bisection method 
        print(f'Created moment: {created_moment}, Torque: {Torque}, RPM_small: {RPM_small} , err_moment: {err_moment}, lr: {lr} iteration: {iter_outer}')
        if iter_outer == 0 and created_moment < Torque:
            print('MAX Created moment is less than Torque, adjust boundaries')
            break
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


    if SAVE_RESULTS:
        drone = defineDrone(config, main_U, small_U, RPM_main, RPM_small)
        main_U, small_U, horses, Gammas, FM, created_moment, Torque, Thrust, power_required, _,_= solve(drone, case=f'{config}', updateConfig=False, save=True)
        #u, v , w = computeVelocityField(plane='XY', shift=-2, discretization=100, plotting=True)
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
    myPlt.plot(config_files, show = True, title=output_title)