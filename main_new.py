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

def adaptive_rpm_controller(
    Torque, created_moment, RPM_small, 
    err_moment_old, base_lr=1.5, 
    min_lr=1.001, max_lr=2.0, 
    sensitivity=0.001
):
    # Calculate current error
    
    err_moment = np.abs(created_moment - Torque)
    if err_moment > err_moment_old:
        lr = base_lr - sensitivity * err_moment
    else:
        lr = base_lr + sensitivity * err_moment

    # Clamp lr to stay stable
    if err_moment < 1:
        max_lr = 1.01
        sensitivity = 0.001

    if err_moment < 0.5:
        max_lr = 1.001
        sensitivity = 0.0001

    lr = np.clip(lr, min_lr, max_lr)
    
    # Adjust RPM based on error direction
    if created_moment > Torque:
        RPM_small /= lr
    else:
        RPM_small *= lr

    # Return updated values
    return RPM_small, err_moment, lr

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
    "main_propeller.NB": [2, 3, 4]
}
config_files = generate_flexible_configs(base_config, variables_to_test, case="study")
output_title = 'NB_influence_rpm1200_D15'

# _______ Solver _______
err_desired = 1e-1
max_iter = 50
RPM_small = 10_000
RPM_main = 900
small_U = 138
main_U = 5
weight = 0.5

for config in config_files:

    if JUST_DISPLAY:
        drone = defineDrone(config)
        drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)
        break

    err = 1
    iter = 0
    err_moment = 1
    dcmdRPM = 1.0
    created_moment_old = 10
    RPM_small_old = 10_000
    alpha = 1000
    lr = 1.3

    # load config file
    while err_moment > err_desired and iter<max_iter:
        err = 1
        iter = 0
        while (err > err_desired and iter<max_iter): 
            drone = defineDrone(config, main_U, small_U, RPM_main, RPM_small)

            old_main_U = main_U 
            old_small_U = small_U
            main_U_new, small_U_new, _,_,_, created_moment, Torque, Thrust, power_required, _,_= solve(drone, case=f'{config}', updateConfig=True)

            # update main_U and small_U using gradient descent
            main_U = main_U_new*(1-weight) + weight * main_U
            small_U = small_U_new*(1-weight) + weight * small_U

            err = np.abs(main_U - old_main_U) + np.abs(small_U - old_small_U)
            iter += 1

            print(f'Iteration: {iter}, Error: {err}, Main U: {main_U}, Small U: {small_U}')
        err_moment_old = err_moment
        err_moment = abs(created_moment - Torque)



        # if err_moment <  err_moment_old:
        #     lr /= 1.05
        #     if lr < 1.003:
        #         lr = 1.003
        # if err_moment < 0.5:
        #     lr = 1.0005
            

        # if created_moment - Torque > 0:
        #     RPM_small /= lr
        # else:
        #     RPM_small *= lr


        
        # print(f'Derivative of created moment: {dcmdRPM}')
        # created_moment_old = created_moment

        
        
        # RPM_small, err_moment, lr = adaptive_rpm_controller(Torque, created_moment, RPM_small, err_moment_old, lr)
        if err_moment_old < err_moment:
            lr -= lr*0.001
            if lr < 1.01:
                lr = 1.001
        if created_moment > Torque:
            RPM_small/= lr
        else:
            RPM_small*= lr
        # update RPMs in the config file
        print(f'Created moment: {created_moment}, Torque: {Torque}, RPM_small: {RPM_small} , err_moment: {err_moment}, lr: {lr} iteration: {iter}')


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