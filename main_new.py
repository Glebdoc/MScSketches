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

#############################
# ______ FLAGS ______
#############################
JUST_DISPLAY = False
PLOT_RESULTS = True
SAVE_RESULTS = True
OPTIMIZATION = False

#############################

# _______ INPUTS _______
config_files = ['def_NACA0012.json', 'def_a18sm.json', 'def_NACA0010.json']
output_title = 'airfoils'

# _______ Solver _______
err_desired = 1e-1
max_iter = 1000
small_U = 30
main_U = 2
weight = 0.5

for config in config_files:

    if JUST_DISPLAY:
        drone = defineDrone(config)
        drone.display(color_main='blue', color_small='green', extra_points=None, extra_lines=None)
        break
    err = 1
    iter = 0
    while (err > err_desired and iter<max_iter): 
        drone = defineDrone(config, main_U, small_U)

        old_main_U = main_U 
        old_small_U = small_U
        main_U, small_U, _,_,_, created_moment, Torque, Thrust, power_required, _,_= solve(drone, case=f' ', plotting=False, updateConfig=True)
                        
        main_U = weight*main_U + (1-weight)*old_main_U 
        small_U = weight*small_U + (1-weight)*old_small_U
        err = np.abs(main_U - old_main_U) + np.abs(small_U - old_small_U)
        iter += 1

        print(f'Iteration: {iter}, Error: {err}, Main U: {main_U}, Small U: {small_U}')
    if SAVE_RESULTS:
        drone = defineDrone(config, main_U, small_U)
        main_U, small_U, _,_,FM, created_moment, Torque, Thrust, power_required, _,_= solve(drone, case=f'{config}', plotting=False, updateConfig=False, save=True)
        # write Thrust and Torque into config file 
       #abs(mean_axial_main), abs(mean_axial_small), total_horses, Gammas, FM, created_moment, Torque, Thrust, power_required, induced_power, profile_power 
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
    myPlt.plot(config_files, show = False, title=output_title)