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
config_files = ['reOffcontOn.json', 'reOffcontOff.json', 'reOncontOn.json', 'reOncontOff.json']
output_title = 'reInfluence_contractionInfluence'

# _______ Solver _______
err_desired = 1e-1
max_iter = 1000
small_U = 30
main_U = 2
weight = 0.2

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
        main_U, small_U, _,_,_, created_moment, Torque, Thrust, power_required, _,_= solve(drone, case=f'{config}', plotting=False, updateConfig=False, save=True)
if PLOT_RESULTS:
    for i in range(len(config_files)):
        config_files[i] = config_files[i].replace('.json', '')
    myPlt.plot(config_files, show = False, title=output_title)