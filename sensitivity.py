
import numpy as np
from typing import Protocol, Any, Dict, Tuple, Optional
from strategies import DroneStrategy, HelicopterStrategy
import os
import matplotlib.pyplot as plt
from trimmer_drone import DroneTrimmer
from trimmer_helicopter import HelicopterTrimmer
import json
import seaborn as sns

def set_bw_design():
    """
    Configure matplotlib + seaborn for consistent black & white (grayscale) plotting style.
    """
    # Seaborn & matplotlib styles
    sns.set_style("whitegrid")
    plt.style.use("seaborn-v0_8")
    sns.set_palette("Greys_r")  # grayscale palette
    
    # Global rcParams for consistent looks
    plt.rcParams.update({
        "figure.figsize": (6, 4),
        "figure.dpi": 200,
        "axes.edgecolor": "black",
        "axes.labelweight": "bold",
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.color": "black",
        "grid.linewidth": 0.5,
        "legend.frameon": True,
        "legend.fancybox": False,
        "legend.shadow": False,  # cleaner for B&W
        "lines.linewidth": 1.5,
        "lines.markersize": 3,
    })
    
    # Line styles / markers / colors for B&W distinction
    design = {
        "line_styles": ['-', '--', '-.'],
        "markers": ['o', 's', '^'],
        "colors": ['black', '0.4', '0.7']  # black, dark gray, light gray
    }
    return design


def update_config(config_path, key1, key2, value, output_path):
    """
    Update a specific key in the JSON configuration file and save to a new file.
    """
    print('config_path=', config_path)
    with open(config_path, 'r') as f:
        config = json.load(f)
        config[key1][key2] = value
        config["settings"]["output_dir"] = output_path

    with open(f'{output_path}/_.json', 'w') as f:
            json.dump(config, f, indent=4)
    return f'{output_path}'


if __name__ == "__main__":

    ########## MAIN WAKE LENGTH SENSITIVITY ANALYSIS ##########

    # strategy = HelicopterStrategy()
    
    # path_orig = "/home/glebdoc/PythonProjects/MScSketches/DesignSpace/sensitivity_analysis/main_wake/"
    # LW = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    # thrust = []
    # torque = []
    
    # key1, key2 = "main_propeller", "wake_length"
    # for lw in LW:
    #     os.makedirs(f'{path_orig}/Lw_{lw}', exist_ok=True)
    #     path = update_config(path_orig + '_.json', key1, key2, lw, f'{path_orig}/Lw_{lw}/')
    #     trimmer = HelicopterTrimmer(strategy, mtow=60, output_dir=path)
    #     solver, iter, errVel = trimmer.trim_velocity(path + '_.json', 400)
    #     solver.save_results(path=path)
    #     thrust.append(solver.thrust)
    #     torque.append(solver.torque)

    # LW = np.asarray(LW, dtype=float)
    # thrust = np.asarray(thrust, dtype=float)
    # torque = np.asarray(torque, dtype=float)


    # # Relative residuals: |Δx| / |x_prev| (protect against divide-by-zero)
    # eps = 1e-12
    # res_thrust = np.abs(np.diff(thrust)) / np.maximum(np.abs(thrust[:-1]), eps)
    # res_torque = np.abs(np.diff(torque)) / np.maximum(np.abs(torque[:-1]), eps)

    # # Plot residuals vs the *second and onward* LW points
    # fig, ax = plt.subplots()
    # design = set_bw_design()
    # ax.plot(LW[1:], res_thrust, 'o-', label='Thrust residual', color=design["colors"][0])
    # ax.plot(LW[1:], res_torque, 's--', label='Torque residual', color=design["colors"][1])
    # ax.set_yscale('log')  # typical for convergence
    # ax.set_xlabel('Main Rotor Wake Length (R)')
    # ax.set_ylabel('Relative change in value')
    # ax.grid(True, which='both', linestyle='--', alpha=0.6)
    # ax.legend()
    # plt.tight_layout()
    # plt.show()
    ########## END MAIN WAKE LENGTH SENSITIVITY ANALYSIS ##########

    ########## DRONE WAKE LENGTH SENSITIVITY ANALYSIS ##########
    # strategy = DroneStrategy()
    # path_orig = "/home/glebdoc/PythonProjects/MScSketches/DesignSpace/sensitivity_analysis/small_wake/"
    # LW = [1, 2, 3, 4, 5, 6, 7, 10, 15, 20, 25, 30]
    # thrust_small = []
    # torque_small = []
    # key1, key2 = "small_propellers", "wake_length"
    # for lw in LW:
    #     os.makedirs(f'{path_orig}/Lw_{lw}', exist_ok=True)
    #     path = update_config(path_orig + '_.json', key1, key2, lw, f'{path_orig}/Lw_{lw}/')
    #     trimmer = DroneTrimmer(strategy, mtow=5, output_dir=path)
    #     solver, iter, errVel = trimmer.trim_velocity(path + '_.json', 366.17187, 9687.5)
    #     solver.save_results(path=path)
    #     thrust_small.append(solver.thrust_small)
    #     torque_small.append(solver.torque_small)
    # LW = np.asarray(LW, dtype=float)
    # thrust_small = np.asarray(thrust_small, dtype=float)
    # torque_small = np.asarray(torque_small, dtype=float)
    # # Relative residuals: |Δx| / |x_prev| (protect against divide-by-zero)

    # eps = 1e-12
    # res_thrust_small = np.abs(np.diff(thrust_small)) / np.maximum(np.abs(thrust_small[:-1]), eps)
    # res_torque_small = np.abs(np.diff(torque_small)) / np.maximum(np.abs(torque_small[:-1]), eps)
    # # Plot residuals vs the *second and onward* LW points
    # fig, ax = plt.subplots()
    # design = set_bw_design()
    # ax.plot(LW[1:], res_thrust_small, 'o-', label='Thrust residual', color=design["colors"][0])
    # ax.plot(LW[1:], res_torque_small, 's--', label='Torque residual', color=design["colors"][1])
    # ax.set_yscale('log')  # typical for convergence
    # ax.set_xlabel('Small Propeller Wake Length (R)')
    # ax.set_ylabel('Relative change in value')
    # ax.grid(True, which='both', linestyle='--', alpha=0.6)
    # ax.legend()
    # plt.tight_layout()
    # plt.show()

    ########## END DRONE WAKE LENGTH SENSITIVITY ANALYSIS ##########

    ########## n main analysis ##########
    # strategy = HelicopterStrategy()
    
    # path_orig = "/home/glebdoc/PythonProjects/MScSketches/DesignSpace/sensitivity_analysis/n_main/"
    # N = [8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96]
    # thrust = []
    # torque = []
    
    # key1, key2 = "main_propeller", "n"
    # for n in N:
    #     os.makedirs(f'{path_orig}/n_{n}', exist_ok=True)
    #     path = update_config(path_orig + '_.json', key1, key2, n, f'{path_orig}/n_{n}/')
    #     trimmer = HelicopterTrimmer(strategy, mtow=60, output_dir=path)
    #     solver, iter, errVel = trimmer.trim_velocity(path + '_.json', 400)
    #     solver.save_results(path=path)
    #     thrust.append(solver.thrust)
    #     torque.append(solver.torque)

    # N = np.asarray(N, dtype=float)
    # thrust = np.asarray(thrust, dtype=float)
    # torque = np.asarray(torque, dtype=float)


    # # Relative residuals: |Δx| / |x_prev| (protect against divide-by-zero)
    # eps = 1e-12
    # res_thrust = np.abs(np.diff(thrust)) / np.maximum(np.abs(thrust[:-1]), eps)
    # res_torque = np.abs(np.diff(torque)) / np.maximum(np.abs(torque[:-1]), eps)

    # # Plot residuals vs the *second and onward* LW points
    # fig, ax = plt.subplots()
    # design = set_bw_design()
    # ax.plot(N[1:], res_thrust, 'o-', label='Thrust residual', color=design["colors"][0])
    # ax.plot(N[1:], res_torque, 's--', label='Torque residual', color=design["colors"][1])
    # ax.set_yscale('log')  # typical for convergence
    # ax.set_xlabel('Main Rotor Blade Elements (n)')
    # ax.set_ylabel('Relative change in value')
    # ax.grid(True, which='both', linestyle='--', alpha=0.6)
    # ax.legend()
    # plt.tight_layout()
    # plt.show()
    ########## END n main analysis ##########

    ########## n small analysis ##########
    # strategy = DroneStrategy()
    # path_orig = "/home/glebdoc/PythonProjects/MScSketches/DesignSpace/sensitivity_analysis/n_small/"
    # N = [8, 12, 16, 20, 24, 28, 32, 36, 40]
    # thrust_small = []
    # torque_small = []
    # key1, key2 = "small_propellers", "n"
    # for n in N:
    #     os.makedirs(f'{path_orig}/n_{n}', exist_ok=True)
    #     path = update_config(path_orig + '_.json', key1, key2, n, f'{path_orig}/n_{n}/')
    #     trimmer = DroneTrimmer(strategy, mtow=5, output_dir=path)
    #     solver, iter, errVel = trimmer.trim_velocity(path + '_.json', 366.17187, 9687.5)
    #     solver.save_results(path=path)
    #     thrust_small.append(solver.thrust_small)
    #     torque_small.append(solver.torque_small)
    # N = np.asarray(N, dtype=float)
    # thrust_small = np.asarray(thrust_small, dtype=float)
    # torque_small = np.asarray(torque_small, dtype=float)
    # # Relative residuals: |Δx| / |x_prev| (protect against divide-by-zero)

    # eps = 1e-12
    # res_thrust_small = np.abs(np.diff(thrust_small)) / np.maximum(np.abs(thrust_small[:-1]), eps)
    # res_torque_small = np.abs(np.diff(torque_small)) / np.maximum(np.abs(torque_small[:-1]), eps)
    # # Plot residuals vs the *second and onward* LW points
    # fig, ax = plt.subplots()
    # design = set_bw_design()
    # ax.plot(N[1:], res_thrust_small, 'o-', label='Thrust residual', color=design["colors"][0])
    # ax.plot(N[1:], res_torque_small, 's--', label='Torque residual', color=design["colors"][1])
    # ax.set_yscale('log')  # typical for convergence
    # ax.set_xlabel('Propeller Blade Elements (n)')
    # ax.set_ylabel('Relative change in value')
    # ax.grid(True, which='both', linestyle='--', alpha=0.6)
    # ax.legend()
    # plt.tight_layout()
    # plt.show()

    ########## END n small analysis ##########

    ########### ppr  analysis ##########
    strategy = HelicopterStrategy()
    
    path_orig = "/home/glebdoc/PythonProjects/MScSketches/DesignSpace/sensitivity_analysis/ppr/"
    PPR = [20, 30, 40, 50, 60, 70, 80, 90, 100, 120, 140]
    thrust = []
    torque = []
    
    key1, key2 = "settings", "ppr"
    for ppr in PPR:
        os.makedirs(f'{path_orig}/ppr_{ppr}', exist_ok=True)
        path = update_config(path_orig + '_.json', key1, key2, ppr, f'{path_orig}/ppr_{ppr}/')
        trimmer = HelicopterTrimmer(strategy, mtow=60, output_dir=path)
        solver, iter, errVel = trimmer.trim_velocity(path + '_.json', 400)
        solver.save_results(path=path)
        thrust.append(solver.thrust)
        torque.append(solver.torque)

    PPR = np.asarray(PPR, dtype=float)
    thrust = np.asarray(thrust, dtype=float)
    torque = np.asarray(torque, dtype=float)


    # Relative residuals: |Δx| / |x_prev| (protect against divide-by-zero)
    eps = 1e-12
    res_thrust = np.abs(np.diff(thrust)) / np.maximum(np.abs(thrust[:-1]), eps)
    res_torque = np.abs(np.diff(torque)) / np.maximum(np.abs(torque[:-1]), eps)

    # Plot residuals vs the *second and onward* LW points
    fig, ax = plt.subplots()
    design = set_bw_design()
    ax.plot(PPR[1:], res_thrust, 'o-', label='Thrust residual', color=design["colors"][0])
    ax.plot(PPR[1:], res_torque, 's--', label='Torque residual', color=design["colors"][1])
    ax.set_yscale('log')  # typical for convergence
    ax.set_xlabel('PPR [-]')
    ax.set_ylabel('Relative change in value')
    ax.grid(True, which='both', linestyle='--', alpha=0.6)
    ax.legend()
    plt.tight_layout()
    plt.show()
