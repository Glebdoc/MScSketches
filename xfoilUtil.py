import os
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd
from scipy.interpolate import RegularGridInterpolator








### The goal of this file is to pre save relevant
# airfoil polars for Various Re numbers
###

airfoil_name = ["a18sm" ]
directory = "./airfoil/dat_files"

alpha_i = -2
alpha_f = 15
alpha_step = 0.5
n_iter = 300
max_runtime = 20  # Max time (in seconds) allowed per XFOIL run

# Let's find the best airfoil for each reynolds number
re = np.linspace(50_000, 500_000, 3)

# we will go through each file in dat_files 
# and run xfoil on it




def createInputFile(airfoil_name, Re, dest_dir="./airfoil/data"):
    if os.path.exists("polar_file.txt"):
        os.remove("polar_file.txt")

    with open("input_file.in", 'w') as input_file:
        input_file.write("LOAD ./airfoil/dat_files/{0}.dat\n".format(airfoil_name))
        input_file.write(airfoil_name + '\n')
        input_file.write("PANE\n")
        input_file.write("OPER\n")
        input_file.write("Visc {0}\n".format(Re))
        input_file.write("PACC\n")
        input_file.write("./{0}/{1}Re{2}.txt\n\n".format(dest_dir, airfoil_name, Re))
        input_file.write("ITER {0}\n".format(n_iter))
        input_file.write("ASeq {0} {1} {2}\n".format(alpha_i, alpha_f, alpha_step))
        input_file.write("\n\n")
        input_file.write("quit\n")

def genDataBase(airfoil_name, Re, max_runtime, dest_dir="./airfoil/data", pull=False):
    if pull:
        airfoil_files = os.listdir(directory)
    else:
        airfoil_files = airfoil_name
    for airfoil in airfoil_files:
        airfoil = airfoil.split(".dat")[0]
        for re in Re:
            createInputFile(airfoil, re, dest_dir=dest_dir)
            try:
                start_time = time.time()
                process = subprocess.Popen("xfoil.exe< input_file.in", shell=True)
                while process.poll() is None:
                    elapsed_time = time.time() - start_time
                    if elapsed_time > max_runtime:
                        process.terminate()
                        print(f"XFOIL run for {airfoil} at Re = {re} exceeded {max_runtime} seconds and was terminated.")
                        break
                process.wait()
                print(f"XFOIL run for {airfoil} at Re = {re} completed.")
                time.sleep(2)  # Sleep for 2 seconds to allow XFOIL to finish writing the polar file
            except Exception as e:
                print(f"XFOIL run for {airfoil} at Re = {re} failed.")

# Re = np.linspace(50_000, 500_000, 3)
# genDataBase(airfoil_name, Re, max_runtime, pull=True, dest_dir="./airfoil/playground")

def computeCubicCoefficients(alpha0, alpha1,f0, f1, df0, df1):
    # solve a system 

    A = np.array([[alpha1**3, alpha1**2, alpha1, 1],
                  [alpha0**3, alpha0**2, alpha0, 1],
                  [3*alpha1**2, 2*alpha1, 1, 0],
                  [3*alpha0**2, 2*alpha0, 1, 0]])
    b = np.array([f1, f0, df1, df0])
    coeffs = np.linalg.solve(A, b)
    return coeffs




def extendPolarData(data):
    alpha = data[:, 0]
    Cl = data[:, 1]
    Cd = data[:, 2]

    dcl0 = (Cl[1] - Cl[0]) / (alpha[1] - alpha[0])
    dcd0 = (Cd[1] - Cd[0]) / (alpha[1] - alpha[0])

    dcl1 = (Cl[-1] - Cl[-2]) / (alpha[-1] - alpha[-2])
    dcd1 = (Cd[-1] - Cd[-2]) / (alpha[-1] - alpha[-2])

    cl0 = Cl[0]
    cd0 = Cd[0]

    cl1 = Cl[-1]
    cd1 = Cd[-1]

    alpha0 = alpha[0] + 360
    alpha1 = alpha[-1] 

    coeffs_cl = computeCubicCoefficients(alpha0, alpha1, cl0, cl1, dcl0, dcl1)
    coeffs_cd = computeCubicCoefficients(alpha0, alpha1, cd0, cd1, dcd0, dcd1)

    cl_polar = lambda x: coeffs_cl[0]*x**3 + coeffs_cl[1]*x**2 + coeffs_cl[2]*x + coeffs_cl[3]
    cd_polar = lambda x: coeffs_cd[0]*x**3 + coeffs_cd[1]*x**2 + coeffs_cd[2]*x + coeffs_cd[3]

    return cl_polar, cd_polar

def clCdfromData(data, target_alpha, plotting=False):
    if plotting:
        my_alpha = np.linspace(-180, 180, 360)
        alpha = data[:, 0]
        Cl = data[:, 1]
        Cd = data[:, 2]
        cl_polar, cd_polar = extendPolarData(data)
        cl = np.zeros_like(my_alpha)
        cd = np.zeros_like(my_alpha)
        for i in range(len(my_alpha)):
            if my_alpha[i] < alpha[0]:
                cl[i] = cl_polar(my_alpha[i]+360)
                cd[i] = cd_polar(my_alpha[i]+360)
            elif my_alpha[i] > alpha[-1]:
                cl[i] = cl_polar(my_alpha[i])
                cd[i] = cd_polar(my_alpha[i])
            else:
                cl[i] = np.interp(my_alpha[i], alpha, Cl)
                cd[i] = np.interp(my_alpha[i], alpha, Cd)
        
        plt.plot(my_alpha, cl, label='Cl interpolated', zorder=1, )
        plt.plot(my_alpha, cd, label='Cd interpolated', zorder=1, )  

        plt.scatter(alpha, Cl, color='red', label='Cl ', zorder=2, alpha=0.5)
        plt.scatter(alpha, Cd, color='blue', label='Cd ', zorder=2, alpha=0.5)
        plt.grid(
                True,
                which='both',          # 'major', 'minor', or 'both'
                linestyle='--',        # Dashed lines
                linewidth=0.7,         
                color='gray',          
                alpha=0.5              # Transparency
            )
        plt.minorticks_on()
        plt.xlabel(r'$\alpha [deg]$')
        plt.ylabel(r'$C_l$, $C_d$')
        plt.legend(fontsize=10, 
                   loc='right', 
                   frameon=True,
                   framealpha=1.0,
                   edgecolor='black',
                   fancybox=False)
        plt.gca().set_aspect(10)
        plt.savefig('interpolated_full.svg', bbox_inches='tight')

    alpha = data[:, 0]
    Cl = data[:, 1]
    Cd = data[:, 2]

    cl_polar, cd_polar = extendPolarData(data)

    if target_alpha < alpha[0]:
        return cl_polar(target_alpha+360), cd_polar(target_alpha+360)
    elif target_alpha > alpha[-1]:
        return cl_polar(target_alpha), cd_polar(target_alpha)
    else:
        return np.interp(target_alpha, alpha, Cl), np.interp(target_alpha, alpha, Cd)

#clCdfromData(np.genfromtxt("./airfoil/data/NACA 0012Re2000000.0.txt", skip_header=12), 10, plotting=True)
    
def updatePolar(airfoil_name):
    N = 720
    alpha = np.linspace(-180, 180, 720)
    for airfoil in airfoil_name:
        for polar in os.listdir("./airfoil/data"):
            Re = 0
            cl, cd = [], [] 
            if airfoil in polar:
                Re =(float(polar.split("Re")[1].split(".txt")[0]))
                data = np.genfromtxt(f"./airfoil/data/{polar}", skip_header=12)
                # # check if the polar file is valid
                # if data.shape[1] != 3:
                #     print(f"Polar file {polar} is not valid.")
                #     continue
                # check if the polar file is empty
                if data.size == 0:
                    print(f"Polar file {polar} is empty.")
                    continue
                cl, cd = [], [] 
                for i in range(N):
                    cl_i, cd_i = clCdfromData(data, alpha[i])
                    print(cl_i, cd_i)
                    cl.append(cl_i)
                    cd.append(cd_i)
                cl = np.array(cl)
                cd = np.array(cd)
            else:
                print(f"Polar file {polar} does not match airfoil {airfoil}.")
                continue
            # write the new polar data to a file
            print(len(cl), len(cd))
            with open(f"./airfoil/data/updated/{polar}", 'w') as f:
                # clear the file
                f.truncate(0)
                # write the header
                f.write(f"Polar data for {airfoil} at Re = {Re}\n")
                f.write(f"Alpha (deg) Cl Cd\n")
                for i in range(N):
                    f.write(f"{alpha[i]} {cl[i]} {cd[i]}\n")

            # close the file
            f.close()
            print(f"Polar data for {airfoil} at Re = {Re} updated.")
 

def getPolar(airfoil_name, Re, target_alpha, interpolate=True):

    # find Re above and below the target Re
    if interpolate:
        Re_files = []
        for polar in os.listdir("./airfoil/data/updated"):
            if airfoil_name in polar:
                Re_files.append(float(polar.split("Re")[1].split(".txt")[0]))
        Re_files = np.array(Re_files)
        Re_files.sort()
        Re_below = Re_files[Re_files < Re]
        if len(Re_below) == 0:
            print(f"No Re below {Re} found for {airfoil_name}.")
            Re_below = [Re_files[0]]
            data = np.genfromtxt(f"./airfoil/data/updated/{airfoil_name}Re{Re_below[0]}.txt", skip_header=12)
            cl, cd = np.interp(target_alpha, data[:, 0], data[:, 1]), np.interp(target_alpha, data[:, 0], data[:, 2])
            return cl, cd
        Re_above = Re_files[Re_files > Re]
        if len(Re_above) == 0:
            print(f"No Re above {Re} found for {airfoil_name}.")
            Re_above = [Re_files[-1]]
            data = np.genfromtxt(f"./airfoil/data/updated{airfoil_name}Re{Re_above[0]}.txt", skip_header=12)
            cl, cd = np.interp(target_alpha, data[:, 0], data[:, 1]), np.interp(target_alpha, data[:, 0], data[:, 2])
            return cl, cd

        data_below = np.genfromtxt(f"./airfoil/data/updated/{airfoil_name}Re{Re_below[-1]}.txt", skip_header=12)
        data_above = np.genfromtxt(f"./airfoil/data/updated/{airfoil_name}Re{Re_above[0]}.txt", skip_header=12)

        cl_below, cd_below = np.interp(target_alpha, data_below[:, 0], data_below[:, 1]), np.interp(target_alpha, data_below[:, 0], data_below[:, 2])
        cl_above, cd_above = np.interp(target_alpha, data_above[:, 0], data_above[:, 1]), np.interp(target_alpha, data_above[:, 0], data_above[:, 2])


        # linear interpolation between the two Re
        Re_diff = Re_above[0] - Re_below[-1]
        if Re_diff == 0:
            print(f"Re above and below are the same for {airfoil_name}.")
            return cl_below, cd_below
        cl = cl_below + (cl_above - cl_below) * (Re - Re_below[-1]) / Re_diff
        cd = cd_below + (cd_above - cd_below) * (Re - Re_below[-1]) / Re_diff
        return cl, cd
    else:
        data = np.genfromtxt(f"./airfoil/data/{airfoil_name}Re{Re}.txt", skip_header=12)
        print('Interpolating')
        cl, cd = clCdfromData(data, target_alpha)
        return cl, cd
    

def build_airfoil_npz(airfoil_name, source_dir="./airfoil/data/updated", output_dir="./airfoil/data/numpy"):
    re_list = []
    alpha_list = []
    cl_list = []
    cd_list = []

    for file in os.listdir(source_dir):
        if airfoil_name in file:
            Re = float(file.split("Re")[1].split(".txt")[0])
            data = np.genfromtxt(os.path.join(source_dir, file), skip_header=2)
            if data.shape[1] != 3:
                continue
            re_list.append(Re)
            alpha_list.append(data[:, 0])
            cl_list.append(data[:, 1])
            cd_list.append(data[:, 2])

    re_sorted_idx = np.argsort(re_list)
    re_array = np.array(re_list)[re_sorted_idx]
    alpha_array = np.array(alpha_list)[re_sorted_idx]
    cl_array = np.array(cl_list)[re_sorted_idx]
    cd_array = np.array(cd_list)[re_sorted_idx]

    # Combine into a 3D array: (n_re, n_alpha, 3)
    polar_data = np.stack([alpha_array, cl_array, cd_array], axis=-1)

    os.makedirs(output_dir, exist_ok=True)
    np.savez_compressed(f"{output_dir}/{airfoil_name}.npz", Re=re_array, polar=polar_data)

    print(f"Saved {airfoil_name} data to {output_dir}/{airfoil_name}.npz")

def load_polar_npz(airfoil_name, npz_dir="./airfoil/numpy"):
    data = np.load(f"{npz_dir}/{airfoil_name}.npz")
    return data["Re"], data["polar"]  # polar: (n_re, n_alpha, 3)

def getPolar_batch(Re_array, alpha_array, airfoil_array, npz_dir="./airfoil/data/numpy"):
    unique_airfoils = np.unique(airfoil_array)
    polar_cache = {}

    # Load all airfoils once
    for af in unique_airfoils:
        Re_vals, polar_vals = load_polar_npz(af, npz_dir)
        polar_cache[af] = (Re_vals, polar_vals)

    Cl = np.zeros_like(Re_array)
    Cd = np.zeros_like(Re_array)

    for i, (Re, alpha, af_name) in enumerate(zip(Re_array, alpha_array, airfoil_array)):
        Re_vals, polar = polar_cache[af_name]
        n_re, n_alpha, _ = polar.shape

        # Find bounding Re indices
        if Re <= Re_vals[0]:
            idx_below = idx_above = 0
        elif Re >= Re_vals[-1]:
            idx_below = idx_above = -1
        else:
            idx_above = np.searchsorted(Re_vals, Re)
            idx_below = idx_above - 1

        t = (Re - Re_vals[idx_below]) / (Re_vals[idx_above] - Re_vals[idx_below]) if idx_above != idx_below else 0

        alpha_b = polar[idx_below, :, 0]
        cl_b = np.interp(alpha, alpha_b, polar[idx_below, :, 1])
        cd_b = np.interp(alpha, alpha_b, polar[idx_below, :, 2])

        alpha_a = polar[idx_above, :, 0]
        cl_a = np.interp(alpha, alpha_a, polar[idx_above, :, 1])
        cd_a = np.interp(alpha, alpha_a, polar[idx_above, :, 2])

        Cl[i] = (1 - t) * cl_b + t * cl_a
        Cd[i] = (1 - t) * cd_b + t * cd_a

    return Cl, Cd

def getCd0_batch(Re_array, airfoil_array, npz_dir="./airfoil/data/numpy"):
    unique_airfoils = np.unique(airfoil_array)
    polar_cache = {}
    cd0 = np.zeros_like(Re_array)
    for af in unique_airfoils:
        Re_vals, polar_vals = load_polar_npz(af, npz_dir)
        polar_cache[af] = (Re_vals, polar_vals)
    for i, (Re, af_name) in enumerate(zip(Re_array, airfoil_array)):
        Re_vals, polar = polar_cache[af_name]
        # find Re above and re below
        if Re <= Re_vals[0]:
            idx_below = idx_above = 0
        elif Re >= Re_vals[-1]:
            idx_below = idx_above = -1
        else:
            idx_above = np.searchsorted(Re_vals, Re)
            idx_below = idx_above - 1

        #find index that is closest to 0 in polar[idx_below, :, 1]
        middle_idx = len(polar[idx_below, :, 1]) // 2
        N =  20
        start  = middle_idx - N
        end = middle_idx + N

        cd0_b = np.interp(0, polar[idx_below, start:end, 1], polar[idx_below,  start:end, 2])

        cd0_a = np.interp(0, polar[idx_above,  start:end, 1], polar[idx_above,  start:end, 2])
        t = (Re - Re_vals[idx_below]) / (Re_vals[idx_above] - Re_vals[idx_below]) if idx_above != idx_below else 0
        cd0[i] = (1 - t) * cd0_b + t * cd0_a
        # print(f"cd0_b: {cd0_b}, cd0_a: {cd0_a}")
    return cd0

# Re_array = np.array([1e5, 2e5, 3e5])
# airfoil_array = np.array(["NACA 0012", "NACA 0012", "NACA 0012"])

# cd0 = getCd0_batch(Re_array, airfoil_array, npz_dir="./airfoil/data/numpy")
# print("Cd0:", cd0)

# airfoil_name = ["NACA 0012", "a18sm" ]
# for airfoil in airfoil_name:
#     build_airfoil_npz(airfoil, source_dir="./airfoil/data/updated", output_dir="./airfoil/data/numpy")