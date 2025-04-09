import os
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd

### The goal of this file is to pre save relevant
# airfoil polars for Various Re numbers
###

airfoil_name = ["NACA 0012", "a18sm", "NACA 0010", "NACA 2224", "NACA 4412", "NACA 2412", "NACA 4410", "NACA 3412" ]
directory = "./airfoil/dat_files"

alpha_i = 0
alpha_f = 15
alpha_step = 0.5
n_iter = 150
max_runtime = 10  # Max time (in seconds) allowed per XFOIL run

def createInputFile(airfoil_name, Re):
    if os.path.exists("polar_file.txt"):
        os.remove("polar_file.txt")

    with open("input_file.in", 'w') as input_file:
        input_file.write("LOAD ./airfoil/dat_files/{0}.dat\n".format(airfoil_name))
        input_file.write(airfoil_name + '\n')
        input_file.write("PANE\n")
        input_file.write("OPER\n")
        input_file.write("Visc {0}\n".format(Re))
        input_file.write("PACC\n")
        input_file.write("./airfoil/data/{0}Re{1}.txt\n\n".format(airfoil_name, Re))
        input_file.write("ITER {0}\n".format(n_iter))
        input_file.write("ASeq {0} {1} {2}\n".format(alpha_i, alpha_f, alpha_step))
        input_file.write("\n\n")
        input_file.write("quit\n")

def genDataBase(airfoil_name, Re, max_runtime):
    for airfoil in airfoil_name:
        for re in Re:
            createInputFile(airfoil, re)
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

def optimalAoA(airfoil_name, Re):
    airfoil_polars = []
    for polar in os.listdir("./airfoil/data"):
        if airfoil_name in polar:
            airfoil_polars.append(float(polar.split("Re")[1].split(".txt")[0]))
    airfoil_polars = np.array(airfoil_polars)
    airfoil_polars.sort()
    closestRe = airfoil_polars[np.argmin(np.abs(airfoil_polars - Re) )]
    df = pd.read_csv(f"./airfoil/data/{airfoil_name}Re{closestRe}.txt", sep='\s+', skiprows=12, header=None)
    df.columns = ["alpha", "CL", "CD", "CDp", "CM", "Top_Xtr", "Bot_Xtr"]
    CL_CD = df["CL"] / df["CD"]
    CL_CD = CL_CD.to_numpy()
    # find the index of the maximum value in CL/CD
    max_index = np.argmax(CL_CD)
    # find the corresponding alpha value
    alpha_optimal = df["alpha"][max_index]
    return alpha_optimal

def plotAirfoilPolars(airfoilName):
    for polar in os.listdir("./airfoil/data"):
        if airfoilName in polar:
            df = pd.read_csv(f"./airfoil/data/{polar}", sep='\s+', skiprows=12, header=None)
            df.columns = ["alpha", "CL", "CD", "CDp", "CM", "Top_Xtr", "Bot_Xtr"]
            label = polar.split("Re")[1].split(".txt")[0]
            print(f"Plotting {airfoilName} at Re = {label}")
            plt.plot(df["alpha"], df["CL"], label=f"Re = {label}")
    plt.xlabel("Alpha")
    plt.legend()
    plt.ylabel("CL")
    plt.show()

Re = np.linspace(20_000, 1_000_000, 70)
genDataBase(airfoil_name, Re, max_runtime)
#plotAirfoilPolars("NACA 0012")

# provided Re as a single number and airfoil name, choose 2 closest polar files and interpolate the data

def getClosestRePolar(Re, airfoil_name):
    Re_files = []
    for polar in os.listdir("./airfoil/data"):
        if airfoil_name in polar:
            Re_files.append(float(polar.split("Re")[1].split(".txt")[0]))
    Re_files = np.array(Re_files)
    Re_files.sort()

    closest_re = Re_files[np.argmin(np.abs(Re_files - Re))]
    df = pd.read_csv(f"./airfoil/data/{airfoil_name}Re{closest_re}.txt", sep='\s+', skiprows=12, header=None)
    df.columns = ["alpha", "CL", "CD", "CDp", "CM", "Top_Xtr", "Bot_Xtr"]
    return df
#getClosestRePolar(137_000, "NACA 0012")