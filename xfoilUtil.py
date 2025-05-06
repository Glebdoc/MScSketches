import os
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd
from scipy.interpolate import RegularGridInterpolator








# ### The goal of this file is to pre save relevant
# # airfoil polars for Various Re numbers
# ###

# airfoil_name = ["NACA 0012", "a18sm" ]
# directory = "./airfoil/dat_files"

# alpha_i = -5
# alpha_f = 17
# alpha_step = 0.5
# n_iter = 300
# max_runtime = 10  # Max time (in seconds) allowed per XFOIL run

# def createInputFile(airfoil_name, Re):
#     if os.path.exists("polar_file.txt"):
#         os.remove("polar_file.txt")

#     with open("input_file.in", 'w') as input_file:
#         input_file.write("LOAD ./airfoil/dat_files/{0}.dat\n".format(airfoil_name))
#         input_file.write(airfoil_name + '\n')
#         input_file.write("PANE\n")
#         input_file.write("OPER\n")
#         input_file.write("Visc {0}\n".format(Re))
#         input_file.write("PACC\n")
#         input_file.write("./airfoil/data/{0}Re{1}.txt\n\n".format(airfoil_name, Re))
#         input_file.write("ITER {0}\n".format(n_iter))
#         input_file.write("ASeq {0} {1} {2}\n".format(alpha_i, alpha_f, alpha_step))
#         input_file.write("\n\n")
#         input_file.write("quit\n")

# def genDataBase(airfoil_name, Re, max_runtime):
#     for airfoil in airfoil_name:
#         for re in Re:
#             createInputFile(airfoil, re)
#             try:
#                 start_time = time.time()
#                 process = subprocess.Popen("xfoil.exe< input_file.in", shell=True)
#                 while process.poll() is None:
#                     elapsed_time = time.time() - start_time
#                     if elapsed_time > max_runtime:
#                         process.terminate()
#                         print(f"XFOIL run for {airfoil} at Re = {re} exceeded {max_runtime} seconds and was terminated.")
#                         break
#                 process.wait()
#                 print(f"XFOIL run for {airfoil} at Re = {re} completed.")
#                 time.sleep(2)  # Sleep for 2 seconds to allow XFOIL to finish writing the polar file
#             except Exception as e:
#                 print(f"XFOIL run for {airfoil} at Re = {re} failed.")

# def optimalAoA(airfoil_name, Re):
#     airfoil_polars = []
#     for polar in os.listdir("./airfoil/data"):
#         if airfoil_name in polar:
#             airfoil_polars.append(float(polar.split("Re")[1].split(".txt")[0]))
#     airfoil_polars = np.array(airfoil_polars)
#     airfoil_polars.sort()
#     closestRe = airfoil_polars[np.argmin(np.abs(airfoil_polars - Re) )]
#     df = pd.read_csv(f"./airfoil/data/{airfoil_name}Re{closestRe}.txt", sep='\s+', skiprows=12, header=None)
#     df.columns = ["alpha", "CL", "CD", "CDp", "CM", "Top_Xtr", "Bot_Xtr"]
#     CL_CD = df["CL"] / df["CD"]
#     CL_CD = CL_CD.to_numpy()
#     # find the index of the maximum value in CL/CD
#     max_index = np.argmax(CL_CD)
#     # find the corresponding alpha value
#     alpha_optimal = df["alpha"][max_index]
#     return alpha_optimal

# def plotAirfoilPolars(airfoilName):
#     for polar in os.listdir("./airfoil/data"):
#         if airfoilName in polar:
#             df = pd.read_csv(f"./airfoil/data/{polar}", sep='\s+', skiprows=12, header=None)
#             df.columns = ["alpha", "CL", "CD", "CDp", "CM", "Top_Xtr", "Bot_Xtr"]
#             label = polar.split("Re")[1].split(".txt")[0]
#             print(f"Plotting {airfoilName} at Re = {label}")
#             plt.plot(df["alpha"], df["CL"], label=f"Re = {label}")
#     plt.xlabel("Alpha")
#     plt.legend()
#     plt.ylabel("CL")
#     plt.show()

# def interpolate_Re_polar(Re, airfoil_name):
#     Re_files = []
#     for polar in os.listdir("./airfoil/data"):
#         if airfoil_name in polar:
#             Re_files.append(float(polar.split("Re")[1].split(".txt")[0]))
#     Re_files = np.array(Re_files)
#     Re_files.sort()


# class PolarDatabase:
#     def __init__(self, folder_path):
#         self.data = {}  # {airfoil: {Re: {'alpha': ..., 'Cl': ..., 'Cd': ...}}}
#         self.interpolators = {}  # {airfoil: {'Cl': ..., 'Cd': ..., 'bounds': ...}}
#         self._load_polars(folder_path)
#         self._build_all_interpolators()

#     def _parse_filename(self, filename):
#         name_part = filename.split("Re")
#         airfoil = name_part[0]
#         Re_str = name_part[1].split(".txt")[0]
#         Re = float(Re_str)
#         return airfoil, Re

#     def _load_polars(self, folder_path):
#         for filename in os.listdir(folder_path):
#             if not filename.endswith('.txt') or 'Re' not in filename:
#                 continue
#             airfoil, Re = self._parse_filename(filename)
#             file_path = os.path.join(folder_path, filename)
#             try:
#                 data = np.loadtxt(file_path, skiprows=12)
#                 alpha, Cl, Cd = data[:, 0], data[:, 1], data[:, 2]
#                 if airfoil not in self.data:
#                     self.data[airfoil] = {}
#                 self.data[airfoil][Re] = {'alpha': alpha, 'Cl': Cl, 'Cd': Cd}
#             except Exception as e:
#                 print(f"Failed to load {filename}: {e}")

#     def _build_all_interpolators(self):
#         for airfoil, re_data in self.data.items():
#             Re_vals = sorted(re_data.keys())

#             # Overlapping alpha range
#             alpha_ranges = [re_data[Re]['alpha'] for Re in Re_vals]
#             alpha_min = max(alpha[0] for alpha in alpha_ranges)
#             alpha_max = min(alpha[-1] for alpha in alpha_ranges)

#             if alpha_max <= alpha_min:
#                 raise ValueError(f"No overlapping alpha range for airfoil '{airfoil}'")

#             alpha_common = np.linspace(alpha_min, alpha_max, 100)
#             Cl_array, Cd_array = [], []

#             for Re in Re_vals:
#                 alpha = re_data[Re]['alpha']
#                 Cl = re_data[Re]['Cl']
#                 Cd = re_data[Re]['Cd']

#                 Cl_interp = np.interp(alpha_common, alpha, Cl)
#                 Cd_interp = np.interp(alpha_common, alpha, Cd)

#                 Cl_array.append(Cl_interp)
#                 Cd_array.append(Cd_interp)

#             Cl_array = np.array(Cl_array)
#             Cd_array = np.array(Cd_array)

#             self.interpolators[airfoil] = {
#                 'Cl': RegularGridInterpolator((Re_vals, alpha_common), Cl_array, bounds_error=False, fill_value=None),
#                 'Cd': RegularGridInterpolator((Re_vals, alpha_common), Cd_array, bounds_error=False, fill_value=None),
#                 'bounds': {
#                     'Re': (Re_vals[0], Re_vals[-1]),
#                     'alpha': (alpha_common[0], alpha_common[-1])
#                 }
#             }

#     def get_cl_cd(self, airfoil, Re_query, alpha_query):
#         if airfoil not in self.interpolators:
#             raise ValueError(f"Airfoil '{airfoil}' not found.")

#         Re_query = np.atleast_1d(Re_query)
#         alpha_query = np.atleast_1d(alpha_query)

#         if Re_query.shape != alpha_query.shape:
#             raise ValueError("Re_query and alpha_query must have the same shape.")

#         bounds = self.interpolators[airfoil]['bounds']
#         Re_min, Re_max = bounds['Re']
#         alpha_min, alpha_max = bounds['alpha']

#         Cl_results = []
#         Cd_results = []

#         for i, (Re_val, alpha_val) in enumerate(zip(Re_query, alpha_query)):
#             within_re_bounds = Re_min <= Re_val <= Re_max
#             within_alpha_bounds = alpha_min <= alpha_val <= alpha_max

#             # if not within_re_bounds:
#             #     print(f"[ID {i}] Re = {Re_val:.2f} is out of bounds [{Re_min}, {Re_max}]")

#             # if not within_alpha_bounds:
#             #     print(f"[ID {i}] Alpha = {alpha_val:.2f} is out of bounds [{alpha_min}, {alpha_max}]")

#             if within_re_bounds and within_alpha_bounds:
#                 point = (Re_val, alpha_val)
#                 Cl = self.interpolators[airfoil]['Cl'](point)
#                 Cd = self.interpolators[airfoil]['Cd'](point)
#             else:
#                 available_res = sorted(self.data[airfoil].keys())
#                 closest_re = min(available_res, key=lambda r: abs(r - Re_val))
#                 alpha_vals = self.data[airfoil][closest_re]['alpha']
#                 Cl_vals = self.data[airfoil][closest_re]['Cl']
#                 Cd_vals = self.data[airfoil][closest_re]['Cd']

#                 closest_idx = np.argmin(np.abs(alpha_vals - alpha_val))
#                 Cl = Cl_vals[closest_idx]
#                 Cd = Cd_vals[closest_idx]

#             Cl_results.append(float(Cl))
#             Cd_results.append(float(Cd))

#         return np.array(Cl_results).reshape(-1, 1), np.array(Cd_results).reshape(-1, 1)




#     def get_bounds(self, airfoil):
#         if airfoil not in self.interpolators:
#             raise ValueError(f"Airfoil '{airfoil}' not found.")
#         return self.interpolators[airfoil]['bounds']
# # directory = "./airfoil/data"
# # db = PolarDatabase(directory)
# # Re_vec = np.array([300_000.0, 400_000.0, 250_000.0])
# # alpha_vec = np.array([10.0, 12.5, 20.0])  # last one might be out of bounds

# # Cl, Cd = db.get_cl_cd("a18sm", Re_vec, alpha_vec)
# # print("Cl =", Cl)
# # print("Cd =", Cd)

# # Re = np.linspace(10_000, 5_000_000, 30)
# # genDataBase(airfoil_name, Re, max_runtime)
# #plotAirfoilPolars("NACA 0012")

# # provided Re as a single number and airfoil name, choose 2 closest polar files and interpolate the data

# # def getClosestRePolar(Re, airfoil_name):
# #     Re_files = []
# #     for polar in os.listdir("./airfoil/data"):
# #         if airfoil_name in polar:
# #             Re_files.append(float(polar.split("Re")[1].split(".txt")[0]))
# #     Re_files = np.array(Re_files)
# #     Re_files.sort()

# #     closest_re = Re_files[np.argmin(np.abs(Re_files - Re))]
# #     df = pd.read_csv(f"./airfoil/data/{airfoil_name}Re{closest_re}.txt", sep='\s+', skiprows=12, header=None)
# #     df.columns = ["alpha", "CL", "CD", "CDp", "CM", "Top_Xtr", "Bot_Xtr"]
# #     return df
# #getClosestRePolar(137_000, "NACA 0012")