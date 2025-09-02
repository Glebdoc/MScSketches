from scipy.optimize import minimize
from main import main, loadConfig
import os, json
import numpy as np
import csv


PATH = 'configs/base.json'
MTOW = 60 
"""
The objective is to produce the given amount of thrust with minimum power consumption.


Constraints: 
1. Counteract main blade torque with tip-propeller thrust 
   For every new RPM_main there should be a good guess of RPM_tip that will counteract the torque.

   To estimate the torque we can use a simple momentum theory approach. 

   Cp = Cp_i + Cp_0 = (k Ct^(3/2)) / sqrt(2) + sigma Cd0 / 8 
   Therefore P = 0.5*Cp*rho*A*(omega*R)^3
   Torque = P/omega

   Therefore the required thrust per tip engine is (Torque / (R_tip))/NB 

   Knowing the required Thrust and the flow speed which is Omega_main*R_main we can estimate the 
   required RPM_tip using actuator disk theory 

   T = Ct*rho n^2 D^4 
   n = sqrt(T/(Ct*rho*D^4))

   but I still need to estimate Ct.

   RPM_tip = 60*n

Variables: 


"""
def modifyConfig(x):
   config = loadConfig(PATH)
   # Update config: 
   config["main_propeller"]["diameter"] = 2*x[0]
   config["small_propellers"]["diameter"] = x[1]

   # Save back to JSON
   with open(PATH, 'w') as f:
      json.dump(config, f, indent=4)

   return config

def thrustPenalty(T, MTOW, p=10): 
   return 1/np.exp(T - MTOW) ** p

R_min = 0.85
R_max = 1.0

D_min = 0.14
D_max = 0.16

def optimization():

   log_file = "optimization_log.csv"

   # Remove old log if exists
   # ask user to confirm
   remove = input(f"Remove old log file {log_file}? (y/n): ")
   if remove and os.path.exists(log_file):
      os.remove(log_file)

   # Write header
   with open(log_file, "w", newline="") as f:
      writer = csv.writer(f)
      writer.writerow(["iter", "R", "D", "thrust", "RPM_main", "RPM_small", "power_req", "Stall err", "err"])

   # This is a simple test for 2 variables R_main and D_small


   bounds = [(R_min, R_max), (D_min, D_max)]
   x_0 = [1.0, 0.15]
   iter = [0]

   def objective(x):
      config = modifyConfig(x)
      thrust, power_req, RPM_main, RPM_small, STALL_FACTOR = main(config="base.json")
      penalty = thrustPenalty(thrust, MTOW)
      err = power_req +  penalty + STALL_FACTOR*1000
      print("R_main ", x[0], "D_small ", x[1], "Err ", err, "RPM_main ", RPM_main, "RPM_small ", RPM_small, )
      # log to csv
      with open(log_file, "a", newline="") as f:
         writer = csv.writer(f)
         writer.writerow([iter[0], x[0], x[1], thrust, RPM_main, RPM_small, power_req, STALL_FACTOR ,err])
      iter[0]+=1
      return err 

   res = minimize(objective, x_0, bounds=bounds, method='Powell' )
   print("Optimization result: ", res)

# design space investigation 
import matplotlib.pyplot as plt
# build a design map from R and D 


def design_map():
   r = np.linspace(R_min, R_max, 3)
   d = np.linspace(D_min, D_max, 3)

   R, D = np.meshgrid(r, d)
   Z = np.zeros(R.shape)
   for i in range(R.shape[0]):
      for j in range(R.shape[1]):
         x = [R[i,j], D[i,j]]
         config = modifyConfig(x)
         thrust, power_req, RPM_main, RPM_small, STALL_FACTOR = main(config="base.json")
         Z[i,j] = power_req
         print("R_main ", x[0], "D_small ", x[1], "Power Req ", power_req, "RPM_main ", RPM_main, "RPM_small ", RPM_small, )

         # LOG to csv
         with open("design_map_log.csv", "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([x[0], x[1], thrust, RPM_main, RPM_small, power_req, STALL_FACTOR])
   
   fig = plt.figure()
   ax = fig.add_subplot(111, projection='3d')
   ax.plot_surface(R, D, Z, cmap='viridis')
   plt.xlabel('R_main (m)')
   plt.ylabel('D_small (m)')
   plt.title('Power Requirement (W)')
   plt.show()

#design_map()

optimization()