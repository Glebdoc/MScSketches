from scipy.optimize import minimize
from main import main, loadConfig, nested_optimization
import matplotlib.pyplot as plt
import os, json
import numpy as np
import csv
import plotter as myPlt


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
def modifyConfig(key, key2, value, savePath=None):
   config = loadConfig(PATH)
   # Update config: 
   config[key][key2] = value

   # Save back to JSON

   if savePath != None:
      with open(savePath, 'w') as f:
         json.dump(config, f, indent=4)


def thrustPenalty(T, MTOW, p=10): 
   return 1/np.exp(T - MTOW) ** p

R_min = 0.9
R_max = 1.1

D_min = 0.14
D_max = 0.16

def optimization():

   log_file = "optimization_log_actual.csv"

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
      RPM_MAIN, RPM_SMALL = nested_optimization(None, results=True)
      thrust, torque, power_required, createdMoment, STALL_FACTOR, iterVel = main(RPM_MAIN, RPM_SMALL, config=None)
      torque_penalty = abs(torque - createdMoment) * 1000
      thrust_penalty = abs(thrust - MTOW) * 100
      err = power_required + STALL_FACTOR*1000 + torque_penalty + thrust_penalty

      with open(log_file, "a", newline="") as f:
         writer = csv.writer(f)
         writer.writerow([iter[0], x[0], x[1], thrust, RPM_MAIN, RPM_SMALL, power_required, STALL_FACTOR, err])
      
      return err 

   res = minimize(objective, x_0, bounds=bounds, method='Powell' )
   print("Optimization result: ", res)


def design_map(refine=3):
   r = np.linspace(R_min, R_max, refine)
   d = np.linspace(D_min, D_max, refine)
   # r = np.array([[1.1]])
   # d = np.array([[0.16]])

   x_0 = [400, 10000]  # Initial guess for RPM_MAIN and RPM_SMALL
   bounds = [(250, 650), (4000, 15000)]  # Bounds for RPM_MAIN and RPM_SMALL

   with open("design_map.csv", "w", newline="") as f:
      writer = csv.writer(f)
      writer.writerow(["R_main (m)", "D_small (m)", "Thrust (N)", "RPM_main", "RPM_small", "Torque (Nm)", "Created Moment (Nm)", "Power Required (W)", "STALL_FACTOR"])

   R, D = np.meshgrid(r, d)
   Z = np.zeros(R.shape)
   for i in range(R.shape[0]):
      for j in range(R.shape[1]):
         x = [R[i,j], D[i,j]]
         print(f"Running for R={x[0]}, D={x[1]}")
         config = modifyConfig('main_propeller', 'radius', x[0])
         RPM_MAIN, RPM_SMALL, thrust, torque, power_required, createdMoment, STALL_FACTOR = nested_optimization(x_0=x_0, bounds=bounds, config=None, results=True)
         Z[i,j] = power_required
         with open("design_map_simplified_angle AH-79100B.csv", "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([x[0], x[1], thrust, RPM_MAIN, RPM_SMALL, torque, createdMoment,  power_required, STALL_FACTOR])
   
   fig = plt.figure()
   ax = fig.add_subplot(111, projection='3d')
   ax.plot_surface(R, D, Z, cmap='viridis')
   plt.xlabel('R_main (m)')
   plt.ylabel('D_small (m)')
   plt.title('Power Requirement (W)')
   plt.show()


def plot_the_field(filename, refinement=5):
   plt.close()
   fig = plt.figure(figsize=(8,6))
   ax = fig.add_subplot(111, projection='3d')
   labels = np.loadtxt(filename, delimiter=',', max_rows=1, dtype=str)
   print(labels)
   data = np.loadtxt(filename, delimiter=',', skiprows=1, usecols=(0,1,7))
   R = data[:,0]
   r_mesh = R.reshape((refinement,refinement))
   D = data[:,1]
   d_mesh = D.reshape((refinement,refinement))
   P = data[:,2]
   p_mesh = P.reshape((refinement,refinement))
   ax.plot_surface(r_mesh, d_mesh, p_mesh, cmap='viridis', )
   
   plt.xlabel('R_main (m)')
   plt.ylabel('D_small (m)')
   plt.title('Power Required (W)')
   plt.show(block=True)
   #plt.savefig('design_map.png', dpi=300)


def _imshow(filename):
   # load csv file and print it
   plt.close()
   labels = np.loadtxt(filename, delimiter=',', max_rows=1, dtype=str)
   print(labels)
   data = np.loadtxt(filename, delimiter=',', skiprows=0, usecols=(0,1,2,3,4,5,6,7))
   R = data[:,0]
   D = data[:,1]
   T = data[:,2]
   RPM_main = data[:,3]
   RPM_small = data[:,4]
   Torque = data[:,5]
   Created_Moment = data[:,6]
   Power_Required = data[:,7]

   mesh = np.meshgrid(np.unique(R), np.unique(D))

   # reshape all arrays to mesh shape
   R = R.reshape(mesh[0].shape)
   D = D.reshape(mesh[0].shape)
   T = T.reshape(mesh[0].shape)
   RPM_main = RPM_main.reshape(mesh[0].shape)
   RPM_small = RPM_small.reshape(mesh[0].shape)
   Torque = Torque.reshape(mesh[0].shape)
   Created_Moment = Created_Moment.reshape(mesh[0].shape)
   Power_Required = Power_Required.reshape(mesh[0].shape)

   # check if converged on created moment. Have a tolerance of 5%
   converged_m = np.abs(Torque - Created_Moment) < 0.03 * np.abs(Torque)
   #check if converget on thrust
   converged_t = (np.abs(T - MTOW) < 0.03 * MTOW)
   # combine both, if either is false then not converged, super
   converged = converged_m & converged_t




   # filter out non converged points
   Power_Required = np.where(converged, Power_Required, np.nan)

   plt.figure(figsize=(8,6))
   plt.imshow(Power_Required, extent=(R_min, R_max, D_min, D_max), origin='lower', aspect=0.2/0.02, cmap='viridis')
   plt.colorbar(label='Power Required (W)')
   plt.xlabel('R_main (m)')
   plt.ylabel('D_small (m)')
   plt.title('Power Required (W) - Non converged points are blank')
   plt.show(block=True)


#design_map(refine=4)


#
#design_map()
#_imshow("design_map_simplified_angle 80.csv")


"""
File structure for optimization log:
 design space/
         batch_1/
            optimization_log.csv
            DP1/
               inner_optimization_log.csv
               plots.png 
               config.json
            DP2/
               inner_optimization_log.csv
               plots.png 
               config.json
"""


def naccelle_exploration(batch, key1, key2, angle_1, angle_2, steps):

   # creatre directory 

   x_0 = [400, 10000]  # Initial guess for RPM_MAIN and RPM_SMALL
   bounds = [(250, 650), (4000, 25000)]  # Bounds for RPM_MAIN and RPM_SMALL

   batch_path = f'./DesignSpace/{batch}'
   os.makedirs(batch_path, exist_ok=True)

   angles = np.linspace(angle_1, angle_2, steps)
   for i in range(len(angles)):

      path =f'./DesignSpace/{batch}/DP{i}'
      os.makedirs(path, exist_ok=True)

      value = angles[i]
      filename = f'/DP{i}_config.json'

      modifyConfig(key1, key2, value, savePath=path+filename)

      RPM_MAIN, RPM_SMALL, thrust, torque, power_required, createdMoment, STALL_FACTOR = nested_optimization(config=path+filename, x_0=x_0, bounds=bounds, results=True, saveInnerLog=path, savePlots=True)


#naccelle_exploration('blade1', 'settings', 'blade1_angle', 60, 90, 4)


# blade1 angle exploration with fixed RPMs 

def propellerAzimuthalStudy(batch, key1, key2, angle_1, angle_2, steps, updatedata=False, twist=False):
   # from one of the converged design points 
   plt.close()
   # 388.8671875,8495.767630161266
   RPM_MAIN = 389
   RPM_SMALL = 8495

   batch_path = f'./DesignSpace/{batch}'
   os.makedirs(batch_path, exist_ok=True)
   angles = np.linspace(angle_1, angle_2, steps)

   # define the plot 3 subplots 
   if twist:
      fig, axs = plt.subplots(1,4, figsize=(12, 8))
      axs[3].set_title('Twist Distribution')
      
   else:
      fig, axs = plt.subplots(1,3, figsize=(9, 8))
   myPlt.set_bw_design()
   if updatedata:
      for i in range(len(angles)):

         path =f'./DesignSpace/{batch}/DP{i}'
         os.makedirs(path, exist_ok=True)

         value = angles[i]
         filename = f'/DP{i}_config.json'

         modifyConfig(key1, key2, value, savePath=path+filename)

         thrust, torque, power_required, createdMoment, STALL_FACTOR, iterVel = main(RPM_MAIN, RPM_SMALL, config=path+filename, savePath=path)
   
   # plot all the results in one figure
   alpha = np.linspace(0.1, 1, len(angles))
   for i in range(len(angles)):
      path =f'./DesignSpace/{batch}/DP{i}'
      data = np.loadtxt(f'{path}/_res.csv', delimiter=',', skiprows=1)
      if i%1==0:
         label = f'{angles[i]:.1f} deg'
      else:
         label = None
      
      myPlt.plot_AoA_perBlade(fig, axs, data, alpha[i], label)
      if twist:
         myPlt.plot_twist(fig, axs[3], data, alpha[i], label)

   
   ymin = -5
   ymax = 15
   count = 0
   for ax in axs:
      
      ax.grid(True)

      # Labels
      ax.set_ylabel('Angle of Attack (deg)')
      ax.set_xlabel('Radial position (m)')

      # Background color
      ax.set_facecolor('#f5f5f5')   # light gray, or pick any color

      # Legend
      if count!=3:
         ax.set_ylim(ymin, ymax)
         ax.set_aspect(1/20)
         ax.legend(loc='upper right', fontsize=9, frameon=True)
      else:
         ax.set_ylabel('Twist (deg)')
         ax.set_ylim(0, 90)
         ax.set_aspect(1/100)
         ax.legend(loc='upper right', fontsize=9, frameon=True)
      count+=1
   #plt.savefig(f'./DesignSpace/{batch}/plot.png', dpi=300)
   plt.show(block=True)
start = 0 
splits = 15 
end = 120 
#propellerAzimuthalStudy('propeller_azimuth_8032_5_AH-791', 'settings', 'blade1_angle', start, end, splits, updatedata=False, twist=True)

#propellerAzimuthalStudy('propeller_azimuth_8035', 'settings', 'blade1_angle', start, end, splits, updatedata=False, twist=True)
# propellerAzimuthalStudy('propeller_pitch', 'settings', 'pitch_incline', -500, -250, 5, updatedata=True, twist=True)
#propellerAzimuthalStudy('propeller_tipPitch', 'small_propellers', 'pitch_tip', 35, 25, 5, updatedata=False, twist=True)
#propellerAzimuthalStudy('propeller_rootPitch', 'small_propellers', 'pitch_root', 75, 85, 5, updatedata=True, twist=True)
#propellerAzimuthalStudy('propeller_azimuth_8032_5', 'settings', 'blade1_angle', start, end, splits, updatedata=False, twist=True)
#propellerAzimuthalStudy('propeller_azimuth_8032_5_AH-791', 'settings', 'blade1_angle', start, end, splits, updatedata=False, twist=True)


import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def propellerAzimuthalAnimation(
    batch, key1, key2, angle_1, angle_2, steps,
    updatedata=False, twist=False,
    save_anim=None, fps=8, trail=3
):
    """
    Animate AoA distribution per blade as azimuth changes.
    trail = number of previous frames to keep visible with fading opacity.
    """

    plt.close('all')
    RPM_MAIN, RPM_SMALL = 389, 8495
    batch_path = f'./DesignSpace/{batch}'
    os.makedirs(batch_path, exist_ok=True)
    angles = np.linspace(angle_1, angle_2, steps)

    if updatedata:
        for i, value in enumerate(angles):
            dp_dir = f'{batch_path}/DP{i}'
            os.makedirs(dp_dir, exist_ok=True)
            cfg_path = f'{dp_dir}/DP{i}_config.json'
            modifyConfig(key1, key2, float(value), savePath=cfg_path)
            _ = main(RPM_MAIN, RPM_SMALL, config=cfg_path, savePath=dp_dir)

    # preload all data
    frames = []
    for i in range(len(angles)):
        dp_dir = f'{batch_path}/DP{i}'
        arr = np.loadtxt(f'{dp_dir}/_res.csv', delimiter=',', skiprows=1)
        frames.append(arr)

    # figure setup
    if twist:
        fig, axs = plt.subplots(1,4, figsize=(12, 8))
        axs[3].set_title('Twist Distribution')
    else:
        fig, axs = plt.subplots(1,3, figsize=(9, 8))
    myPlt.set_bw_design()
    title = fig.suptitle(f"Azimuth = {angles[0]:.1f}°", fontsize=14)

    # storage for keeping last few plotted lines
    history = []

    def update(frame_idx):
        arr = frames[frame_idx]
        angle = angles[frame_idx]

        # append this frame to history
        history.append((arr, angle))
        if len(history) > trail:
            history.pop(0)  # keep only last 'trail' frames

        # clear axes
        for ax in axs:
            ax.cla()

        # replot all history frames with fading opacity
        for j, (arr_h, ang_h) in enumerate(history):
            alpha = (j+1)/len(history)  # fade older frames
            label = f'{ang_h:.1f} deg' if j == len(history)-1 else None
            myPlt.plot_AoA_perBlade(fig, axs, arr_h, alpha, label)
            if twist:
                myPlt.plot_twist(fig, axs[3], arr_h, alpha, label)

        # cosmetics
        for j, ax in enumerate(axs):
            ax.grid(True)
            ax.set_facecolor('#f5f5f5')
            ax.set_xlabel('Radial position (m)')
            if twist and j==3:
                ax.set_ylabel('Twist (deg)')
                ax.set_ylim(0, 90)
                ax.set_aspect(1/100)
            else:
                ax.set_ylabel('Angle of Attack (deg)')
                ax.set_ylim(-5, 15)
                ax.set_aspect(1/20)
            ax.legend(loc='upper right', fontsize=9, frameon=True)

        title.set_text(f"Azimuth = {angle:.1f}°")
        return axs

    ani = FuncAnimation(
        fig, update, frames=len(frames), interval=1000/fps, blit=False
    )

    if save_anim:
        ext = os.path.splitext(save_anim)[1].lower()
        if ext == ".mp4":
            ani.save(save_anim, writer='ffmpeg', fps=fps, dpi=150)
        elif ext == ".gif":
            ani.save(save_anim, writer='pillow', fps=fps)
        else:
            print("Unsupported format. Use .mp4 or .gif.")
    else:
        plt.show(block=True)


propellerAzimuthalAnimation(
    'propeller_azimuth_8032_5_eppler_animation_full_capped',
    'settings', 'blade1_angle',
    0, 360, 50,
    updatedata=False,
    twist=True,
    save_anim="DesignSpace/prop_anim_full_12FPS_notrail_capped.mp4",
    fps=12,
    trail=1
)


#propellerAzimuthalStudy('propeller_azimuth_8032_eppler560_cordDist', 'settings', 'small_chord_a', -2, 5, 5, updatedata=True, twist=True)
# propellerAzimuthalStudy('propeller_azimuth_8032_AH-79100B_pitchtip', 'small_propellers', 'pitch_tip', 32.5, 38, 4, updatedata=True, twist=True)

#design_map(refine=3)