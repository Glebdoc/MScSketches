from scipy.optimize import minimize
from main import main, loadConfig, nested_optimization
import matplotlib.pyplot as plt
import os, json
import numpy as np
import csv
import plotter as myPlt
from bemUtils import computeVelocityField
from matplotlib.animation import FuncAnimation
import os, re
from pathlib import Path
import imageio.v2 as imageio


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
   RPM_MAIN = 340
   RPM_SMALL = 12000

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



def propellerAzimuthalAnimation(
    batch, key1, key2, angle_1, angle_2, steps,
    updatedata=False, twist=False,
    save_anim=None, fps=8, trail=3, black=False
):
    """
    Animate AoA distribution per blade as azimuth changes.
    trail = number of previous frames to keep visible with fading opacity.
    If black=True: black background + white lines.
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
        fig, axs = plt.subplots(1, 4, figsize=(12, 8))
        axs[3].set_title('Twist Distribution', color=('white' if black else None))
    else:
        fig, axs = plt.subplots(1, 3, figsize=(9, 8))

    # Use your BW design only for light theme
    if not black:
        myPlt.set_bw_design()

    # Dark theme cosmetics
    if black:
        fig.patch.set_facecolor('black')
        for ax in axs:
            ax.set_facecolor('black')
            # force white line color for anything plotted without an explicit color
            ax.set_prop_cycle(color=['white'])
            # subtle grid on black
            ax.grid(True, color='0.3', alpha=0.5)
        title = fig.suptitle(f"Azimuth = {angles[0]:.1f}°", fontsize=14, color='white')
    else:
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
            # re-apply background + cycler + grid after cla()
            if black:
                ax.set_facecolor('black')
                ax.set_prop_cycle(color=['white'])
                ax.grid(True, color='0.3', alpha=0.5)

        # replot all history frames with fading opacity
        for j, (arr_h, ang_h) in enumerate(history):
            alpha = (j + 1) / len(history)  # fade older frames
            label = f'{ang_h:.1f} deg' if j == len(history) - 1 else None
            myPlt.plot_AoA_perBlade(fig, axs, arr_h, alpha, label, black=black)
            if twist:
                myPlt.plot_twist(fig, axs[3], arr_h, alpha, label)

        # cosmetics per-axis
        for j, ax in enumerate(axs):
            if not black:
                ax.grid(True)
                ax.set_facecolor('#f5f5f5')

            # labels
            if twist and j == 3:
                ax.set_ylabel('Twist (deg)', color=('white' if black else None))
                ax.set_ylim(0, 90)
                ax.set_aspect(1 / 100)
            else:
                ax.set_ylabel('Angle of Attack (deg)', color=('white' if black else None))
                ax.set_ylim(-5, 15)
                ax.set_aspect(1 / 20)

            ax.set_xlabel('Radial position (m)', color=('white' if black else None))
            ax.tick_params(colors=('white' if black else None))

            # spines & legend styling
            if black:
                for sp in ax.spines.values():
                    sp.set_color('white')
                leg = ax.legend(loc='upper right', fontsize=9, frameon=True)
                if leg:
                    leg.get_frame().set_facecolor('black')
                    leg.get_frame().set_edgecolor('white')
                    for txt in leg.get_texts():
                        txt.set_color('white')
            else:
                ax.legend(loc='upper right', fontsize=9, frameon=True)

        title.set_text(f"Azimuth = {angle:.1f}°")
        if black:
            title.set_color('white')
        return axs

    ani = FuncAnimation(
        fig, update, frames=len(frames), interval=1000 / fps, blit=False
    )

    if save_anim:
        ext = os.path.splitext(save_anim)[1].lower()
        if ext == ".mp4":
            # ensure the saved video keeps the black figure facecolor
            ani.save(save_anim, writer='ffmpeg', fps=fps, dpi=150,
                     savefig_kwargs={'facecolor': fig.get_facecolor()})
        elif ext == ".gif":
            ani.save(save_anim, writer='pillow', fps=fps,
                     savefig_kwargs={'facecolor': fig.get_facecolor()})
        else:
            print("Unsupported format. Use .mp4 or .gif.")
    else:
        plt.show(block=True)

# propellerAzimuthalAnimation(
#     'propeller_azimuth_8032_5_eppler_animation_full_capped',
#     'settings', 'blade1_angle',
#     0, 360, 50,
#     updatedata=False,
#     twist=False,
#     save_anim="DesignSpace/prop_anim_full_12FPS_notrail_capped_black.gif",
#     fps=12,
#     trail=1,
#     black=True
# )


#propellerAzimuthalStudy('propeller_azimuth_8032_eppler560_cordDist', 'settings', 'small_chord_a', -2, 5, 5, updatedata=True, twist=True)
#propellerAzimuthalStudy('propeller_azimuth_8032_AH-79100B_pitchtip', 'small_propellers', 'pitch_tip', 32.5, 38, 2, updatedata=True, twist=True)

#design_map(refine=3)

def circular_patch(center, radius, n_r=30, n_theta=100, plane='YZ', return_edges=False):
    """
    Create a circular patch mesh (structured grid in polar coords) in a specified plane.
    
    Parameters
    ----------
    center : tuple of float
        (x0, y0, z0) coordinates of the circle center.
    radius : float
        Circle radius.
    n_r : int
        Radial resolution (number of points in r).
    n_theta : int
        Angular resolution (number of points in theta).
    plane : str
        Plane of the circle: 'YZ' or 'XY'.
    return_edges : bool
        If True, also return edge grids (for visualization/meshing).
    """

    x0, y0, z0 = center
    # centers
    r  = np.linspace(0, radius, n_r)
    th = np.linspace(0, 2*np.pi, n_theta, endpoint=False)
    R, TH = np.meshgrid(r, th, indexing='ij')

    if plane == 'YZ':
        X = np.full_like(R, x0)
        Y = y0 + R*np.cos(TH)
        Z = z0 + R*np.sin(TH)

    elif plane == 'XY':
        X = x0 + R*np.cos(TH)
        Y = y0 + R*np.sin(TH)
        Z = np.full_like(R, z0)
    
    elif plane == 'XZ':
        X = x0 + R*np.cos(TH)
        Y = np.full_like(R, y0)
        Z = z0 + R*np.sin(TH)

    else:
        raise NotImplementedError("Only 'YZ' and 'XY' planes are implemented")

    if not return_edges:
        return X, Y, Z

    # edges (one more in each direction)
    r_e  = np.linspace(0, radius, n_r+1)
    th_e = np.linspace(0, 2*np.pi, n_theta+1)  # include 2π to close the seam
    R_e, TH_e = np.meshgrid(r_e, th_e, indexing='ij')

    if plane == 'YZ':
        X_e = np.full_like(R_e, x0)
        Y_e = y0 + R_e*np.cos(TH_e)
        Z_e = z0 + R_e*np.sin(TH_e)

    elif plane == 'XY':
        X_e = x0 + R_e*np.cos(TH_e)
        Y_e = y0 + R_e*np.sin(TH_e)
        Z_e = np.full_like(R_e, z0)
    
    elif plane == 'XZ':
        X_e = x0 + R_e*np.cos(TH_e)
        Y_e = np.full_like(R_e, y0)
        Z_e = z0 + R_e*np.sin(TH_e)

    return (X, Y, Z), (X_e, Y_e, Z_e)




def velocity_field_around_propeller(batch, steps, center, radius, plane,
                                    variable='mag', title=None, dark_mode=False):

    batch_path = f'./DesignSpace/{batch}'

    # Create batch-level "Fields" subfolder
    fields_path = os.path.join(batch_path, "Fields")
    os.makedirs(fields_path, exist_ok=True)

    # Build circular grid once
    (Xc, Yc, Zc), (Xe, Ye, Ze) = circular_patch(
        center=(center[0], center[1], center[2]),
        radius=radius,
        n_r=90, n_theta=360,
        plane=plane,
        return_edges=True
    )
    pts = np.column_stack((Xc.ravel(), Yc.ravel(), Zc.ravel()))

    for i in range(steps):
        dp_dir = os.path.join(batch_path, f'DP{i}')
        data = np.loadtxt(os.path.join(dp_dir, 'table_final.txt'))
        gammas = np.loadtxt(os.path.join(dp_dir, '_gammas.txt'))

        # Compute velocities
        u, v, w = computeVelocityField(data, pts, gammas, plane=plane)
        w = w  # Invert Z velocity if needed
        U = u.reshape(Xc.shape)
        V = v.reshape(Xc.shape)
        W = w.reshape(Xc.shape)
        mag = np.sqrt(U**2 + V**2 + W**2)

        if variable.lower() == 'u':
            mag = U
        elif variable.lower() == 'v':
            mag = V
        elif variable.lower() == 'w':
            mag = W
        elif variable.lower() == 'mag':
            pass
        else:
            raise ValueError(f"Unsupported variable: {variable}. Use 'U', 'V', 'W', or 'mag'")

        # Create plot
        fig, ax = plt.subplots(figsize=(7, 6))

        # Switch to dark background if required
        if dark_mode:
            fig.patch.set_facecolor('black')
            ax.set_facecolor('black')
            ax.tick_params(colors='white')
            for spine in ax.spines.values():
                spine.set_color('white')
            text_color = 'white'
        else:
            text_color = 'black'

        # Plot depending on plane
        if plane == 'YZ':
            pcm = ax.pcolormesh(Ye, Ze, mag, shading='auto', cmap='jet')
            fig.colorbar(pcm, ax=ax, label='Velocity Magnitude (m/s)')

            step = 3
            ax.quiver(Yc[::step, ::step], Zc[::step, ::step],
                      V[::step, ::step], W[::step, ::step],
                      scale=None, width=0.002, color=text_color)

            ax.set_xlabel('Y (m)', color=text_color)
            ax.set_ylabel('Z (m)', color=text_color)
            ax.set_title(f'YZ velocity slice (DP {i})', color=text_color)

        elif plane == 'XY':
            pcm = ax.pcolormesh(Xe, Ye, mag, shading='auto', cmap='jet')
            fig.colorbar(pcm, ax=ax, label='Velocity Magnitude (m/s)')

            step = 3
            ax.quiver(Xc[::step, ::step], Yc[::step, ::step],
                      U[::step, ::step], V[::step, ::step],
                      scale=None, width=0.002, color=text_color)

            ax.set_xlabel('X (m)', color=text_color)
            ax.set_ylabel('Y (m)', color=text_color)
            ax.set_title(f'XY velocity slice (DP {i})', color=text_color)

        elif plane == 'XZ':
            pcm = ax.pcolormesh(Xe, Ze, mag, shading='auto', cmap='jet')
            fig.colorbar(pcm, ax=ax, label='Velocity Magnitude (m/s)')

            step = 3
            ax.quiver(Xc[::step, ::step], Zc[::step, ::step],
                      U[::step, ::step], W[::step, ::step],
                      scale=None, width=0.002, color=text_color)

            ax.set_xlabel('X (m)', color=text_color)
            ax.set_ylabel('Z (m)', color=text_color)
            ax.set_title(f'XZ velocity slice (DP {i})', color=text_color)

        else:
            raise ValueError(f"Unsupported plane: {plane}. Use 'XY', 'YZ', or 'XZ'")

        ax.set_aspect('equal')

        # Save into Fields folder
        suffix = "dark" if dark_mode else "light"
        out_png = os.path.join(fields_path, f'DP{i}_{plane}_field_{title}_{suffix}.png')
        fig.savefig(out_png, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
        plt.close(fig)

        print(f"Saved {out_png}")




      #u, v, w = computeVelocityField(plane='YZ', shift=0, discretization=20, plotting=True)



def stitch_velocity_fields(batch: str,
                           out_name: str = "velocity_fields.gif",
                           fps: int = 8,
                           loop: int = 0):
    """
    Stitch PNGs in ./DesignSpace/{batch}/Fields/ into a GIF.

    Args:
        batch: batch folder name under ./DesignSpace/
        out_name: name of the output GIF file (default: velocity_fields.gif)
        fps: frames per second
        loop: number of times the GIF should loop (0 = infinite)
    """
    batch_path = Path("./DesignSpace") / batch
    fields_path = batch_path / "Fields"
    assert fields_path.is_dir(), f"Fields folder not found: {fields_path}"

    # collect and sort PNGs
    def dp_index(path: Path):
        m = re.search(r'DP(\d+)', path.stem)
        return int(m.group(1)) if m else 1e9

    frames = sorted(fields_path.glob("DP*_field.png"), key=dp_index)
    if not frames:
        raise FileNotFoundError(f"No PNGs found in {fields_path}")

    out_path = fields_path / out_name
    with imageio.get_writer(out_path, mode="I", duration=1.0/fps, loop=loop) as writer:
        for f in frames:
            writer.append_data(imageio.imread(f))
    print(f"Saved GIF: {out_path.resolve()} ({len(frames)} frames @ {fps} fps)")

#propellerAzimuthalStudy('azimuth_for_field', 'settings', 'blade1_angle', 0, 360, 100, updatedata=True, twist=False)
# RADIUS = 0.5
# VARIABLE = 'W'
# TITLE = 'full_W'
# CENTER = (0.0, 0.0, 0.0)
# DARKMODE = True
# # velocity_field_around_propeller('quad_simple', 1, center=(0.0, 0.0, 0.0), radius=RADIUS, plane='YZ', variable='W', title="W")
# # velocity_field_around_propeller('quad_simple', 1, center=(0.0, 0.0, 0.0), radius=RADIUS, plane='XZ', variable='W', title="W")
# # velocity_field_around_propeller('quad_simple', 1, center=(0.0, 0.0, 0.0), radius=RADIUS, plane='XY', variable='W', title="W")

# velocity_field_around_propeller('quad_simple', 1, center=CENTER, radius=RADIUS, plane='YZ', 
#                                 variable=VARIABLE, title=TITLE, dark_mode=DARKMODE)
# velocity_field_around_propeller('quad_simple', 1, center=CENTER, radius=RADIUS, plane='XZ', 
#                                 variable=VARIABLE, title=TITLE, dark_mode=DARKMODE)
# velocity_field_around_propeller('quad_simple', 1, center=CENTER, radius=RADIUS, plane='XY', 
#                                 variable=VARIABLE, title=TITLE, dark_mode=DARKMODE)

# RADIUS = 0.5
# VARIABLE = 'U'
# TITLE = 'full_U'
# CENTER = (0.0, 0.0, 0.0)
# velocity_field_around_propeller('quad_simple', 1, center=CENTER, radius=RADIUS, plane='YZ', 
#                                 variable=VARIABLE, title=TITLE, dark_mode=DARKMODE)
# velocity_field_around_propeller('quad_simple', 1, center=CENTER, radius=RADIUS, plane='XZ', 
#                                 variable=VARIABLE, title=TITLE, dark_mode=DARKMODE)
# velocity_field_around_propeller('quad_simple', 1, center=CENTER, radius=RADIUS, plane='XY', 
#                                 variable=VARIABLE, title=TITLE, dark_mode=DARKMODE)

# RADIUS = 0.5
# VARIABLE = 'V'
# TITLE = 'full_V'
# CENTER = (0.0, 0.0, 0.0)
# velocity_field_around_propeller('quad_simple', 1, center=CENTER, radius=RADIUS, plane='YZ', 
#                                 variable=VARIABLE, title=TITLE, dark_mode=DARKMODE)
# velocity_field_around_propeller('quad_simple', 1, center=CENTER, radius=RADIUS, plane='XZ', 
#                                 variable=VARIABLE, title=TITLE, dark_mode=DARKMODE)
# velocity_field_around_propeller('quad_simple', 1, center=CENTER, radius=RADIUS, plane='XY', 
#                                 variable=VARIABLE, title=TITLE, dark_mode=DARKMODE)

# stitch_velocity_fields(
#     batch="azimuth_for_field",
#     out_name="fields.gif",
#     fps=20
# )

def cache_velocity_fields(
    batch: str,
    steps: int,
    center=(-0.15, 1.0, 0.0),
    radius=0.15,
    n_r=30,
    n_theta=120,
    plane='YZ'
):
    """
    Build a circular grid once, compute velocities for each DP{i},
    and save results to ./DesignSpace/{batch}/Fields/ without plotting.

    Files written:
      Fields/grid_yz_disk.npz      -> Xc, Yc, Zc (centers), Xe, Ye, Ze (edges), meta
      Fields/DP{i}_field.npz       -> U, V, W (reshaped to grid), mag, meta
      Fields/meta.json             -> parameters for reference
    """
    batch_path = f'./DesignSpace/{batch}'
    fields_path = os.path.join(batch_path, "Fields")
    os.makedirs(fields_path, exist_ok=True)

    # ----- Build grid once -----
    (Xc, Yc, Zc), (Xe, Ye, Ze) = circular_patch(
        center=center, radius=radius,
        n_r=n_r, n_theta=n_theta,
        plane=plane, return_edges=True
    )
    pts = np.column_stack((Xc.ravel(), Yc.ravel(), Zc.ravel()))

    # Save grid once
    grid_meta = {
        "plane": plane, "center": tuple(center), "radius": float(radius),
        "n_r": int(n_r), "n_theta": int(n_theta)
    }
    np.savez_compressed(
        os.path.join(fields_path, "grid_yz_disk.npz"),
        Xc=Xc, Yc=Yc, Zc=Zc, Xe=Xe, Ye=Ye, Ze=Ze, meta=json.dumps(grid_meta)
    )

    # Also store a human-readable meta.json
    with open(os.path.join(fields_path, "meta.json"), "w") as f:
        json.dump({"grid": grid_meta, "steps": int(steps)}, f, indent=2)

    # ----- Compute & save per-DP velocities -----
    for i in range(steps):
        dp_dir = os.path.join(batch_path, f'DP{i}')
        data = np.loadtxt(os.path.join(dp_dir, 'table_final.txt'))

        # Compute velocities on the circular patch
        u, v, w = computeVelocityField(data, pts, plane=plane)  # returns flat arrays
        U = u.reshape(Xc.shape)
        V = v.reshape(Xc.shape)
        W = w.reshape(Xc.shape)
        mag = np.sqrt(U**2 + V**2 + W**2)

        # Save per-DP cache
        np.savez_compressed(
            os.path.join(fields_path, f"DP{i}_field.npz"),
            U=U, V=V, W=W, mag=mag,
            meta=json.dumps({"dp": i})
        )
        print(f"Saved cache: {os.path.join(fields_path, f'DP{i}_field.npz')}")

# cache_velocity_fields('azimuth_for_field', 
# 100)

def render_cached_fields_to_pngs(batch: str, out_dir: str = None,cmap_limits=None,
                                 quiver_step=3, vmin=0, vmax=10, cmap='jet'):
    base = Path(f'./DesignSpace/{batch}/Fields')
    grid = np.load(base / "grid_yz_disk.npz", allow_pickle=True)
    Xc, Yc, Zc, Xe, Ye, Ze = grid["Xc"], grid["Yc"], grid["Zc"], grid["Xe"], grid["Ye"], grid["Ze"]

    def dp_idx(p: Path):
        m = re.search(r'DP(\d+)_field\.npz', p.name)
        return int(m.group(1)) if m else 10**9

    frames = sorted(base.glob("DP*_field.npz"), key=dp_idx)
    out_dir = Path(out_dir or base)  # default into Fields/
    out_dir.mkdir(parents=True, exist_ok=True)

    for f in frames:
        d = np.load(f)
        U, V, W, mag = d["U"], d["V"], d["W"], d["mag"]

        fig, ax = plt.subplots(figsize=(7, 6))
        # --- black background ---
        fig.patch.set_facecolor('black')
        ax.set_facecolor('black')

        # field
        pcm = ax.pcolormesh(Ye, Ze, mag, shading='auto', cmap=cmap, vmin=vmin, vmax=vmax)
        cbar = fig.colorbar(pcm, ax=ax, label='Velocity Magnitude (m/s)')
        # colorbar to white text & outline, black bg
        cbar.ax.tick_params(colors='white')
        cbar.outline.set_edgecolor('white')
        cbar.set_label('Velocity Magnitude (m/s)', color='white')

        # quiver in white so it shows on black
        ax.quiver(Yc[::quiver_step, ::quiver_step], Zc[::quiver_step, ::quiver_step],
                  V[::quiver_step, ::quiver_step], W[::quiver_step, ::quiver_step],
                  color='white', scale=None, width=0.002)

        # labels, ticks, spines, title in white
        ax.set_xlabel('Y (m)', color='white')
        ax.set_ylabel('Z (m)', color='white')
        ax.set_title(f.stem.replace('_', ' '), color='white')
        ax.tick_params(colors='white')
        for spine in ax.spines.values():
            spine.set_color('white')

        ax.set_aspect('equal')

        out_png = out_dir / f.with_suffix('.png').name
        fig.savefig(out_png, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
        plt.close(fig)
        print("Wrote", out_png)

def stitch_cached_pngs_to_gif(batch: str, gif_name: str = "fields.gif", fps: int = 8):
    base = Path(f'./DesignSpace/{batch}/Fields')
    pngs = sorted(base.glob("DP*_field.png"), key=lambda p: int(re.search(r'DP(\d+)', p.stem).group(1)))
    gif_path = base / gif_name
    with imageio.get_writer(gif_path, mode="I", duration=1.0/fps, loop=0) as wr:
        for p in pngs:
            wr.append_data(imageio.imread(p))
    print("Saved GIF:", gif_path.resolve())


#cache_velocity_fields('azimuth_for_field', 100)
# render_cached_fields_to_pngs('azimuth_for_field', out_dir=None, cmap_limits=None, quiver_step=3, vmin=0, vmax=10)
#render_cached_fields_to_pngs('azimuth_for_field', out_dir=None, cmap_limits=None, quiver_step=3, vmin=0, vmax=10)
# stitch_cached_pngs_to_gif('azimuth_for_field', gif_name="fields_fixedscale_10ms.gif", fps=20)
#stitch_cached_pngs_to_gif('azimuth_for_field', gif_name="fields_fixedscale_10ms_black.gif", fps=20)

#propellerAzimuthalStudy('naccelle8090_constRPM', 'small_propellers', 'angle', 90, 90, 4, updatedata=True, twist=False)
#propellerAzimuthalStudy('azimuth_for_field', 'settings', 'blade1_angle', 0, 360, 100, updatedata=True, twist=False)
#propellerAzimuthalStudy('azimuth_for_field_sm15', 'settings', 'blade1_angle', 0, 360, 100, updatedata=True, twist=False)

#propellerAzimuthalAnimation(
#     'azimuth_new_dist_reFalse',
#     'settings', 'blade1_angle',
#     0, 360, 100,
#     updatedata=False,
#     twist=False,
#     save_anim="DesignSpace/azimuth_new_dist_reFalse_noItsTrue.gif",
#     fps=12,
#     trail=1,
#     black=True
# )

#propellerAzimuthalStudy('azimuth_new_dist_reFalse', 'settings', 'blade1_angle', 0, 360, 100, updatedata=True, twist=False)
#naccelle_exploration('main_chord_a_1m','settings', 'main_chord_a', -2, -1, 3)



def powerMap():

    """
    Key design variables for each configuration: 

        Drone 
            - diameter (dependent)
            - propeller diameter  
            - blade pitch
            - blade twist
            - airfoil main 
            - NB

        Helicopter 
            - diameter (dependent)
            - AR 
            - blade pitch
            - blade twist
            - airfoil 
            - NB 

        Quadcopter
            - diagonal  
            - rotor diameter (dependent)
            - blade pitch
            - blade twist 
            - airfoil 
            - AR             
    """