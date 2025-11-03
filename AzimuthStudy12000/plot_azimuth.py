import os
import json
import numpy as np
import matplotlib.pyplot as plt
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

# --- CONFIGURE THIS SECTION ---
base_dir = "./drone/"             # directory containing DP0, DP1, ..., DP120
underscore_key = "blade1_angle"   # key to extract from _.json -> settings[underscore_key] (degrees)
performance_key = "thrust_small"  # key to extract from performance.json (optional, collected but unused below)
# ------------------------------

dp_values = []
angles_deg = []         # angles per DP (degrees)
performance_values = [] # optional metric per DP
profiles_dT = []        # dT profiles for blade 1
profiles_alpha = []     # alpha profiles for blade 1
profiles_v_axial = []   # v_axial profiles for blade 1
r_ref = None            # common r grid

# Loop through DP0 → DP120
for i in range(121):
    folder_name = f"DP{i}"
    folder_path = os.path.join(base_dir, folder_name)

    if not os.path.isdir(folder_path):
        print(f"Skipping missing folder: {folder_name}")
        continue

    underscore_path  = os.path.join(folder_path, "_.json")
    performance_path = os.path.join(folder_path, "performance.json")
    data_path        = os.path.join(folder_path, "_res.npz")

    try:
        with open(underscore_path, "r", encoding="utf-8") as f:
            underscore_data = json.load(f)
        with open(performance_path, "r", encoding="utf-8") as f:
            performance_data = json.load(f)

        npz = np.load(data_path)
        data = npz['data']
        
        # Extract arrays
        f_axial = data["f_axial"]
        alpha = data["alpha"]
        v_axial = data["v_axial"]
        r = data["r"]

        # Extract sizes and slice for blade 1 only
        N_main = underscore_data["main_propeller"]["n"]
        n_small = underscore_data["small_propellers"]["n"]
        start = 3 * (N_main - 1)
        stop = start + n_small - 1
        
        dT_blade1 = f_axial[start:stop]
        alpha_blade1 = alpha[start:stop]
        v_axial_blade1 = -v_axial[start:stop]  # Flip sign
        r_case = r[start:stop+1]

        angles_deg.append(underscore_data["settings"][underscore_key])
        performance_values.append(performance_data.get(performance_key, np.nan))
        profiles_dT.append(dT_blade1)
        profiles_alpha.append(alpha_blade1)
        profiles_v_axial.append(v_axial_blade1)
        dp_values.append(i)

    except Exception as e:
        print(f"⚠️ Error reading from {folder_name}: {e}")

angles_deg = np.asarray(angles_deg)
angles_rad = np.deg2rad(angles_deg)
profiles_dT = np.asarray(profiles_dT)
profiles_alpha = np.asarray(profiles_alpha)
profiles_v_axial = np.asarray(profiles_v_axial)
r_centers = r_case

# Sort by angle
order = np.argsort(angles_rad)
angles_rad = angles_rad[order]
profiles_dT = profiles_dT[order, :]
profiles_alpha = profiles_alpha[order, :]
profiles_v_axial = profiles_v_axial[order, :]

# Transpose for pcolormesh (M=n_r, N=n_angles)
C_dT = profiles_dT.T
C_alpha = profiles_alpha.T
C_v_axial = profiles_v_axial.T

# Remove first angle and matching data
angles_rad = angles_rad[1:]
C_dT = C_dT[:, 1:]
C_alpha = C_alpha[:, 1:]
C_v_axial = C_v_axial[:, 1:]
r_centers = r_centers[:-1]

# Create 3 subplots for blade 1
fig, axes = plt.subplots(
    1, 3, figsize=(18, 7    ),
    subplot_kw={'projection': 'polar'}
)

# Data and labels
colormap = 'magma'
datasets = [C_dT, C_alpha, C_v_axial]
labels = ['dT [N/m]', r'$\alpha$ [deg]', r'$v_{axial}$ [m/s]']
cmaps = [colormap, colormap, colormap]

# Create theta grid (full circle)
theta_grid = np.linspace(0, 2*np.pi, len(angles_rad))

for i, (ax, data, label, cmap) in enumerate(zip(axes, datasets, labels, cmaps)):
    # Pcolormesh
    pc = ax.pcolormesh(theta_grid, r_centers, data, shading='gouraud', cmap=cmap)
    
    # Add contour lines
    contours = ax.contour(theta_grid, r_centers, data, levels=7, 
                          colors='white', linewidths=0.5, alpha=0.95, linestyles='--')
    #ax.clabel(contours, inline=True, fontsize=8, fmt='%.2f')
    
    # Configure polar plot
    ax.set_theta_zero_location('E')
    ax.set_theta_direction(1)
    ax.set_ylim(0, r_centers.max())  # Start from 0 to show hub
    ax.set_yticklabels([])
    ax.grid(False)
    #ax.set_title(label, fontsize=14, pad=20)
    
    # Add colorbar without overlap
    cbar = plt.colorbar(pc, ax=ax, pad=0.15, shrink=0.8)
    cbar.set_label(label, fontsize=10)

plt.tight_layout()
plt.show()

# --- Thrust variation plot ---
plt.close()
plt.figure(figsize=(8, 5))
design = set_bw_design()
plt.plot(angles_deg, performance_values, 's-', label="Instantaneous Propeller Thrust")
plt.plot(angles_deg, np.ones(len(angles_deg))*np.average(performance_values), 
         '--', label=f'Average Thrust: {np.average(performance_values):.2f} N')
plt.xlabel(r"Azimuth angle $\phi$ [deg]")
plt.ylabel(r"$T_p$ [N]")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()