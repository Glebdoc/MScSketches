import os
import json
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURE THIS SECTION ---
base_dir = "./drone/"             # directory containing DP0, DP1, ..., DP120
underscore_key = "blade1_angle"   # key to extract from _.json -> settings[underscore_key] (degrees)
performance_key = "thrust_small"  # key to extract from performance.json (optional, collected but unused below)
# ------------------------------

dp_values = []
angles_deg = []         # angles per DP (degrees)
performance_values = [] # optional metric per DP
profiles = []           # list of thrust radial profiles (one per DP)
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
        npz = npz['data']
        # Assuming arrays are stored as plain arrays in the npz:
        f_tan = npz["f_tan"]
        r     = npz["r"]

        # Extract sizes and slice this case's "small propellers" segment
        N_main = underscore_data["main_propeller"]["n"]
        n_small = underscore_data["small_propellers"]["n"]
        start = 3 * (N_main - 1)
        stop  = start + n_small-1          # <-- stop is EXCLUSIVE; gives exactly n_small points
        dt_case = f_tan[start:stop]
        r_case  = r[start:stop+1]

        # First successful case sets the reference r-grid
        if r_ref is None:
            r_ref = r_case.copy()
        else:
            # If lengths or values differ, interpolate to the reference grid
            if (len(r_case) != len(r_ref)) or np.max(np.abs(np.diff(r_case))) == 0 or np.any(r_case != r_ref):
                dt_case = np.interp(r_ref, r_case, dt_case)
            # From now on we use r_ref everywhere
            
        angles_deg.append(underscore_data["settings"][underscore_key])
        performance_values.append(performance_data.get(performance_key, np.nan))
        profiles.append(dt_case)
        dp_values.append(i)

    except Exception as e:
        print(f"⚠️ Error reading from {folder_name}: {e}")

if not profiles:
    raise RuntimeError("No valid data collected. Check file paths and keys.")

# Convert lists to arrays and sort by angle
angles_deg = np.asarray(angles_deg)
angles_rad = np.deg2rad(angles_deg)
profiles   = np.asarray(profiles)          # shape: (n_cases, n_r)
r_centers  = r_ref                          # 1D, length n_r

# Sort by angle so the plot is ordered
order = np.argsort(angles_rad)
angles_rad = angles_rad[order]
profiles   = profiles[order, :]            # still (n_cases, n_r)

# We want C with shape (M, N) where M = n_r and N = n_angles
C = profiles.T                              # shape -> (n_r, n_angles)

angles_rad = angles_rad[1:]     # remove the first angle
C = C[:, 1:]                    # remove corresponding column in C
r_centers = r_centers[:-1]      # optional: match radial dimension too
# ---------------------------------------

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(7, 6))
pc = ax.pcolormesh(angles_rad, r_centers, C, shading='auto', cmap='turbo')
cbar = plt.colorbar(pc, ax=ax, pad=0.1)
cbar.set_label('dT')

ax.set_theta_zero_location('E')
ax.set_theta_direction(1)
plt.tight_layout()
plt.show()
# --- Plot ---
plt.figure(figsize=(8, 5))
plt.plot(angles_deg, performance_values, 's-', label="performance.json values")
plt.xlabel(f"$\phi$")
plt.ylabel("Value")
plt.title("Values from DP folders")
plt.legend()
plt.grid(True)
plt.show()