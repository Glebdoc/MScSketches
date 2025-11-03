# plot_both.py
import json, os
import numpy as np
import matplotlib.pyplot as plt

# ---- small helper: nice grayscale style ----
# def set_bw():
#     plt.style.use("seaborn-v0_8-whitegrid")
#     plt.rcParams.update({
#         "figure.figsize": (10, 6),
#         "figure.dpi": 160,
#         "axes.edgecolor": "black",
#         "axes.labelweight": "bold",
#         "grid.alpha": 0.3,
#         "grid.color": "0.5",
#         "lines.linewidth": 1.8,
#         "axes.prop_cycle": plt.cycler(color=["0.1","0.5"])  # black & mid-gray
#     })

# def load_case(case_dir):
#     """Load config JSON and structured 'data' array from _res.npz."""
#     with open(os.path.join(case_dir, "_.json"), "r") as f:
#         cfg = json.load(f)
#     npz = np.load(os.path.join(case_dir, "_res.npz"))
#     arr = npz["data"]  # structured array with fields
#     return cfg, arr

# def mask_main_rotor(arr, cfg):
#     """
#     For the spinning-drone case, separate main-rotor elements from tip-prop elements
#     using the chord range of the main rotor from JSON. For helicopter, returns all.
#     """
#     craft = cfg["main_propeller"].get("AIRCRAFT", "").lower()
#     if "drone" in craft:  # spinning drone case
#         c_root = float(cfg["main_propeller"]["chord_root"])
#         c_tip  = float(cfg["main_propeller"]["chord_tip"])
#         c_mid  = 0.5*(c_root + c_tip)
#         # main rotor chords are much larger than tip-prop chords → threshold halfway to smaller chords
#         # Build a conservative threshold slightly below the midpoint
#         thresh = 0.8 * c_mid
#         return arr[arr["chords"] >= thresh]
#     else:
#         # helicopter: only main rotor exists
#         return arr



drone = np.load('./drone/_res.npz')
drone = drone['data']
heli = np.load('./helicopter/_res.npz')
heli = heli['data']
n = 70 -1
plt.plot(drone['r'][:n], drone['alpha'][:n])
plt.plot(heli['r'][:n], heli['alpha'][:n])
plt.show()

plt.plot(drone['r'][:n], drone['gammas'][:n], label='drone')
plt.plot(heli['r'][:n], heli['gammas'][:n], label='helicopter')
plt.legend()
plt.show()

plt.plot(drone['r'][:n], drone['v_axial'][:n], label='drone')
plt.plot(heli['r'][:n], heli['v_axial'][:n], label='helicopter')
plt.legend()
plt.show()

plt.plot(drone['r'][:n], drone['v_tangential'][:n], label='drone')
plt.plot(heli['r'][:n], heli['v_tangential'][:n], label='helicopter')
plt.legend()
plt.show()

plt.plot(drone['r'][:n], drone['f_axial'][:n], label='drone')
plt.plot(heli['r'][:n], heli['f_axial'][:n], label='helicopter')
plt.legend()
plt.show()

plt.plot(drone['r'][:n], drone['f_tan'][:n], label='drone', marker='o')
plt.plot(heli['r'][:n], heli['f_tan'][:n], label='helicopter', marker='o')
plt.legend()
plt.show()

plt.plot(drone['r'][:n], drone['Cl'][:n]/drone['Cd'][:n], label='drone', marker='o')
plt.plot(heli['r'][:n], heli['Cl'][:n]/drone['Cd'][:n], label='helicopter', marker='o')
plt.legend()
plt.show()

# def span_norm(main_arr, cfg):
#     """Return nondimensional span coordinate s = r/R using main-rotor radius from JSON."""
#     R = float(cfg["main_propeller"]["diameter"]) * 0.5
#     s = main_arr["r"] / R
#     return s

# def sort_by_s(s, *cols):
#     idx = np.argsort(s)
#     return (s[idx],) + tuple(c[idx] for c in cols)

# def plot_drone_vs_heli(drone_dir="./drone", heli_dir="./helicopter"):
#     set_bw()

#     # load
#     cfg_d, arr_d = load_case(drone_dir)
#     cfg_h, arr_h = load_case(heli_dir)

#     # select main-rotor elements
#     main_d = mask_main_rotor(arr_d, cfg_d)
#     main_h = mask_main_rotor(arr_h, cfg_h)

#     # build s = r/R
#     s_d = span_norm(main_d, cfg_d)
#     s_h = span_norm(main_h, cfg_h)

#     # pick fields to compare
#     fld = {
#         r"$\Gamma$ (m$^2$/s)": ("gammas",),
#         r"$\alpha$ (deg)": ("alpha",),
#         r"$\phi$ inflow (deg)": ("inflowangle",),
#         r"$C_\ell$": ("Cl",),
#         r"$C_d$": ("Cd",),
#         r"$v_\mathrm{axial}$ (m/s)": ("v_axial",),
#     }

#     fig, axes = plt.subplots(2, 3, figsize=(12, 7), constrained_layout=True)
#     axes = axes.ravel()

#     for ax, (label, names) in zip(axes, fld.items()):
#         # extract & sort
#         sd, = sort_by_s(s_d, )
#         sh, = sort_by_s(s_h, )
#         yd = sort_by_s(s_d, main_d[names[0]])[1]
#         yh = sort_by_s(s_h, main_h[names[0]])[1]

#         ax.plot(sh, yh, label="Helicopter", color="0.1")
#         ax.plot(sd, yd, label="Spinning-drone (main)", color="0.5")
#         ax.set_xlabel(r"$r/R$")
#         ax.set_ylabel(label)
#         ax.set_xlim(0, 1.0)
#         ax.grid(True, alpha=0.3)

#     # one legend for all
#     handles, labels = axes[0].get_legend_handles_labels()
#     fig.legend(handles, labels, loc="upper center", ncol=2, frameon=True)

#     fig.suptitle("Main-rotor distributions: Helicopter vs Spinning-drone", y=1.02)
#     out = "drone_vs_heli_main.png"
#     plt.savefig(out, dpi=200, bbox_inches="tight")
#     print(f"Saved {out}")
#     plt.show()

# if __name__ == "__main__":
#     plot_drone_vs_heli("./drone", "./helicopter")





# # print(f"Loaded file: {path}")
# # print("Available keys:", drone.files)

# # Preview each array
# for key in drone.files:
#     arr = drone[key]
#     print(f"{key}: shape={arr.shape}, dtype={arr.dtype}")
#     if arr.ndim == 1:
#         print("  first 5:", arr[:5])
#     elif arr.ndim == 2:
#         print("  first 2 rows:\n", arr[:2])

# def plot_self(self):
#         alpha =  self.alpha 
#         r = self.r 
#         chords = self.chords
#         Cl = self.Cl
#         Cd = self.Cd
#         Re = self.Re
#         inflowangle = self.inflowangle*180/np.pi
#         twist = self.twist
#         v_axial = self.v_axial
#         v_tangential = self.v_tangential
#         vel_total = self.vel_total
#         gamma = self.gammas
#         alpha_ideal = self.alpha_cl32cd

#         main_n = self.main_n
#         small_n = self.small_n
#         npM = self.npM

#         r_main = r[:main_n-1]
#         r_main = self.normalize(r_main)
#         r_small = r[npM: npM + small_n-1]
#         r_small = self.normalize(r_small)

#         fig, axs = plt.subplots(3, 3, figsize=(15, 12))
#         axs[0, 0].plot(r_main, alpha[:main_n-1])
#         # plot ideal alpha  corresponding to cl^3/2 / cd 
#         axs[0, 0].plot(r_main, alpha_ideal[:main_n-1], color='green', label='Ideal alpha (main)', linestyle=':')
#         for i in range(self.small_NB):
#             axs[0, 0].plot(r_small, alpha[npM + i*(small_n-1): npM + (i+1)*(small_n-1)], linestyle='--', label=f'Small blade {i+1}')
#         axs[0, 0].set_title('Angle of Attack (Main Rotor)')
#         axs[0, 0].set_xlabel('Radius (m)')
#         axs[0, 0].set_ylabel('Alpha (deg)')
#         axs[0, 0].legend()
#         axs[0, 0].grid()

        
#         axs[0, 1].plot(r_main, inflowangle[:main_n -1])
#         axs[0, 1].plot(r_small, inflowangle[npM: npM + small_n-1], color='orange')

#         axs[0, 2].plot(r_main, twist[:main_n -1])
#         axs[0, 2].plot(r_small, twist[npM: npM + small_n-1], color='orange')

#         # plot axial velocity
#         axs[1, 0].plot(r_main, v_axial[:main_n -1])
#         for i in range(self.small_NB):
#             axs[1, 0].plot(r_small, v_axial[npM + i*(small_n-1): npM + (i+1)*(small_n-1)], linestyle='--', label=f'Small blade {i+1}')

#         # plot Re 
#         axs[1,1].plot(r_main, Re[:main_n -1])
#         for i in range(self.small_NB):
#             axs[1, 1].plot(r_small, Re[npM + i*(small_n-1): npM + (i+1)*(small_n-1)], linestyle='--', label=f'Small blade {i+1}')
#         axs[1,1].legend()

#         # plot Mach number 
#         a = 340 # m/s
#         V_infty = np.sqrt(v_axial**2 + v_tangential**2)
#         M = np.array(V_infty)/a
#         axs[1, 2].plot(r_main, M[:main_n -1])
#         for i in range(self.small_NB):
#             axs[1, 2].plot(r_small, M[npM + i*(small_n-1): npM + (i+1)*(small_n-1)], linestyle='--', label=f'Small blade {i+1}')
#         axs[1,2].legend()
