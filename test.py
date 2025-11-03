import numpy as np 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D

# --- Replace with your solver instance ---
solver = ...  # e.g., solver = DroneSolver(drone, output_dir=path)
total_colloc_points = solver.total_colloc_points
n_origin = solver.n_origin
n_azimuth = solver.n_azimuth

# --- Compute local vectors (rotation plane basis) ---
eps = 1e-12
r_vec = total_colloc_points - n_origin
r_perp = r_vec - np.sum(r_vec * n_azimuth, axis=1, keepdims=True) * n_azimuth
r_perp /= np.linalg.norm(r_perp, axis=1, keepdims=True) + eps

tan_direction = np.cross(n_azimuth, r_perp)
tan_direction /= np.linalg.norm(tan_direction, axis=1, keepdims=True) + eps

# --- Pick which collocation point(s) to visualize ---
indices = [0, 10, 20, 50]   # change or add more indices here

fig = plt.figure(figsize=(7,7))
ax = fig.add_subplot(111, projection="3d")

for i in indices:
    o = n_origin[i]
    # draw axis (red)
    ax.quiver(o[0], o[1], o[2],
              n_azimuth[i,0], n_azimuth[i,1], n_azimuth[i,2],
              color='r', length=0.15, normalize=True)
    # draw radial (green)
    ax.quiver(o[0], o[1], o[2],
              r_perp[i,0], r_perp[i,1], r_perp[i,2],
              color='g', length=0.15, normalize=True)
    # draw tangential (blue)
    ax.quiver(o[0], o[1], o[2],
              tan_direction[i,0], tan_direction[i,1], tan_direction[i,2],
              color='b', length=0.15, normalize=True)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Rotor axis (red), radial (green), tangential (blue)")
ax.set_box_aspect([1,1,1])
plt.show()
