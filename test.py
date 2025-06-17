import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

data_main = np.loadtxt('./A18_(smoothed)_Re0.191_M0.00_N9.0.txt', skiprows=12)
data_small = np.loadtxt('./A18 (smoothed)_A18_(smoothed)_Re0.050_M0.00_N9.0.txt',skiprows=12)

spline = make_interp_spline(data_main[:, 0], data_main[:, 1], k=3)
alpha_smooth = np.linspace(data_main[:, 0].min(), data_main[:, 0].max(), 300)
cl_smooth = spline(alpha_smooth)

spline2 = make_interp_spline(data_small[::3, 0], data_small[::3, 1], k=2)


alpha_smooth_small = np.linspace(data_small[::3, 0].min(), data_small[::3, 0].max(), 300)
cl_smooth_small = spline2(alpha_smooth_small)

plt.plot(data_main[:, 0], data_main[:, 1], label='A18 (smoothed) Re0.191', marker='o')
plt.plot(alpha_smooth, cl_smooth, label='A18 (smoothed) Re0.191 (spline)', linestyle='--')
plt.plot(data_small[:, 0], data_small[:, 1], label='A18 (smoothed) Re0.050', marker='x')
plt.plot(alpha_smooth_small, cl_smooth_small, label='A18 (smoothed) Re0.050 (spline)', linestyle='--')
plt.xlabel('Alpha [deg]')
plt.ylabel('Cl')
plt.title('A18 Airfoil Cl Comparison')
plt.grid()
plt.legend()
plt.show()

# def median_filter(data, kernel_size=5, startFromTheEnd=True):
#     if startFromTheEnd:
#         data = data[::-1]
#     for i in range(len(data)):
#         if i < kernel_size // 2 or i >= len(data) - kernel_size // 2:
#             continue
#         data[i] = np.median(data[i - kernel_size // 2:i + kernel_size // 2 + 1])
#     if startFromTheEnd:
#         data = data[::-1]
#     return data

# x = np.linspace(0, 1, 100)
# data = np.sin(2 * np.pi * x) + 0.1 * np.random.normal(size=x.shape)
# filtered_data = median_filter(data.copy(), kernel_size=10)
# plt.plot(x, data, label='Original Data', alpha=0.5)
# plt.plot(x, filtered_data, label='Filtered Data', color='red')
# plt.xlabel('X-axis')
# plt.ylabel('Y-axis')
# plt.title('Median Filter Example')
# plt.legend()
# plt.grid()
# plt.show()

# plot QBlade corrections 
# data = np.loadtxt('QBlade_3D_correction_0.txt', skiprows=3)
# vl_data =  np.genfromtxt(f'./results/experimental_pitch_root30_res.csv', delimiter=',', skip_header=1)

                     

# # plot 3D correction
# plt.plot(data[:, 0], data[:, 1], label='3D Correction', marker='s')

# # no correction
# plt.plot(data[:, 0], data[:, 3], label='No Correction', marker='o')
# # plot TipLoss
# plt.plot(data[:, 0], data[:, 7], label='Tip Loss', marker='x')
# plt.plot(vl_data[:49, 0], vl_data[:49, 4], label='Vortex Lattice', marker='^')
# plt.legend()
# plt.xlabel('r')
# plt.ylabel(r'$\alpha$')
# plt.grid()
# plt.show()
#plt.savefig('2007_improved_wake.png', dpi=300)
#plt.savefig('3007_aoa_added0.133.png', dpi=300)

# plt.savefig('2007_aoa.png', dpi=300)
# xfoil_data = np.loadtxt('A18.txt', skiprows=12)
# alpha = xfoil_data[:, 0]
# cl = xfoil_data[:, 1]
# cd = xfoil_data[:, 2]

# qblade_data = np.loadtxt('A18_Re0.148_QBlade.txt', skiprows=12)
# alpha_qb = qblade_data[:, 0]
# cl_qb = qblade_data[:, 1]
# cd_qb = qblade_data[:, 2]

# cfd_data = np.loadtxt('A18_Re0.148_CFD.txt', skiprows=1)
# alpha_cfd = cfd_data[:, 0]
# cl_cfd = cfd_data[:, 1]
# #cl_cfd_computed = cfd_data[:, 3]/(1.225*0.5*(2.15826**2))
# cd_cfd = -cfd_data[:, 2]
    
# fig, ax = plt.subplots(1, 2, figsize=(10, 4))
# ax[0].plot(alpha, cl, label='A18 XFOIL', marker='o')
# ax[0].plot(alpha_qb, cl_qb, label='A18 QBlade', marker='x')
# ax[0].plot(alpha_cfd, cl_cfd, label='A18 CFD', marker='s')
# #ax[0].plot(alpha_cfd, cl_cfd_computed, label='A18 CFD Computed', marker='^')
# ax[0].set_xlabel('Alpha [deg]')
# ax[0].set_ylabel('Cl')
# ax[0].grid()
# ax[0].legend()

# ax[1].plot(alpha, cd, label='A18 XFOIL', marker='o')
# ax[1].plot(alpha_qb, cd_qb, label='A18 QBlade', marker='x')
# ax[1].plot(alpha_cfd, cd_cfd, label='A18 CFD', marker='s')
# ax[1].set_xlabel('Alpha [deg]')
# ax[1].set_ylabel('Cd')
# ax[1].grid()
# ax[1].legend()
# plt.tight_layout()
# plt.show()



# def rotation_matrix(angle_x=0, angle_y=0, angle_z=0):
#         """ Compute the full 3D rotation matrix (XYZ order). """
#         Rx = np.array([[1, 0, 0],
#                        [0, np.cos(angle_x), -np.sin(angle_x)],
#                        [0, np.sin(angle_x), np.cos(angle_x)]])
        
#         Ry = np.array([[np.cos(angle_y), 0, np.sin(angle_y)],
#                        [0, 1, 0],
#                        [-np.sin(angle_y), 0, np.cos(angle_y)]])
        
#         Rz = np.array([[np.cos(angle_z), -np.sin(angle_z), 0],
#                        [np.sin(angle_z), np.cos(angle_z), 0],
#                        [0, 0, 1]])

#         return Rz @ Ry @ Rx 


# vector = np.array([0, 0, 2])

# # rotate around y axis - 90 

# angle_y = -np.pi/2
# angle_z = 2*np.pi/3

# R = rotation_matrix(angle_y=angle_y)
# rotated_vector_y = R @ vector

# R = rotation_matrix(angle_y = angle_y, angle_z=angle_z)
# rotated_vector_yz = R @ vector

# # Define origin
# origin = np.array([0, 0, 0])

# import pyvista as pv
# import numpy as np




# Define vectors and origin
# vectors = np.array([
#     vector,
#     rotated_vector_y,
#     rotated_vector_yz
# ])
# origins = np.array([
#     [0, 0, 0],
#     [0, 0, 0],
#     [0, 0, 0]
# ])

# # Create PyVista point cloud
# point_cloud = pv.PolyData(origins)
# point_cloud["vectors"] = vectors  # attach vectors to each point

# # Use glyphs to draw arrows with actual magnitude
# arrows = point_cloud.glyph(orient="vectors", scale="vectors", factor=1.0)

# # Plot
# plotter = pv.Plotter()
# plotter.add_mesh(arrows, color="orange")
# plotter.add_point_labels(origins + vectors, ["v1", "v2", "v3"], font_size=36, text_color="black")
# plotter.add_axes_at_origin()
# plotter.show_bounds(xlabel='X', ylabel='Y', zlabel='Z')
# plotter.show()
