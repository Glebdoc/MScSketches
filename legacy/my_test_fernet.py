import numpy as np 
import matplotlib.pyplot as plt

naccelle_angle = 70  # degrees
downwash = 2 #m/s
a = 1.0 
b = -np.deg2rad(90 - naccelle_angle)  # pitch of the helix (rise per radian) 
phi0 = np.pi/2         # phase shift (radians)
omega = -5.0       # <--- spin: +CCW, -CW (looking from +Z toward origin)

t = np.linspace(0, 2*np.pi, 100)


theta = omega*t + phi0

# arclength (speed * t); speed = ||r'(t)||
#s = np.sqrt((a*omega)**2 + b**2) * t

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# --- Frenet frame (vectorized) for r(t) = (a cos θ, a sin θ, b t) ---
# Unit tangent T = r'(t)/||r'(t)|| with ω
speed_inv = 1.0 / np.sqrt((a*omega)**2 + b**2)
T = np.zeros((len(t), 3))
T[:,0] = -a*omega*np.sin(theta) * speed_inv
T[:,1] =  a*omega*np.cos(theta) * speed_inv
T[:,2] =  b * speed_inv

# Unit normal N (points radially inward)
N = np.vstack((-np.cos(theta), -np.sin(theta), np.zeros_like(t))).T

# Unit binormal B = T × N
B = np.cross(T, N)

# Curve points
rx = a*np.cos(theta)
ry = a*np.sin(theta)
rz = b*t

# Plot one Frenet–Serret frame
i = 20
ax.quiver(rx[i], ry[i], rz[i], T[i,0], T[i,1], T[i,2], color='r', length=0.5, normalize=True)
ax.quiver(rx[i], ry[i], rz[i], N[i,0], N[i,1], N[i,2], color='g', length=0.5, normalize=True)
ax.quiver(rx[i], ry[i], rz[i], B[i,0], B[i,1], B[i,2], color='b', length=0.5, normalize=True)

i = 0
ax.quiver(rx[i], ry[i], rz[i], T[i,0], T[i,1], T[i,2], color='r', length=0.5, normalize=True)
ax.quiver(rx[i], ry[i], rz[i], N[i,0], N[i,1], N[i,2], color='g', length=0.5, normalize=True)
ax.quiver(rx[i], ry[i], rz[i], B[i,0], B[i,1], B[i,2], color='b', length=0.5, normalize=True)

i = 40
ax.quiver(rx[i], ry[i], rz[i], T[i,0], T[i,1], T[i,2], color='r', length=0.5, normalize=True)
ax.quiver(rx[i], ry[i], rz[i], N[i,0], N[i,1], N[i,2], color='g', length=0.5, normalize=True)
ax.quiver(rx[i], ry[i], rz[i], B[i,0], B[i,1], B[i,2], color='b', length=0.5, normalize=True)

i = 60
ax.quiver(rx[i], ry[i], rz[i], T[i,0], T[i,1], T[i,2], color='r', length=0.5, normalize=True)
ax.quiver(rx[i], ry[i], rz[i], N[i,0], N[i,1], N[i,2], color='g', length=0.5, normalize=True)
ax.quiver(rx[i], ry[i], rz[i], B[i,0], B[i,1], B[i,2], color='b', length=0.5, normalize=True)

i=20


# --- Your extra geometry (unchanged) ---
origin= np.array([0,1,0])
direction = np.array([0.5,0,-0.5])
direction = direction / np.linalg.norm(direction)
ax.quiver(origin[0], origin[1], origin[2], direction[0], direction[1], direction[2],
          color='r', length=0.5, normalize=True)

point_orig = np.array([0.2,1.3,0.3])
ax.scatter(point_orig[0], point_orig[1], point_orig[2], color='g', s=50)

point = direction*t[i] + point_orig
ax.scatter(point[0], point[1], point[2], color='b', s=50)

# shortest vector from line to 'point'
print('np.dot(point - origin, direction)', np.dot(point - origin, direction))
print('np.dot(direction, direction)*direction', np.dot(direction, direction)*direction)
print('np.dot(point - origin, direction) / np.dot(direction, direction) * direction', np.dot(point - origin, direction) / np.dot(direction, direction) * direction)
d = (point - origin) - np.dot(point - origin, direction) / np.dot(direction, direction) * direction
print('d', d)

# (optional) express d in the Frenet frame at i:
# d_T = d·T[i], d_N = d·N[i], d_B = d·B[i]
d_T, d_N, d_B = np.dot(d, T[i]), np.dot(d, N[i]), np.dot(d, B[i])
point_in_frame =- d_T*T[i] + (d_N*N[i] )+ (d_B*B[i]) + np.array([rx[i], ry[i], rz[i]])
ax.scatter(point_in_frame[0], point_in_frame[1], point_in_frame[2], color='m', s=50)

# helix
ax.plot(rx, ry, rz, 'k')
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_title('Helix with controllable spin')
ax.set_aspect('equal')

plt.show()
