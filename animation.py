import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Initial state
x0 = np.array([0.0, 1.09, 0.0])
n_azimuth = np.array([-1.0, 0.0, 0.0])
main_azimuth = np.array([0.0, 0.0, 1.0])
origin = np.array([0.0, 1.0, 0.0])

omega_s = 1000 * 2 * np.pi / 60
omega_m = 1000 * 2 * np.pi / 60
dt = 0.0001

positions = [x0]
origins = [origin]
theta = 0.0
n_azimuths = [n_azimuth]

# One full revolution
T =3* 2 * np.pi / omega_m
number_of_steps = int(T / dt)

# Velocity function
def compute_velocity(x, origin, n_azimuth, omega_s, main_azimuth, omega_m):
    return np.cross(x - origin, n_azimuth * omega_s) + np.cross(x, main_azimuth * omega_m)

# Runge-Kutta integration
for _ in range(number_of_steps):
    k1 = -compute_velocity(x0, origin, n_azimuth, omega_s, main_azimuth, omega_m)
    k2 = -compute_velocity(x0 + 0.5 * dt * k1, origin, n_azimuth, omega_s, main_azimuth, omega_m)
    k3 = -compute_velocity(x0 + 0.5 * dt * k2, origin, n_azimuth, omega_s, main_azimuth, omega_m)
    k4 = -compute_velocity(x0 + dt * k3, origin, n_azimuth, omega_s, main_azimuth, omega_m)
    
    x0 = x0 + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    positions.append(x0)

    theta += dt * omega_m
    n_azimuth = np.array([-np.cos(theta), -np.sin(theta), 0.0])
    origin = np.array([-np.sin(theta), np.cos(theta), 0.0])

    n_azimuths.append(n_azimuth)
    origins.append(origin)

positions = np.array(positions)
n_azimuths = np.array(n_azimuths)
origins = np.array(origins)

# Animation
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-1.2, 1.2)
ax.set_zlim(-.5, .5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Particle Animation')

particle, = ax.plot([], [], [], 'ro', label='Particle')
trail, = ax.plot([], [], [], 'b-', linewidth=1, alpha=0.6, label='Trajectory')
azimuth_arrow = ax.quiver(0, 0, 0, 0, 0, 0, color='purple', length=0.2)

def init():
    particle.set_data([], [])
    particle.set_3d_properties([])
    trail.set_data([], [])
    trail.set_3d_properties([])
    return particle, trail, azimuth_arrow

def update(frame):
    global azimuth_arrow
    pos = positions[frame]
    particle.set_data([pos[0]], [pos[1]])
    particle.set_3d_properties([pos[2]])

    trail.set_data(positions[:frame+1, 0], positions[:frame+1, 1])
    trail.set_3d_properties(positions[:frame+1, 2])

    # Remove the previous quiver arrow
    azimuth_arrow.remove()

    # Add new azimuth arrow
    az = n_azimuths[frame]
    org = origins[frame]
    azimuth_arrow = ax.quiver(
        org[0], org[1], org[2],
        az[0], az[1], az[2],
        color='purple', length=0.2, normalize=True
    )

    return particle, trail, azimuth_arrow

frames = range(0, len(positions), 5)  # Show every 5th frame
ani = FuncAnimation(fig, update, frames=frames, interval=30)
# ani = FuncAnimation(
#     fig, update, frames=len(positions),
#     init_func=init, interval=2, blit=False
# )
# ani.save('trajectory.gif', writer='pillow', fps=25)

plt.legend()
plt.show()
