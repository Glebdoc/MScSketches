import numpy as np
import matplotlib.pyplot as plt

# Initialize x-coordinates
x = np.linspace(0, 1, 10)

# Initialize y-coordinates with variation
y = np.zeros(10)  
random_variation = 0.1 * np.random.rand(10)  
y += random_variation  
y[0] = 0



R = 1  # Target radius

plt.plot(x, y, label="Original Line")  # Plot original line

for i in range(1, 10):  
    r_to_point = np.sqrt(x[i]**2 + y[i]**2)  # Compute original radius
    theta = np.arctan(x[i]/1)
    r_to_axis = np.sqrt(R**2 + x[i]**2)  # Compute radius to axis
    deltaR = r_to_point - r_to_axis
    r_new = R + deltaR  # Compute new radius

    # Convert back to Cartesian coordinates
    x[i] = r_new * np.sin(theta)
    y[i] = r_new * np.cos(theta)

plt.plot(x, y, label="Transformed Curve")  # Plot transformed line
plt.legend()
plt.axis("equal")  # Keep aspect ratio correct
plt.show()
