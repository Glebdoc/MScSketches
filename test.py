import numpy as np 
import matplotlib.pyplot as plt
R = 2
hub = 0.2
n = 40
theta = np.linspace(np.pi/6, np.arccos(hub/R), n)

x1 = np.cos(theta)*R
x2 = np.cos(0.5*theta)
x3 = np.cos(0.25*theta)
x4 = np.cos(0.01*theta)

# normalize x2 
x2 = (x2 - np.min(x2)) / (np.max(x2) - np.min(x2)) 
x2 = x2 *(R - hub) + hub

x3 = (x3 - np.min(x3)) / (np.max(x3) - np.min(x3))
x3 = x3 *(R - hub) + hub

x4 = (x4 - np.min(x4)) / (np.max(x4) - np.min(x4))
x4 = x4 *(R - hub) + hub

y1 = np.zeros(n)
y2 = np.ones(n)
y3 = np.ones(n)*0.5
y4 = np.ones(n)*0.25

plt.plot(x1, y1, 'ro')
plt.plot(x2, y2, 'bo')
plt.plot(x3, y3, 'go')
plt.plot(x4, y4, 'ko')
plt.show()