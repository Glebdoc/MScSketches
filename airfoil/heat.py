import numpy as np 
import matplotlib.pyplot as plt

N = 10
T_final = 30
delta_t = 1e-5
k = 0.1

x = np.linspace(0,np.pi, N)
h = x[1]

u_0 = np.sin(x)
u = np.zeros_like(u_0)

t =0 
while t<= T_final:
    u[1:-1] = delta_t*k*((u_0[1:-1] - u_0[:-2]- u_0[2:])/h**2) + u_0[1:-1]
    u_0 = u 
    t += delta_t
    #plt.plot(x, u,  color='k', marker='o')
plt.plot(x, u,  color='k', marker='o')
plt.show()