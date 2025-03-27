import numpy as np
import matplotlib.pyplot as plt

uue = lambda eta: 2*eta - eta**2
eta = np.linspace(0, 1, 100)

# case 1: M = 0, adiabatic wall 
ydelta = lambda eta: 1*eta
temp1 = np.ones(len(eta))


plt.plot(uue(eta), ydelta(eta), label='M=0, adiabatic wall')

# case 2: M = 4, adiabatic wall
ydelta = lambda eta: eta + 8*(1.4-1)*(eta - 4/3*eta**3 + eta**4 -0.2*eta**5)
uue2 = uue(eta)
temp2 = 1 + 8*(1.4-1)*(1-4*eta**2 + 4*eta**3-eta**4)

plt.plot(uue(eta), ydelta(eta), label='M=4, adiabatic wall')

# case 3: M = 4, cooled wall with T_w = T_e 
ydelta = lambda eta: eta + 8*(1.4-1)*(eta**2 -5/3*eta**3 +eta**4 -0.2*eta**5)
temp3 = 1 + 8*(1.4-1)*(2*eta -5*eta**2 +4*eta**3 -eta**4)

plt.plot(uue(eta), ydelta(eta), label='M=4, cooled wall')


plt.legend()
plt.xlabel(r'$u/U_e$')
plt.ylabel(r'$y/\delta_{inc}$')
plt.grid()
plt.show()

plt.close()
ydelta = lambda eta: 1*eta
plt.plot(temp1, ydelta(eta), label='M=0, adiabatic wall')
ydelta = lambda eta: eta + 8*(1.4-1)*(eta - 4/3*eta**3 + eta**4 -0.2*eta**5)
plt.plot(temp2, ydelta(eta), label='M=4, adiabatic wall')
ydelta = lambda eta: eta + 8*(1.4-1)*(eta**2 -5/3*eta**3 +eta**4 -0.2*eta**5)
plt.plot(temp3, ydelta(eta), label='M=4, cooled wall')
plt.xlabel(r'$T/T_e$')
plt.ylabel(r'$y/\delta_{inc}$')
plt.grid()
plt.legend()
plt.show()