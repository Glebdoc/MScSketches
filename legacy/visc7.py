import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('./task7.txt', delimiter=' ')

y = data[:,1]/1000
uue = data[:,2]
print(data)
U_0 = 12.054
nu = 15e-6

Re_y = U_0*y/nu

Cf = np.linspace(2.1e-3, 2.3e-3, 10)




plt.semilogx(Re_y, uue,  label='u/Ue')

# for cf in Cf:
#     myUe = np.sqrt(cf/2)*((1/0.41)*np.log(Re_y*np.sqrt(cf/2)) + 5.0)
#     plt.plot(Re_y, myUe,  '--',  label=f'Cf = {cf:.5f}', linewidth=0.5)

cf = 0.00221
myUe = np.sqrt(cf/2)*((1/0.41)*np.log(Re_y*np.sqrt(cf/2)) + 5.0)

diff = np.abs(myUe - uue)
x_max = np.argmax(diff)

plt.plot([Re_y[x_max], Re_y[x_max]], [myUe[x_max], uue[x_max]], '--', label=f'Wake component strength = {np.max(diff)*U_0:.3f}[m/s]', color='red')

plt.plot(Re_y, myUe,  '--',  label=f'Law of the wall, Cf = {cf:.5f}', linewidth=0.5)
plt.ylim(0.1, 1.1)
plt.xlabel(r'$Re_y$', fontsize=12)
plt.ylabel(r'$u/U_e$', fontsize=12)
plt.grid(axis='x', which='major')
plt.legend()
plt.show()