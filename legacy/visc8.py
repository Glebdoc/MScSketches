import numpy as np
import matplotlib.pyplot as plt

PI = [0.5, 1.5, 4]


def defect_profiles(PI):
    eta = np.linspace(0, 1, 200)
    u = lambda eta: - (1/0.41)*np.log(eta) + (2*PI/0.41)*(1-3*eta**2+2*eta**3)
    plt.plot(eta, u(eta), label=f'PI = {PI}')

for pi in PI:
    defect_profiles(pi)

plt.xlabel(r'$\eta$', fontsize=12)
plt.ylabel(r'$(U_e-\overline{u})/v^*$', fontsize=12)
plt.grid()
plt.legend()
plt.show()
