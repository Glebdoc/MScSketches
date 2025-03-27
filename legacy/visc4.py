import numpy as np
import matplotlib.pyplot as plt

xl = np.linspace(1,4, 100)
xl1 = np.linspace(0,1, 10)
a = 0.45
momThickness = lambda xl: a*((2*xl -1)**1.1)
straightLine = lambda xl1: 0.45*xl1

#statford = lambda xl: xl**2 * (1-(2*xl-1)**(-0.2))*(4/(25*(2*xl-1)**2.4))
statford = lambda xl: 0.16*xl*xl*(2*xl -1)**(-2.4)*(1-(2*xl-1)**(-0.2))

plt.plot(xl, momThickness(xl), color='red')
plt.plot(xl1, straightLine(xl1), color='blue')
plt.xlabel(r'x/L')
plt.ylabel(r'$\frac{\theta^2 U_\infty}{\nu L}$')
plt.grid()

plt.show()
plt.close()

plt.plot(xl, statford(xl),  label='Stratford\'s separation parameter')
plt.xlabel(r'x/L')
plt.ylabel('Stratford separation parameter')
plt.plot([1,4], [0.0104,0.0104], 'r--', label='Stratford\'s separation value',)
plt.grid()
plt.legend()
plt.show()
