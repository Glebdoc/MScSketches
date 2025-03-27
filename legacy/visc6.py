import numpy as np 
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

myLambda = lambda xl : 0.075*(1-(1-xl)**(-6))

myTheta = lambda xl: np.sqrt(0.075*   ((1-xl)**(-6)   -1)    )

xl = np.linspace(0,0.12,100)

z = lambda: 1/4 - myLambda(xl)

H = lambda z: 2.0 + 4.14*z - 83.5*z**2 + 854*z**3 - 3337*z**4 + 4576*z**5

# plt.plot(xl, myTheta(xl), label=r'$\theta$')
# plt.xlabel(r'$x/L$', fontsize=12)
# plt.ylabel(r'$\frac{\theta}{L} \sqrt{\frac{U_0 L}{\nu}}$', fontsize=12)
# plt.grid()
# plt.show()
# plt.close()

# plt.plot(xl, H(z()), label='H') 
# plt.xlabel(r'$x/L$', fontsize=12)
# plt.ylabel(r'$H$', fontsize=12)
# plt.grid()
# plt.show()
U_0 = 20
L = 20 
nu = 15e-6

x = np.linspace(0, 3, 100)

ue = lambda x: U_0*(1 - x/L)
theta = lambda x: np.sqrt(0.075*nu*L/U_0*((1- x/L)**(-6) - 1))
my_lambda = lambda x: 0.075*(1- (1 - x/L)**(-6))
z = lambda x :1/4 - my_lambda(x)
re_theta = lambda x: ue(x)*theta(x)/nu
re_xtrans = lambda x: ue(x)*x/nu

re_crit = lambda x: np.exp(26.3-8*H(z(x)))
re_trans = lambda x: 2.9*re_xtrans(x)**0.4


trans = lambda x: re_theta(x) - re_trans(x)
crit = lambda x: re_theta(x) - re_crit(x)

guess= [1.4]

x_intersections = fsolve(trans, guess)
x_crit = fsolve(crit, 0.1)
y_crit = re_theta(x_crit)
y_trans = re_theta(x_intersections)


plt.plot(x , re_theta(x), label=r'$Re_{\theta}$')
plt.ylim(0,1000)

plt.plot(x , re_trans(x), label=r'$Re_{\theta, trans}$')
plt.plot(x ,re_crit(x), label=r'$Re_{\theta, crit}$')
plt.plot([x_intersections, x_intersections], [0, 1000], '--', color='grey')
plt.plot([x_crit, x_crit], [0, 1000], '--', color='grey')

plt.plot(x_intersections, y_trans, marker='o', markerfacecolor='red', label=f'x_trans = {float(x_intersections):.3f}', color='grey')
plt.plot(x_crit, y_crit, marker='o', markerfacecolor='green', label=f'x_crit = {float(x_crit):.3f}', color='grey')
plt.xlabel('x')
plt.ylabel('Re')
plt.grid()
plt.legend()
plt.show()



