import numpy as np 
import matplotlib.pyplot as plt
""""
From: A Prescribed Wake Lifting Surface Hover Performance Analysis
      by J. David Kocurek James L. Tangler

C_T = T/rho pi R^2 (Omega R)^2

theta_1  - Blade linear twist, washout negative   (pitch_tip - pitch_root)

"""

def get_coefs(theta_1):
    B = - 0.000729*theta_1 
    C = -2.3 + 0.206*theta_1
    m = 1.0 - 0.25 * np.exp(0.040*theta_1)
    n = 0.5 - 0.0172 * theta_1
    return B, C, m, n

def get_k1(B, C, C_T, NB, n ,m):
    k1 = B + C*(C_T/(NB**n))**m
    return k1

def get_k2(B, C, C_T, NB, n ,m):
    C_T0 = (NB**n) * ((- B/C)**(1/m))
    k2 = - (C_T - C_T0)**0.5
    return k2


"""

r = A + (1-A)*np.exp(-lambda*theta) ; where A = 0.78

assuming T = 70N 

lambda = 4.0 * (C_T**0.5)

"""

def computeCt(T, Omega, R):
    return T/(1.225*np.pi*R*R*(Omega*R)**2)

def computeR(theta):
    A = 0.78
    Ct = computeCt(70, 100, 0.5)  # Example values for T, Omega, R
    lmbda = 4.0 * (Ct**0.5)
    return A + (1 - A) * np.exp(-lmbda * theta)

theta = np.linspace(0, 4*360*np.pi/180, 100)  # Example theta values
print("Theta values:", theta*180/np.pi)
R_values = computeR(theta)
plt.plot(theta, R_values)
plt.grid()
plt.xlabel('Theta')
plt.ylabel('R values')
plt.title('R values vs Theta')
plt.show()