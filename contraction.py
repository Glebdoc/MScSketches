import numpy as np 
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
