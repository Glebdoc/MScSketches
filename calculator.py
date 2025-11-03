import numpy as np

RE = 100_000 
NB = 2
mu = 1.8e-5
RPM = 7_000 
R = 0.05
rho =1.225
taper = 0.5
hub = R*0.15

def solidity_from_re(span_position = 1.0):
    return (NB*RE*mu)/(np.pi*rho*RPM*0.1047*R*R*span_position)

def chord_from_solidity(solidity):
    return solidity*np.pi*R/NB

def compute_MAC(c_r, c_t):
    k = (c_t - c_r)/R
    y = lambda x: x*k + c_r
    A = R * 0.5*(c_r + c_t) 
    x = np.linspace(hub, R, 20)
    dx = x[1] - x[0]
    MAC = 1/A * np.sum(y(x)*y(x) *dx)
    span_pos = (MAC-c_r)/k
    return MAC, span_pos/R

def chords_from_average(chord):
    c_r = 2*chord/(1+ taper)
    c_t = taper*c_r
    return c_r, c_t

def R_from_DL(DL, W=60):
    return np.sqrt(W/(np.pi*DL))

err = 1
tol = 1.e-3
span_pos_old = 1.0
while err > tol:
    min_solidity = solidity_from_re(span_pos_old)
    print(min_solidity)
    chord = chord_from_solidity(min_solidity)
    c_r, c_t = chords_from_average(chord)
    MAC, span_pos = compute_MAC(c_r, c_t)
    AR = R/chord
    err = abs(span_pos_old - span_pos)
    span_pos_old = span_pos

print('Min solidity:', min_solidity)
print('Average chord:', chord)
print(f'Root chord:{c_r} Tip chord:{c_t}')
print('MAC:', MAC, 'at R=', span_pos)
print('Resultant AR:', AR)