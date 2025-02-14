import numpy as np 
g = 9.81 # [m/s2]
cd0 = 0.01
FM = 0.5

payload = 100 # [kg]
GrossWeight = 1.3*payload*g # [N] 
DL  = 150 # [N/m2] values within range from 95 - 380, more likely something between 100 and 200
solidity = 0.06 # [-] solidity 0.05 - 0.15
NB = 3 # [-] number of blades 2 - can't fly as a copter, 4 - looks like nazi symbol, 3 - good

R = np.sqrt(GrossWeight/(np.pi*DL)) # [m]
chord = np.pi*R*solidity/NB # [m]
AR = R/chord # [-] should be between 14 and 20 
P_ideal = GrossWeight*np.sqrt(GrossWeight/(2*np.pi*R*R)) # [W]
P_hovering = P_ideal/FM # [W]


print(f"R: {R:.2f} m")
print(f"chord: {chord:.2f} m")
print(f"AR: {AR:.2f} -")
print(f"P_ideal: {P_ideal:.2f} W")
print(f"P_hovering: {P_hovering:.2f} W")

