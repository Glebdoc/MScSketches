import itertools
import pandas as pd
import numpy as np

MTOW = 60

def R_from_DL(DL, W=60):
    return np.sqrt(W/(np.pi*DL))


# Factor ranges (low, high)
R_heli = np.array([0.9, 1.1])  # Rotor radius in meters
R_quadcopter = R_heli / 4
diagonal2R = np.array([1, 1.5])  # Diagonal to radius ratio

dji_disk_loading = 102.3
diagonal = 0.643
taper = 0.7
NB = 2
quadcopter_factors = {
    "disk_loading": [0.8*dji_disk_loading, dji_disk_loading, 1.2*dji_disk_loading],
    "diagonal": [0.8*diagonal, diagonal, 1.2*diagonal],
    "taper": [0.8*taper, 1.2*taper],
    "sigma": [0.06 , 0.08,  0.01]
}   
# drone_factors = {
#     "R": [0.97, 1.03],
#     "pitch": [15, 20],
#     "R_small": [0.1, 0.12],
#     "NB_small": [3],
#     "sigma_small": [0.08, 0.10],
# }
R_base=1.0
DL_base = MTOW/(np.pi*R_base*R_base)
base_sigma = 0.08
base_lambda = 0.075
pitch_small_base =  67
taper_base = 0.777


drone_factors = {
    "disk loading": [0.9*DL_base, DL_base, 1.1*DL_base],
    "sigma_main": [0.07, 0.09],
    "Re_min":[90_000, 110_000],
    "lambda_p": [0.0675, 0.0825],
    "pitch_small":[65, 69],
    "taper": [0.9*taper_base, 1.1*taper_base]
}

helicopter_factors = {
    "disk loading": [0.9*DL_base, DL_base, 1.1*DL_base],
    "sigma_main": [0.07, 0.08, 0.09],
    "taper": [0.9*taper_base, 1.1*taper_base]
}
n = 120
azimuth_factors = {
    "azimuth_angle":np.linspace(0,2*np.pi- np.pi/n, n),
}


factors = [azimuth_factors]
titles = ['azimuth']
# Build full factorial design
count = 0
for f in factors:
    title = titles[count]
    levels = list(f.values())
    names = list(f.keys())
    design = list(itertools.product(*levels))

    df = pd.DataFrame(design, columns=names)
    # save to csv
    df.to_csv(f'./Factorial_trial/factorial_data/{title}_factorial_design.csv', index=False)
    count += 1
print(df)
