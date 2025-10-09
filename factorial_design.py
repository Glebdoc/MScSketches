import itertools
import pandas as pd
import numpy as np

# Factor ranges (low, high)
R_heli = np.array([0.9, 1.1])  # Rotor radius in meters
R_quadcopter = R_heli / 4
diagonal2R = np.array([1, 1.5])  # Diagonal to radius ratio
helicopter_factors = {
    "R": R_heli,
    "NB": [2, 3],
    "sigma": [0.08, 0.10],
    "theta": [15, 20],
}
quadcopter_factors = {
    "R": R_quadcopter,
    "diagonal": diagonal2R,
    "NB": [2, 3],
    "sigma": [0.08, 0.10],
    "theta": [15, 20],
}   
drone_factors = {
    "R": [0.97, 1.03],
    "pitch": [15, 20],
    "R_small": [0.1, 0.12],
    "NB_small": [3],
    "sigma_small": [0.08, 0.10],
}


factors = [drone_factors]
titles = ['drone']
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
