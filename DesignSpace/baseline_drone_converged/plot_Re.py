import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 

# load npz 
data = np.load('_res.npz')
data = data['data']

r = data['r']
Re = data['Re']

n =50 -1 
npM = 

plt.figure(figsize=(8,6))
plt.plot(r[:n], Re[:n], label='Reynolds Number Distribution', color='blue')
for i in range(3): 
    plt.plot()