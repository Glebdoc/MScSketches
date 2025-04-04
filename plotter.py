import os, json
import numpy as np
import matplotlib.pyplot as plt

def plot(files, show = True, title=False):

    # define a figure with 6 subplots
    fig, axs = plt.subplots(2, 3, figsize=(15, 10)) 

    # read CFD data 
    # cfd = np.genfromtxt('./results/cumulativeLift.txt', delimiter=None, skip_header=5)
    # r = cfd[:, 0]
    # lift_cumulative = cfd[:, 1]
    # # now we need to take a derivative of the cumulative lift to get the lift
    # step = r[1] - r[0]
    # lift = (lift_cumulative[1:] - lift_cumulative[:-1])

    # axs[1, 1].plot(r[:-1], lift, label='CFD', marker='o')

    for file in files:
        data = np.genfromtxt(f'./results/{file}_res.csv', delimiter=',', skip_header=1)
        file = file.replace("_res.csv", "")

        # plot the data
        # r, v_axial, v_tangential, inflowangle, alpha, Faxial, Ftan
        axs[0, 0].plot(data[:, 0], -data[:, 1], label=f'{file}', marker='o')
        axs[0, 0].set_title('r vs v_axial')
        axs[0, 0].legend()

        axs[0, 1].plot(data[:, 0], -data[:, 2], label=f'{file}', marker='o')
        axs[0, 1].set_title('r vs v_tangential')
        axs[0, 1].legend()

        axs[0, 2].plot(data[:, 0], np.rad2deg(data[:, 3]), label=f'{file}', marker='o')
        axs[0, 2].set_title('r vs inflowangle')
        axs[0, 2].legend()

        axs[1, 0].plot(data[:, 0], data[:, 4], label=f'{file}', marker='o')
        axs[1, 0].set_title('r vs alpha')
        axs[1, 0].legend()

        axs[1, 1].plot(data[:, 0], data[:, 5], label=f'{file}', marker='o')
        axs[1, 1].set_title('r vs Faxial')
        axs[1, 1].legend()

        axs[1, 2].plot(data[:, 0], data[:, 6], label=f'{file}', marker='o')
        axs[1, 2].set_title('r vs Ftan')
        axs[1, 2].legend()
    if title:
        plt.tight_layout()
        plt.savefig(f'./plots/fig_{title}.png', dpi = 500)

    
    if show:
        plt.show()