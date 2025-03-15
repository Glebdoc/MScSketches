import os, json
import numpy as np
import matplotlib.pyplot as plt

def plot():

    # read files from results folder
    result_files = [f for f in os.listdir('./results') if f.endswith('.csv')]

    # define a figure with 6 subplots
    fig, axs = plt.subplots(2, 3)

    for file in result_files:
        data = np.genfromtxt(f'./results/{file}', delimiter=',', skip_header=1)
        file = file.replace("results_def_", "")
        file = file.replace(".csv", "")

        #within file there is NB followed with an int and inc with an int. I need to extract these values
        NB = int(file.split("_")[0].replace("NB", ""))
        inc = int(file.split("_")[1].replace("inc", ""))

        if NB == 3:
            color = 'b'
        elif NB == 4:
            color = 'r'
        elif NB == 5:
            color = 'g'
        else:
            color = 'black'

        if inc == 60:
            linestyle = '-'
        elif inc == 75:
            linestyle = '--'
        elif inc == 90:
            linestyle = ':'
        else:
            linestyle = '-.'

        markersize = 2

        file = f'NB{NB}_inc{inc}_LD{data[2, -1]:.1f}'

        # plot the data
        # r, v_axial, v_tangential, inflowangle, alpha, Faxial, Ftan
        axs[0, 0].plot(data[:, 0], -data[:, 1], label=f'{file}', marker='o', linestyle=linestyle, color=color, markersize=markersize)
        axs[0, 0].set_title('r vs v_axial')
        axs[0, 0].legend()

        axs[0, 1].plot(data[:, 0], -data[:, 2], label=f'{file}', marker='o', linestyle=linestyle, color=color, markersize=markersize)
        axs[0, 1].set_title('r vs v_tangential')
        axs[0, 1].legend()

        axs[0, 2].plot(data[:, 0], np.rad2deg(data[:, 3]), label=f'{file}', marker='o', linestyle=linestyle, color=color, markersize=markersize)
        axs[0, 2].set_title('r vs inflowangle')
        axs[0, 2].legend()

        axs[1, 0].plot(data[:, 0], data[:, 4], label=f'{file}', marker='o', linestyle=linestyle, color=color, markersize=markersize)
        axs[1, 0].set_title('r vs alpha')
        axs[1, 0].legend()

        axs[1, 1].plot(data[:, 0], data[:, 5], label=f'{file}', marker='o', linestyle=linestyle, color=color, markersize=markersize)
        axs[1, 1].set_title('r vs Faxial')
        axs[1, 1].legend()

        axs[1, 2].plot(data[:, 0], data[:, 6], label=f'{file}', marker='o', linestyle=linestyle, color=color, markersize=markersize)
        axs[1, 2].set_title('r vs Ftan')
        axs[1, 2].legend()

    plt.show()


plot()