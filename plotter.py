import os, json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def plot(files, show=True, title=False, misc=True):
    # Create figure and gridspec
    fig = plt.figure(figsize=(15, 12))  # Just use plt.figure, not plt.subplots
    gs = gridspec.GridSpec(3, 3, height_ratios=[1, 3, 3])  # First row for table

    # Subplots: rows 1 and 2
    axs = np.array([
        [fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1]), fig.add_subplot(gs[1, 2])],
        [fig.add_subplot(gs[2, 0]), fig.add_subplot(gs[2, 1]), fig.add_subplot(gs[2, 2])]
    ])

    labels = ['Thrust', 'Torque', 'LD', 'FM', 'η']
    table_data = []


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
    
        if misc:
            
            values = data[:5, -1] if data.shape[0] >= 5 else np.zeros(5)
            row = [file] + [f"{v:.2f}" for v in values]
            table_data.append(row)
    col_labels = ['Config'] + labels
    ax_table = fig.add_subplot(gs[0, :])
    ax_table.axis('off')
    table = ax_table.table(cellText=table_data, colLabels=col_labels, loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.5)
    if title:
        plt.tight_layout(rect=[0, 0, 0.75, 1])
        plt.savefig(f'./plots/fig_{title}.png', dpi = 500)
    
    if show:
        plt.show()