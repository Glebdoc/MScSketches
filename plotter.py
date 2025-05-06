import os, json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

        # results = np.column_stack((r.flatten(),              0
        #                         v_axial.flatten(),           1
        #                         v_tangential.flatten(),      2
        #                         inflowangle.flatten(),       3
        #                         alpha.flatten(),             4
        #                         Faxial.flatten(),            5
        #                         Ftan.flatten(),              6
        #                         Gammas.flatten(),            7
        #                         v_rot_norm.flatten(),        8
                                #v_rot_a,                      9
                                #v_rot_t,                      10
        #                         u.flatten(),                 11
                                #v.flatten(),                  12
                                #w.flatten(),                  13
                                #Cl.flatten(),                 14
                                #Cd.flatten(),                 15
                                # Re.flatten(),                16 
        #                         misc))

def plot_per_blade(axs, data, file, r_main, r_small, n_main, n_small, npM, npS, i):
    axs[0, 0].plot(r_main, -data[:n_main, 1], label=f'm1{file}', marker='o')
    axs[0, 0].plot(r_main, -data[n_main:2*n_main, 1], label=f'm2{file}', marker='o')
    axs[0, 0].plot(r_main, -data[2*n_main:3*n_main, 1], label=f'm2{file}', marker='o')

    # axs[0, 0].plot(r_small, -data_small[:, 1], label=f'{file} small', marker='o', linestyle="--")
    axs[0, 0].set_title('r vs v_axial')
    axs[0, 0].legend()

    axs[0, 1].plot(r_main, -data[:n_main, i], label=f'm1{file}', marker='o')
    axs[0, 1].plot(r_main, -data[n_main:2*n_main, i], label=f'm2{file}', marker='o')
    axs[0, 1].plot(r_main, -data[2*n_main:3*n_main, i], label=f'm2{file}', marker='o')

    # axs[0, 0].plot(r_small, -data_small[:, 1], label=f'{file} small', marker='o', linestyle="--")
    axs[0, 1].set_title('r vs v_rot')
    axs[0, 1].legend()

    axs[0, 2].plot(r_small, -data[npM:npM+n_small, i], label=f's11{file}', marker='o')
    axs[0, 2].plot(r_small, -data[npM + npS:npM+npS + n_small, i], label=f's12{file}', marker='o')
    axs[0, 2].plot(r_small, -data[npM+ 2*npS:npM+2*npS + n_small, i], label=f's13{file}', marker='o')

    # axs[0, 0].plot(r_small, -data_small[:, 1], label=f'{file} small', marker='o', linestyle="--")
    axs[0, 2].set_title('r vs v_rot_small')
    axs[0, 2].legend()

    axs[1, 0].plot(r_small, -data[npM+n_small:npM+2*n_small, i], label=f's21{file}', marker='o')
    axs[1, 0].plot(r_small, -data[npM + n_small+ npS:npM+npS + 2*n_small, i], label=f's22{file}', marker='o')
    axs[1, 0].plot(r_small, -data[npM+ 2*npS + n_small:npM+2*npS + 2*n_small, i], label=f's23{file}', marker='o')

    # axs[0, 0].plot(r_small, -data_small[:, 1], label=f'{file} small', marker='o', linestyle="--")
    axs[1, 0].set_title('r vs v_rot_small')
    axs[1, 0].legend()

    axs[1,1].plot(r_small, -data[npM+2*n_small:npM+3*n_small,i], label=f's31{file}', marker='o')
    axs[1, 1].plot(r_small, -data[npM + 2*n_small+ npS:npM+npS + 3*n_small, i], label=f's32{file}', marker='o')
    axs[1, 1].plot(r_small, -data[npM+ 2*npS + 2*n_small:npM+2*npS + 3*n_small, i], label=f's33{file}', marker='o')

    # axs[0, 0].plot(r_small, -data_small[:, 1], label=f'{file} small', marker='o', linestyle="--")
    axs[1, 1].set_title('r vs v_rot_small')
    axs[1, 1].legend()

def plot(files, show=True, title=False, misc=True):
    # Create figure and gridspec
    fig = plt.figure(figsize=(20, 12))  # Just use plt.figure, not plt.subplots
    gs = gridspec.GridSpec(3, 3, height_ratios=[1, 3, 3])  # First row for table

    # Subplots: rows 1 and 2
    axs = np.array([
        [fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1]), fig.add_subplot(gs[1, 2])],
        [fig.add_subplot(gs[2, 0]), fig.add_subplot(gs[2, 1]), fig.add_subplot(gs[2, 2])]
    ])

    # misc[0] = Thrust
    # misc[1] = Torque
    # misc[2] = LD
    # misc[3] = FM
    # misc[4] = power_required/total_power
    # misc[5] = induced_power 
    # misc[6] = profile_power
    # misc[7] = induced_power + profile_power
    # misc[8] = computed_power
    #     misc[9] = npM
    # misc[10] = npS
    # misc[11] = main_NB
    # misc[12] = small_NB
    # misc[13] = drone.main_prop.RPM
    #     misc[14] = drone.small_props[0].RPM

    labels = ['Thrust', 'Torque', 'LD', 'FM', 'η', 'P_ind', 'P_prof', 'P_tot', 'P_comp', 'npM', 'npS', 'NBm', 'NBs', 'RPMmain', 'RPMsmall', 'err',]
    table_data = []




    for file in files:
        data = np.genfromtxt(f'./results/{file}_res.csv', delimiter=',', skip_header=1)
        file = file.replace("_res.csv", "")

        npM = int(data[9, -1])  # number of points in the main rotor
        npS = int(data[10, -1])  # number of points in the small rotor
        main_NB = int(data[11, -1])
        small_NB = int(data[12, -1])

        n_main = int(npM/main_NB)
        n_small = int(npS/small_NB)

        r_small = data[npM:npM + n_small, 0]
        r_main = data[:n_main, 0]


        #plot the data
        # r, v_axial, v_tangential, inflowangle, alpha, Faxial, Ftan, Gammas, v_rot_norm, u, v, w
        v_axial = data[:, 1]
        v_tangential = data[:, 2]
        inflowangle = data[:, 3]
        alpha = data[:, 4]
        Faxial = data[:, 5]
        Ftan = data[:, 6]
        Gammas = data[:, 7]
        Cl = data[:, 14]
        Cd = data[:, 15]

        r_small_normalized = (r_small - r_small[0]) / (r_small[-1] - r_small[0])
        r_small_rescaled = r_small_normalized * (r_main[-1] - r_main[0]) + r_main[0]
        r_small = r_small_rescaled

        # # plot V_axial for the 
        # axs[0, 0].plot(r_main, -v_axial[:n_main], label=f'm1{file}', marker='o')
        # axs[0, 0].plot(r_small, -v_axial[npM:npM + n_small], label=f's1{file}', marker='o')

        # axs[0, 0].set_title('r vs v_axial')
        # axs[0, 0].legend()

        # # plot V_tangential 
        # axs[0, 1].plot(r_main, -v_tangential[:n_main], label=f'm1{file}', marker='o')
        # axs[0, 1].plot(r_small, -v_tangential[npM:npM + n_small], label=f's1{file}', marker='o')
        # axs[0, 1].set_title('r vs v_tangential')
        # axs[0, 1].legend()

        # plot Cl 
        axs[0, 0].plot(r_main, Cl[:n_main], label=f'm1{file}', marker='o')
        axs[0, 0].plot(r_small, Cl[npM:npM + n_small], label=f's1{file}', marker='o')
        axs[0, 0].set_title('r vs Cl')
        axs[0, 0].legend()

        # plot Cd
        axs[0, 1].plot(r_main, Cd[:n_main], label=f'm1{file}', marker='o')
        axs[0, 1].plot(r_small, Cd[npM:npM + n_small], label=f's1{file}', marker='o')
        axs[0, 1].set_title('r vs Cd')
        axs[0, 1].legend()

        # plot inflowangle
        axs[0, 2].plot(r_main, np.rad2deg(inflowangle[:n_main]), label=f'm1{file}', marker='o')
        axs[0, 2].plot(r_small, np.rad2deg(inflowangle[npM:npM + n_small]), label=f's1{file}', marker='o')
        axs[0, 2].set_title('r vs inflowangle')
        axs[0, 2].legend()

        # plot alpha
        axs[1, 0].plot(r_main, alpha[:n_main], label=f'm1{file}', marker='o')
        axs[1, 0].plot(r_small, alpha[npM:npM + n_small], label=f's1{file}', marker='o')
        axs[1, 0].set_title('r vs alpha')
        axs[1, 0].legend()

        # # plot Faxial
        # axs[1, 1].plot(r_main, Faxial[:n_main], label=f'm1{file}', marker='o')
        # axs[1, 1].plot(r_small, Faxial[npM:npM + n_small], label=f's1{file}', marker='o')
        # axs[1, 1].set_title('r vs Faxial')
        # axs[1, 1].legend()

        # plot Re 
        axs[1, 1].plot(r_main, data[:n_main, 16], label=f'm1{file}', marker='o')
        axs[1, 1].plot(r_small, data[npM:npM + n_small, 16], label=f's1{file}', marker='o')
        axs[1, 1].set_title('r vs Re')
        axs[1, 1].legend()
        # plot Ftan
        axs[1, 2].plot(r_main, Ftan[:n_main], label=f'm1{file}', marker='o')
        axs[1, 2].plot(r_small, Ftan[npM:npM + n_small], label=f's1{file}', marker='o')
        axs[1, 2].set_title('r vs Ftan')
        axs[1, 2].legend()

        if misc:
            data[15, -1] = 100*(data[7, -1] - data[8, -1])/data[7, -1]  # err
            values = data[:15, -1] if data.shape[0] >= 15 else np.zeros(15)
            # extend values

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

    