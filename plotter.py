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
                                # Lift_unitspan.flatten(), 17
                                # chords.flatten(),           18
        #                         misc))

# misc[0] = Thrust
    # misc[1] = Torque
    # misc[2] = LD
    # misc[3] = FM
    # misc[4] = power_required/total_power
    # misc[5] = induced_power 
    # misc[6] = profile_power
    # misc[7] = induced_power + profile_power
    # misc[8] = computed_power
    # misc[9] = npM
    # misc[10] = npS
    # misc[11] = main_NB
    # misc[12] = small_NB
    # misc[13] = drone.main_prop.RPM
    # misc[14] = drone.small_props[0].RPM
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

def plot(files, show=True, title=False, misc=True, helicopter=False, QBlade=False):
    plt.close()
    cfd_faxial_heli = np.array([	13.857112,
                            36.955533,
                            74.822902,
                            125.97218,
                            188.47118,
                            259.16722,
                            328.30965,
                            412.20165,
                            465.96409])
    
    cfd_ftan_heli = np.array([	3.7062831,
                            8.6494665,
                            15.197173,
                            22.570386,
                            30.259358,
                            37.539395,
                            43.719315,
                            49.108042,
                            54.728354])
    cfd_r_heli = np.linspace(0.15, 0.95, len(cfd_faxial_heli))

    cfd_r_drone = np.array([0.15,
                            0.25,
                            0.35,
                            0.45,
                            0.55,
                            0.65,
                            0.75,
                            0.8125,
                            0.8375,
                            0.8625,
                            0.8875,
                            0.9125,
                            0.9375,
                            0.9625,
                            0.9875
                            ])
    cfd_faxial_drone = np.array([21.203668,
                                    39.070535,
                                    79.876164,
                                    134.43845,
                                    200.86617,
                                    276.10326,
                                    357.24676,
                                    413.95608,
                                    447.48172,
                                    486.57392,
                                    535.35868,
                                    597.4842,
                                    689.16336,
                                    704.75084,
                                    673.85792])
    cfd_ftan_drone = np.array([4.2645645,
                                9.2669645,
                                16.458348,
                                24.738673,
                                33.454984,
                                41.832769,
                                49.563131,
                                54.302936,
                                56.503592,
                                58.01602,
                                59.332168,
                                65.553364,
                                58.010724,
                                39.0764828,
                                78.72962])



    # Create figure and gridspec
    fig = plt.figure(figsize=(25, 8))  # Just use plt.figure, not plt.subplots
    gs = gridspec.GridSpec(3, 4, height_ratios=[1, 3, 3])  # First row for table

    # Subplots: rows 1 and 2
    axs = np.array([
        [fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1]), fig.add_subplot(gs[1, 2]), fig.add_subplot(gs[1, 3])],
        [fig.add_subplot(gs[2, 0]), fig.add_subplot(gs[2, 1]), fig.add_subplot(gs[2, 2]), fig.add_subplot(gs[2, 3])]
    ])

    

    labels = ['Thrust', 'Torque', 'LD', 'FM', 'η', 'P_ind', 'P_prof', 'P_tot', 'P_comp', 'npM', 'npS', 'NBm', 'NBs', 'RPMmain', 'RPMsmall', 'err',]
    table_data = []



    colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
    count = 0
    for file in files:

        data = np.genfromtxt(f'./results/{file}_res.csv', delimiter=',', skip_header=1)
        file = file.replace("_res.csv", "")

        npM = int(data[9, -1])  # number of points in the main rotor
        main_NB = int(data[11, -1])
        n_main = int(npM/main_NB)
        r_main = data[:n_main, 0]


        if not helicopter:
            npS = int(data[10, -1])  # number of points in the small rotor
            small_NB = int(data[12, -1])
            n_small = int(npS/small_NB)
            r_small = data[npM:npM + n_small, 0]
            r_small_normalized = (r_small - r_small[0]) / (r_small[-1] - r_small[0])
            r_small_rescaled = r_small_normalized * (r_main[-1] - r_main[0]) + r_main[0]
            r_small = r_small_rescaled
        
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
        L_unitspan = data[:, 17]
        chords = data[:, 18]

        area = chords[:n_main-1] * (r_main[1:] - r_main[:-1])  # assuming uniform chord length

        color = colors[count % len(colors)]

        # plot Cl 
        axs[0, 0].plot(r_main, Cl[:n_main], label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[0, 0].plot(r_small, Cl[npM:npM + n_small], label=f's1{file}', marker='x',  color=color, linestyle="--")   
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_cl.txt', skip_header=3)
            axs[0, 0].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[0, 0].set_title('r vs Cl')
        axs[0, 0].legend()

        # plot Cd
        axs[0, 1].plot(r_main, Cd[:n_main], label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[0, 1].plot(r_small, Cd[npM:npM + n_small], label=f's1{file}', marker='x', color=color, linestyle="--")
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_cd.txt', skip_header=3)
            axs[0, 1].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[0, 1].set_title('r vs Cd')
        axs[0, 1].legend()

        # plot Re 
        axs[0, 2].plot(r_main, data[:n_main, 16], label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[0, 2].plot(r_small, data[npM:npM + n_small, 16], label=f's1{file}', marker='x', color=color, linestyle="--")
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_re.txt', skip_header=3)
            axs[0, 2].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[0, 2].set_title('r vs Re')
        axs[0, 2].legend()

        # plot v_axial
        axs[0, 3].plot(r_main, v_axial[:n_main], label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[0, 3].plot(r_small, v_axial[npM:npM + n_small], label=f's1{file}', marker='x', color=color, linestyle="--")
        # if QBlade:
        #     data_qb = np.genfromtxt(f'./QBlade_vaxial.txt', skip_header=3)
        #     axs[0, 3].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[0, 3].set_title('r vs v_axial')
        axs[0, 3].legend()
        

        # plot inflowangle
        axs[1, 0].plot(r_main, np.rad2deg(inflowangle[:n_main]), label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[1, 0].plot(r_small, np.rad2deg(inflowangle[npM:npM + n_small]), label=f's1{file}', marker='x', color=color, linestyle="--")
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_inflow.txt', skip_header=3)
            axs[1, 0].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[1, 0].set_title('r vs inflowangle')
        axs[1, 0].legend()

        # plot alpha
        axs[1, 1].plot(r_main, alpha[:n_main], label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[1, 1].plot(r_small, alpha[npM:npM + n_small], label=f's1{file}', marker='x', color=color, linestyle="--")
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_aoa.txt', skip_header=3)
            axs[1, 1].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[1, 1].set_title('r vs alpha')
        axs[1, 1].legend()

        # plot Faxial
        axs[1, 2].plot(r_main[:-1], Faxial[:n_main-1]/area, label=f'm1{file}', marker='o', color=color)
        # lift_unitspan = data[:n_main, 17]
        axs[1, 2].plot(cfd_r_heli, cfd_faxial_heli, label=f'heli CFD Faxial/unitspan', marker='o', color=color, linestyle="--")
        if not helicopter:
            axs[1, 2].plot(r_small, Faxial[npM:npM + n_small], label=f's1{file}', marker='x', color=color, linestyle="--")
            axs[1, 2].plot(cfd_r_drone, cfd_faxial_drone, label=f'drone CFD Faxial/unitspan', marker='x', color=color, linestyle="--")
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_Faxial.txt', skip_header=3)
            axs[1, 2].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[1, 2].set_title('r vs Faxial')
        axs[1, 2].legend()

        
        # plot Ftan
        axs[1, 3].plot(r_main[:-1], Ftan[:n_main-1]/area, label=f'm1{file}', marker='o', color=color)
        axs[1, 3].plot(cfd_r_heli, cfd_ftan_heli, label=f'heli CFD Ftan/unitspan', marker='o', color=color, linestyle="--")

        if not helicopter:
            axs[1, 3].plot(r_small, Ftan[npM:npM + n_small], label=f's1{file}', marker='x', color=color, linestyle="--")
            axs[1, 3].plot(cfd_r_drone, cfd_ftan_drone, label=f'drone CFD Ftan/unitspan', marker='x', color=color, linestyle="--")
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_Ftan.txt', skip_header=3)
            axs[1, 3].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[1, 3].set_title('r vs Ftan')
        axs[1, 3].legend()

        if misc:
            data[15, -1] = 100*(data[7, -1] - data[8, -1])/data[7, -1]  # err
            values = data[:15, -1] if data.shape[0] >= 15 else np.zeros(15)
            # extend values

            row = [file] + [f"{v:.2f}" for v in values]
            table_data.append(row)
        count += 1
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


def plotForces(files, misc=True):
    plt.close()
    cfd_faxial_heli = np.array([	13.857112,
                            36.955533,
                            74.822902,
                            125.97218,
                            188.47118,
                            259.16722,
                            328.30965,
                            412.20165,
                            465.96409])
    
    cfd_ftan_heli = np.array([	3.7062831,
                            8.6494665,
                            15.197173,
                            22.570386,
                            30.259358,
                            37.539395,
                            43.719315,
                            49.108042,
                            54.728354])
    cfd_r_heli = np.linspace(0.15, 0.95, len(cfd_faxial_heli))

    cfd_r_drone = np.array([0.15,
                            0.25,
                            0.35,
                            0.45,
                            0.55,
                            0.65,
                            0.75,
                            0.8125,
                            0.8375,
                            0.8625,
                            0.8875,
                            0.9125,
                            0.9375,
                            0.9625,
                            0.9875
                            ])
    cfd_faxial_drone = np.array([21.203668,
                                    39.070535,
                                    79.876164,
                                    134.43845,
                                    200.86617,
                                    276.10326,
                                    357.24676,
                                    413.95608,
                                    447.48172,
                                    486.57392,
                                    535.35868,
                                    597.4842,
                                    689.16336,
                                    704.75084,
                                    673.85792])
    cfd_ftan_drone = np.array([4.2645645,
                                9.2669645,
                                16.458348,
                                24.738673,
                                33.454984,
                                41.832769,
                                49.563131,
                                54.302936,
                                56.503592,
                                58.01602,
                                59.332168,
                                65.553364,
                                58.010724,
                                39.0764828,
                                78.72962])

    fig, axs = plt.subplots(1, 2, figsize=(10, 6))

    colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
    count = 0
    for file in files:

        data = np.genfromtxt(f'./results/{file}_res.csv', delimiter=',', skip_header=1)
        file = file.replace("_res.csv", "")

        npM = int(data[9, -1])  # number of points in the main rotor
        main_NB = int(data[11, -1])
        n_main = int(npM/main_NB)
        r_main = data[:n_main, 0]

        try:
            npS = int(data[10, -1])  # number of points in the small rotor
            small_NB = int(data[12, -1])
            n_small = int(npS/small_NB)
            r_small = data[npM:npM + n_small, 0]
            r_small_normalized = (r_small - r_small[0]) / (r_small[-1] - r_small[0])
            r_small_rescaled = r_small_normalized * (r_main[-1] - r_main[0]) + r_main[0]
            r_small = r_small_rescaled
        except:
            n_small = 0
            small_NB = 0
            r_small = np.array([])
            
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
        L_unitspan = data[:, 17]
        chords = data[:, 18]

        area = chords[:n_main-1] * (r_main[1:] - r_main[:-1])  # assuming uniform chord length

        color = colors[count % len(colors)]

        # plot Faxial
        axs[0].plot(r_main[:-1], Faxial[:n_main-1]/area, label=f'm1{file}', marker='o', color=color)
        
        

        # plot Ftan
        axs[1].plot(r_main[:-1], Ftan[:n_main-1]/area, label=f'm1{file}', marker='o', color=color)
        
        

        count += 1
    axs[0].plot(cfd_r_heli, cfd_faxial_heli, label=f'heli CFD Faxial/unitspan', marker='o', color='black', linestyle="--")
    axs[0].plot(cfd_r_drone, cfd_faxial_drone, label=f'drone CFD Faxial/unitspan', marker='x', color='red', linestyle="--")
    axs[0].set_xlabel('r')
    axs[0].set_ylabel('Faxial/unitspan')
    axs[0].grid()
    axs[0].legend()

    axs[1].plot(cfd_r_heli, cfd_ftan_heli, label=f'heli CFD Ftan/unitspan', marker='o', color='black', linestyle="--")
    axs[1].plot(cfd_r_drone, cfd_ftan_drone, label=f'drone CFD Ftan/unitspan', marker='x', color='red', linestyle="--")
    axs[1].set_xlabel('r')
    axs[1].set_ylabel('Ftan/unitspan')
    axs[1].grid()
    axs[1].legend()

    plt.show()

plotForces(files=['exp_drone_chord_tip0.1', 'YourMomsHeli_chord_tip0.1', 'YourMomsHeli_wake_length2', 'YourMomsHeli_wake_length4', 'YourMomsHeli_wake_length6'], misc=True)
#plot(files=['exp_drone_chord_tip0.1'], show=True, title='no', misc=True, helicopter=False, QBlade=False)