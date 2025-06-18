import os, json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.ticker as mtick

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
    # cfd_faxial_heli = np.array([	13.857112,
    #                         36.955533,
    #                         74.822902,
    #                         125.97218,
    #                         188.47118,
    #                         259.16722,
    #                         328.30965,
    #                         412.20165,
    #                         465.96409])
    cfd_faxial_heli = np.array([	11.01122,
                                    32.934581,
                                    70.070923,
                                    121.09236,
                                    184.45948,
                                    257.45775,
                                    328.69693,
                                    424.60523,
                                    470.37266
                                    ])
                                        
    # cfd_ftan_heli = np.array([	3.7062831,
    #                         8.6494665,
    #                         15.197173,
    #                         22.570386,
    #                         30.259358,
    #                         37.539395,
    #                         43.719315,
    #                         49.108042,
    #                         54.728354])

    cfd_ftan_heli = np.array([	3.2781992,
                                8.1568434,
                                14.819917,
                                22.339605,
                                30.172741,
                                37.468915,
                                43.907587,
                                48.866468,
                                54.690536
                                ])
                                    
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
    fig = plt.figure(figsize=(16, 8))  # Just use plt.figure, not plt.subplots
    gs = gridspec.GridSpec(3, 4, height_ratios=[1.2, 3, 3])  # First row for table

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
        #plot_per_blade(axs, data, file, r_main, r_small, n_main, n_small, npM, npS, 1)

        # plot Cl 
        axs[0, 0].plot(r_main, Cl[:n_main], label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[0, 0].plot(r_small, Cl[npM:npM + n_small], label=f's1{file}', marker='x',  color=color, linestyle="--")   
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_cl.txt', skip_header=3)
            axs[0, 0].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[0, 0].set_title('r vs Cl')
        #axs[0, 0].legend()

        # plot Cd
        axs[0, 1].plot(r_main, Cd[:n_main], label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[0, 1].plot(r_small, Cd[npM:npM + n_small], label=f's1{file}', marker='x', color=color, linestyle="--")
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_cd.txt', skip_header=3)
            axs[0, 1].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[0, 1].set_title('r vs Cd')
        #axs[0, 1].legend()

        # plot Re 
        axs[0, 2].plot(r_main, data[:n_main, 16], label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[0, 2].plot(r_small, data[npM:npM + n_small, 16], label=f's1{file}', marker='x', color=color, linestyle="--")
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_re.txt', skip_header=3)
            axs[0, 2].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[0, 2].set_title('r vs Re')
        #axs[0, 2].legend()

        # plot v_axial
        axs[0, 3].plot(r_main, v_axial[:n_main], label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[0, 3].plot(r_small, v_axial[npM:npM + n_small], label=f's1{file}', marker='x', color=color, linestyle="--")
        # if QBlade:
        #     data_qb = np.genfromtxt(f'./QBlade_vaxial.txt', skip_header=3)
        #     axs[0, 3].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[0, 3].set_title('r vs v_axial')
        #axs[0, 3].legend()
        

        # plot inflowangle
        axs[1, 0].plot(r_main, np.rad2deg(inflowangle[:n_main]), label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[1, 0].plot(r_small, np.rad2deg(inflowangle[npM:npM + n_small]), label=f's1{file}', marker='x', color=color, linestyle="--")
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_inflow.txt', skip_header=3)
            axs[1, 0].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[1, 0].set_title('r vs inflowangle')
        #axs[1, 0].legend()

        # plot alpha
        axs[1, 1].plot(r_main, alpha[:n_main], label=f'm1{file}', marker='o', color=color)
        if not helicopter:
            axs[1, 1].plot(r_small, alpha[npM:npM + n_small], label=f's1{file}', marker='x', color=color, linestyle="--")
        if QBlade:
            data_qb = np.genfromtxt(f'./QBlade_aoa.txt', skip_header=3)
            axs[1, 1].plot(data_qb[:, 0], data_qb[:, 1], label=f'QBlade', marker='o')
        axs[1, 1].set_title('r vs alpha')
        #axs[1, 1].legend()

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
        #axs[1, 2].legend()

        
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
        #axs[1, 3].legend()

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
    handles, labels = axs[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='center left', bbox_to_anchor=(0.84, 0.5), fontsize='large')
    if title:
        plt.tight_layout(rect=[0.05, 0, 0.9, 1])
        plt.savefig(f'./plots/fig_{title}.png', dpi = 500)
    
    if show:
        plt.show()


def tipRotorForces(cfd_file, ll_files):
    cfd_data = np.genfromtxt(f'./cfdResults/{cfd_file}', delimiter=',', skip_header=1)
    ll_data = np.genfromtxt(f'./results/{ll_files}_res.csv', delimiter=',', skip_header=1)

    # plot 2 subplots for faxial and Ftan
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    # plot cfd data
    r_cfd = cfd_data[:, 4]
    area_cfd = cfd_data[:, 5]
    f_axial_b1 = cfd_data[:, 6]
    f_axial_b2 = cfd_data[:, 7]
    f_axial_b3 = cfd_data[:, 8]

    f_tan_b1 = cfd_data[:, 9]/r_cfd
    f_tan_b2 = cfd_data[:, 10]/r_cfd
    f_tan_b3 = cfd_data[:, 11]/r_cfd

    axs[0].plot(r_cfd, f_axial_b1/area_cfd, label='CFD Blade 1', marker='o', color='blue')
    axs[0].plot(r_cfd, f_axial_b2/area_cfd, label='CFD Blade 2', marker='o', color='red')
    axs[0].plot(r_cfd, f_axial_b3/area_cfd, label='CFD Blade 3', marker='o', color='green')

    axs[1].plot(r_cfd, f_tan_b1/area_cfd, label='CFD Blade 1', marker='o', color='blue')
    axs[1].plot(r_cfd, f_tan_b2/area_cfd, label='CFD Blade 2', marker='o', color='red')
    axs[1].plot(r_cfd, f_tan_b3/area_cfd, label='CFD Blade 3', marker='o', color='green')

    # plot ll data
    data = ll_data
    npM = int(data[9, -1])  # number of points in the main rotor
    npS = int(data[10, -1])  # number of points in the small rotor
    small_NB = int(data[12, -1])
    n_small = int(npS/small_NB)
    r_small = data[npM:npM + n_small, 0]
    chords = data[:, 18]
    area = chords[npM:npM + n_small]*(r_small[1]- r_small[0])  # assuming uniform chord length

    f_axial_b1_ll = data[npM:npM + n_small, 5]
    f_axial_b2_ll = data[npM + n_small :npM + n_small*2, 5]
    f_axial_b3_ll = data[npM + 2*n_small:npM + 3*n_small, 5]

    f_tan_b1_ll = data[npM:npM + n_small, 6]
    f_tan_b2_ll = data[npM + n_small :npM + n_small*2, 6]
    f_tan_b3_ll = data[npM + 2*n_small:npM + 3*n_small, 6]

    axs[0].plot(r_small, f_axial_b1_ll/area, label='LL Blade 1', marker='x', linestyle='--', color='blue')
    axs[0].plot(r_small, f_axial_b2_ll/area, label='LL Blade 2', marker='x', linestyle='--', color='red')
    axs[0].plot(r_small, f_axial_b3_ll/area, label='LL Blade 3', marker='x', linestyle='--', color='green')

    axs[1].plot(r_small, f_tan_b1_ll/area, label='LL Blade 1', marker='x', linestyle='--', color='blue')
    axs[1].plot(r_small, f_tan_b2_ll/area, label='LL Blade 2', marker='x', linestyle='--', color='red')
    axs[1].plot(r_small, f_tan_b3_ll/area, label='LL Blade 3', marker='x', linestyle='--', color='green')



    axs[0].legend()
    axs[1].legend()

    # add a second axis for the relative error
    ax0_err = axs[0].twinx()
    ax1_err = axs[1].twinx()

    # compute relative error
    # interpolate f_axial

    f_axial_b1 = np.interp(r_small, r_cfd, f_axial_b1/area_cfd)
    f_axial_b2 = np.interp(r_small, r_cfd, f_axial_b2/area_cfd)
    f_axial_b3 = np.interp(r_small, r_cfd, f_axial_b3/area_cfd)

    f_tan_b1 = np.interp(r_small, r_cfd, f_tan_b1/area_cfd)
    f_tan_b2 = np.interp(r_small, r_cfd, f_tan_b2/area_cfd)
    f_tan_b3 = np.interp(r_small, r_cfd, f_tan_b3/area_cfd)

    rel_err_axial_b1 = np.abs((f_axial_b1_ll/area - f_axial_b1) / np.maximum(np.abs(f_axial_b1), 1e-12)) * 100
    rel_err_axial_b2 = np.abs((f_axial_b2_ll/area - f_axial_b2) / np.maximum(np.abs(f_axial_b2), 1e-12)) * 100
    rel_err_axial_b3 = np.abs((f_axial_b3_ll/area - f_axial_b3) / np.maximum(np.abs(f_axial_b3), 1e-12)) * 100

    rel_err_tan_b1 = np.abs((f_tan_b1_ll/area - f_tan_b1) / np.maximum(np.abs(f_tan_b1), 1e-12)) * 100
    rel_err_tan_b2 = np.abs((f_tan_b2_ll/area - f_tan_b2) / np.maximum(np.abs(f_tan_b2), 1e-12)) * 100
    rel_err_tan_b3 = np.abs((f_tan_b3_ll/area - f_tan_b3) / np.maximum(np.abs(f_tan_b3), 1e-12)) * 100

    ax0_err.plot(r_small, rel_err_axial_b1, '--', label='RelErr Blade 1', color='blue', alpha=0.3)
    ax0_err.plot(r_small, rel_err_axial_b2, '--', label='RelErr Blade 2', color='red', alpha=0.3)
    ax0_err.plot(r_small, rel_err_axial_b3, '--', label='RelErr Blade 3', color='green', alpha=0.3)

    ax1_err.plot(r_small, rel_err_tan_b1, '--', label='RelErr Blade 1', color='blue', alpha=0.3)
    ax1_err.plot(r_small, rel_err_tan_b2, '--', label='RelErr Blade 2', color='red', alpha=0.3)
    ax1_err.plot(r_small, rel_err_tan_b3, '--', label='RelErr Blade 3', color='green', alpha=0.3)

    ax0_err.yaxis.set_major_formatter(mtick.PercentFormatter())
    ax1_err.yaxis.set_major_formatter(mtick.PercentFormatter())

    ax0_err.legend(loc='upper right')
    ax1_err.legend(loc='upper right')

    plt.legend()
    plt.show()


def convergence(files):
    # define 2 subplots 
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))
    
    # Load all data for easier reference
    all_data = [np.genfromtxt(f'./cfdResults/{file}.csv', delimiter=',', skip_header=1) for file in files]
    
    r_main = all_data[0][:, 0]  # assume all share the same r
    area_main = all_data[0][:, 3]  # same for area
    f_axial_ref = all_data[-1][:, 1]
    f_tangential_ref = all_data[-1][:, 2]

    # Plot main forces (axial/tangential)
    for data, file in zip(all_data, files):
        f_axial = data[:, 1]
        f_tangential = data[:, 2]
        axs[0].plot(r_main, f_axial / area_main, label=file, marker='o')
        axs[1].plot(r_main, f_tangential / area_main, label=file, marker='o')

    axs[0].set_ylabel("Axial Force / Area")
    axs[1].set_ylabel("Tangential Force / Area")
    axs[1].set_xlabel("Radius")

    axs[0].legend()

    # Create twin axes for relative error
    ax0_err = axs[0].twinx()
    ax1_err = axs[1].twinx()

    for data, file in zip(all_data[0:-1], files[0:-1]):
        f_axial = data[:, 1]
        f_tangential = data[:, 2]

        # Compute relative error (avoiding division by zero)
        rel_err_axial = np.abs((f_axial - f_axial_ref) / np.maximum(np.abs(f_axial_ref), 1e-12))*100 
        rel_err_tangential = np.abs((f_tangential - f_tangential_ref) / np.maximum(np.abs(f_tangential_ref), 1e-12))*100

        ax0_err.plot(r_main, rel_err_axial, '--', label=f'RelErr {file}')
        ax1_err.plot(r_main, rel_err_tangential, '--', label=f'RelErr {file}')

    ax0_err.set_ylabel("Relative Error (Axial)")
    ax1_err.set_ylabel("Relative Error (Tangential)")

    # Optional: show both legends (use a common one if needed)
    lines, labels = axs[1].get_legend_handles_labels()
    lines2, labels2 = ax1_err.get_legend_handles_labels()
    axs[1].legend(lines + lines2, labels + labels2)

    plt.tight_layout()
    plt.show()


# def mainRotorForces(cfd_file, ll_files):
#     cfd_data = np.genfromtxt(f'./cfdResults/{cfd_file}', delimiter=',', skip_header=1)
#     ll_data = np.genfromtxt(f'./results/{ll_files}_res.csv', delimiter=',', skip_header=1)

#     # plot 2 subplots for faxial and Ftan
#     fig, axs = plt.subplots(2, 1, figsize=(10, 8))

#     # plot cfd data
#     r_cfd = cfd_data[:, 0]
#     area_cfd = cfd_data[:, 3]
#     f_axial = cfd_data[:, 1]
#     f_tan = cfd_data[:, 2]

#     axs[0].plot(r_cfd, f_axial/area_cfd, label='CFD', marker='o', color='blue')
#     axs[1].plot(r_cfd, f_tan/area_cfd, label='CFD', marker='o', color='blue')

#     # plot ll data
#     data = ll_data
#     npM = int(data[9, -1])  # number of points in the main rotor
#     main_NB = int(data[11, -1])
#     n_main = int(npM/main_NB)
#     r_main = data[:n_main, 0]
#     chords = data[:, 18]
#     area = chords[:n_main]*(r_main[1]- r_main[0])  # assuming uniform chord length

#     f_axial_ll = data[:n_main, 5]
#     f_tan_ll = data[:n_main, 6]

#     axs[0].plot(r_main, f_axial_ll/area, label='LL', marker='x', linestyle='--', color='red')
#     axs[1].plot(r_main, f_tan_ll/area, label='LL', marker='x', linestyle='--', color='red')

#     # add a second axis for the relative error
#     ax0_err = axs[0].twinx()
#     ax1_err = axs[1].twinx()

#     # interpolate cfd data
#     f_axial_interp = np.interp(r_main, r_cfd, f_axial/area_cfd)
#     f_tan_interp = np.interp(r_main, r_cfd, f_tan/area_cfd)

#     # compute relative error
#     rel_err_axial = np.abs((f_axial_ll/area - f_axial_interp) / np.maximum(np.abs(f_axial_interp), 1e-12)) * 100
#     rel_err_tan = np.abs((f_tan_ll/area - f_tan_interp) / np.maximum(np.abs(f_tan_interp), 1e-12)) * 100

#     ax0_err.plot(r_main, rel_err_axial, '--', label='RelErr Axial', color='blue', alpha=0.3)
#     ax1_err.plot(r_main, rel_err_tan, '--', label='RelErr Tangential', color='blue', alpha=0.3)

#     ax0_err.yaxis.set_major_formatter(mtick.PercentFormatter())
#     ax1_err.yaxis.set_major_formatter(mtick.PercentFormatter())

#     ax0_err.legend(loc='upper right')
#     ax1_err.legend(loc='upper right')

#     axs[0].set_ylabel("Axial Force / Area")
#     axs[1].set_ylabel("Tangential Force / Area")
#     axs[1].set_xlabel("Radius")
#     axs[0].legend()
#     axs[1].legend()
#     plt.tight_layout()
#     plt.show()


def mainRotorForces(cfd_file, ll_files):
    # Load CFD data
    cfd_data = np.genfromtxt(f'./cfdResults/{cfd_file}', delimiter=',', skip_header=1)
    r_cfd = cfd_data[:, 0]
    area_cfd = cfd_data[:, 3]
    f_axial = cfd_data[:, 1]
    f_tan = cfd_data[:, 2]

    # Plot setup
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    # Plot CFD
    axs[0].plot(r_cfd, f_axial / area_cfd, label='CFD', marker='o', color='blue')
    axs[1].plot(r_cfd, f_tan / area_cfd, label='CFD', marker='o', color='blue')

    ###########################################
    ax0_err = axs[0].twinx()
    ax1_err = axs[1].twinx()
    alpha=1
    for ll_file in ll_files:
        alpha-=0.9/len(ll_files) # Decrease alpha for each LL file to make them more transparent

        # Load LL data
        ll_data = np.genfromtxt(f'./results/{ll_file}_res.csv', delimiter=',', skip_header=1)
        npM = int(ll_data[9, -1])  # number of points in the main rotor
        main_NB = int(ll_data[11, -1])
        n_main = int(npM / main_NB)
        r_ll = ll_data[:n_main, 0]
        chords = ll_data[:, 18]
        area_ll = chords[:n_main] * (r_ll[1] - r_ll[0])  # assuming uniform spacing

        f_axial_ll = ll_data[:n_main, 5]
        f_tan_ll = ll_data[:n_main, 6]

        # Plot LL (interpolated)
        axs[0].plot(r_ll, f_axial_ll/area_ll, label='LL', marker='x', linestyle='--', color='red', alpha=alpha)
        axs[1].plot(r_ll, f_tan_ll/area_ll, label='LL', marker='x', linestyle='--', color='red')

        # Interpolate LL data to CFD radius points
        f_axial_ll_interp = np.interp(r_cfd, r_ll, f_axial_ll / area_ll)
        f_tan_ll_interp = np.interp(r_cfd, r_ll, f_tan_ll / area_ll)

        # Compute relative error
        rel_err_axial = np.abs((f_axial / area_cfd - f_axial_ll_interp) / np.maximum(np.abs(f_axial / area_cfd), 1e-12)) * 100
        rel_err_tan = np.abs((f_tan / area_cfd - f_tan_ll_interp) / np.maximum(np.abs(f_tan / area_cfd), 1e-12)) * 100

        # Add twin axes for error
        
        ax0_err.plot(r_cfd, rel_err_axial, '--', label='RelErr Axial', color='blue', alpha=0.3)
        ax1_err.plot(r_cfd, rel_err_tan, '--', label='RelErr Tangential', color='blue', alpha=0.3)

        
        # compute the average relative error
        avg_rel_err_axial = np.mean(rel_err_axial)
        avg_rel_err_tan = np.mean(rel_err_tan)

        

        # plot average relative error
        ax0_err.axhline(avg_rel_err_axial, color='black', linestyle='--',
                label=f'Avg RelErr {ll_file.split("_")[-1]}: {avg_rel_err_axial:.2f}%', alpha=0.5)

        ax1_err.axhline(avg_rel_err_tan, color='black', linestyle='--',
                label=f'Avg RelErr {ll_file.split("_")[-1]}: {avg_rel_err_tan:.2f}%', alpha=0.5)


    ax0_err.legend(loc='upper center')
    ax1_err.legend(loc='upper center')

    ax0_err.yaxis.set_major_formatter(mtick.PercentFormatter())
    ax1_err.yaxis.set_major_formatter(mtick.PercentFormatter())
    

    axs[0].set_ylabel("Axial Force / Area")
    axs[1].set_ylabel("Tangential Force / Area")
    axs[1].set_xlabel("Radius")
    axs[0].legend()
    axs[1].legend()

    # add narrow grid 
    axs[0].grid(True, linestyle='--', linewidth=0.5)
    axs[1].grid(True, linestyle='--', linewidth=0.5)

    plt.tight_layout()
    plt.show()

    
#tipRotorForces('drone8040_7445_CFD_1500.csv', 'drone8040_7445_n80_newNPZ_blade1_angle15' )
#convergence(['drone8040_7445_CFD_1500','drone8040_7445_CFD_2000'])
#mainRotorForces('drone8040_7445_CFD_2000.csv', ['drone8040_7445_n80_newNPZ_blade1_angle15', 'drone8040_7445_n80_newNPZ_blade1_angle30', 'drone8040_7445_n80_newNPZ_blade1_angle45', 'drone8040_7445_n80_newNPZ_blade1_angle60', 'drone8040_7445_n80_newNPZ_blade1_angle85', 'drone8040_7445_n80_newNPZ_blade1_angle95'])
mainRotorForces('drone8040_7445_CFD_2000.csv', ['drone8040_7445_SWE_blade1_angle0', 'drone8040_7445_SWE_wake_length5', 'drone8040_7445_ppr60_wake_length8'])