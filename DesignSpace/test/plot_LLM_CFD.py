import numpy as np
import matplotlib.pyplot as plt
from plotter import set_bw_design, cumulative_to_differentiated

def plot_axial_force():
    #LLM data. load npz from DesignSpace/test/_res.npz
    data_LLM = np.load(f'./DesignSpace/test/_res.npz')
    data_LLM = data_LLM['data']
    r = data_LLM['r']
    c = data_LLM['chords']
    r_steps = r[9]-r[8]
    area =  r_steps
    f_axial_LLM = data_LLM['f_axial'] / area
    n = 50 -1


    # cfd data 
    axial_cumulative = np.genfromtxt(f'./cfdResults/results/mainy_50', skip_header=4, skip_footer=1)
    axial_cfd = cumulative_to_differentiated(axial_cumulative[:,0], axial_cumulative[:,1])

    plt.figure(figsize=(8,6))
    design = set_bw_design()

    plt.plot(axial_cumulative[:, 0]+0.1, axial_cfd, label='CFD', marker='o', color=design['colors'][0])
    plt.plot(r[:n], f_axial_LLM[:n], label='LLM', marker='x', linestyle='--', color=design['colors'][1])
    plt.xlabel('Radius [m]')
    plt.ylabel(r'$F_{axial}$')

    plt.legend()
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.show()
    #plt.savefig('./DesignSpace/test/axial_force_comparison.png', dpi=300)  


def plot_tan_force():
    #LLM data. load npz from DesignSpace/test/_res.npz
    data_LLM = np.load(f'./DesignSpace/test/_res.npz')
    data_LLM = data_LLM['data']
    r = data_LLM['r']
    c = data_LLM['chords']
    r_steps = r[1]-r[0]
    #area =  r_steps
    area =  r_steps
    f_axial_LLM = data_LLM['f_tan'] / area
    n = 50 -1


    # cfd data 
    axial_cumulative = np.genfromtxt(f'./cfdResults/results/mainx_50', skip_header=4, skip_footer=1)
    axial_cfd = cumulative_to_differentiated(axial_cumulative[:,0], axial_cumulative[:,1])

    plt.figure(figsize=(8,6))
    design = set_bw_design()

    plt.plot(axial_cumulative[:, 0]+0.1, axial_cfd, label='CFD', marker='o', color=design['colors'][0])
    plt.plot(r[:n], f_axial_LLM[:n], label='LLM', marker='x', linestyle='--', color=design['colors'][1])
    plt.xlabel('Radius [m]')
    plt.ylabel(r'$F_{tan}$')

    plt.legend()
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.show()
    #plt.savefig('./DesignSpace/test/tan_force_comparison.png', dpi=300)  

if __name__ == '__main__':
    plot_axial_force()
    plot_tan_force()