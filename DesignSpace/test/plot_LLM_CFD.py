import os
import numpy as np
import matplotlib.pyplot as plt
from plotter import set_bw_design, smooth_cumulative_to_distribution


def _label_from_path(path: str) -> str:
    """Create a short label from the npz path."""
    folder = os.path.basename(os.path.dirname(path))
    return folder if folder != '' else os.path.basename(path)


def plot_axial_force(llm_paths):
    """
    Plot CFD axial force and one or more LLM axial force distributions.
    Includes absolute relative error on secondary y-axis.
    """
    # ---- CFD data ----
    axial_cumulative = np.genfromtxt(
        './cfdResults/results/mainy_50',
        skip_header=4,
        skip_footer=1
    )
    r_cfd = axial_cumulative[:-1, 0] + 0.1
    f_cfd = smooth_cumulative_to_distribution(
        axial_cumulative[:-1, 0],
        axial_cumulative[:-1, 1]
    )

    design = set_bw_design()
    colors = design['colors']

    fig, ax1 = plt.subplots(figsize=(8, 6))
    ax2 = ax1.twinx()  # Create secondary y-axis

    # CFD curve on primary axis
    ax1.plot(
        r_cfd, f_cfd,
        label='CFD',
        marker='o',
        color=colors[0],
        linewidth=2
    )

    # ---- LLM curves ----
    for i, path in enumerate(llm_paths):
        data_LLM = np.load(path)['data']
        r = data_LLM['r']
        r_steps = r[9] - r[8]
        area = r_steps
        f_axial_LLM = data_LLM['f_axial'] / area
        n = 50 - 1

        label = f'LL model'
        color = colors[(i + 1) % len(colors)]

        # Plot LLM on primary axis
        ax1.plot(
            r[:n], f_axial_LLM[:n],
            label=label,
            marker='x',
            linestyle='--',
            color=color
        )

        # Interpolate CFD to LLM radial positions for error calculation
        f_cfd_interp = np.interp(r[:n], r_cfd, f_cfd)
        
        # Calculate absolute relative error: |CFD - LLM| / |CFD| * 100%
        rel_error = np.abs(f_cfd_interp - f_axial_LLM[:n]) / np.abs(f_cfd_interp) * 100
        
        # Plot error on secondary axis
        ax2.plot(
            r[:n], rel_error,
            label=f'Error ({label})',
            marker='s',
            linestyle=':',
            color=color,
            alpha=0.7
        )

    # Primary axis formatting
    ax1.set_xlabel('Radius [m]')
    ax1.set_ylabel(r'$F_{axial}$ [N/m]', color='black')
    ax1.tick_params(axis='y', labelcolor='black')
    ax1.grid(True, linestyle='--', linewidth=0.5)
    ax1.legend(loc='upper left')

    # Secondary axis formatting
    ax2.set_ylabel('Absolute Relative Error [%]')
    ax2.tick_params(axis='y')
    ax2.legend(loc='upper right')

    fig.tight_layout()
    fig.savefig('./DesignSpace/test/axial_force_comparison.png', dpi=300)


def plot_tan_force(llm_paths):
    """
    Plot CFD tangential force and one or more LLM tangential force distributions.
    Includes absolute relative error on secondary y-axis.
    """
    # ---- CFD data ----
    axial_cumulative = np.genfromtxt(
        './cfdResults/results/mainx_50',
        skip_header=4,
        skip_footer=1
    )
    r_cfd = axial_cumulative[:-1, 0] + 0.1
    f_cfd = smooth_cumulative_to_distribution(
        axial_cumulative[:-1, 0],
        axial_cumulative[:-1, 1]
    )

    design = set_bw_design()
    colors = design['colors']

    fig, ax1 = plt.subplots(figsize=(8, 6))
    ax2 = ax1.twinx()  # Create secondary y-axis

    # CFD curve on primary axis
    ax1.plot(
        r_cfd, f_cfd,
        label='CFD',
        marker='o',
        color=colors[0],
        linewidth=2
    )

    # ---- LLM curves ----
    for i, path in enumerate(llm_paths):
        data_LLM = np.load(path)['data']
        r = data_LLM['r']
        r_steps = r[1] - r[0]
        area = r_steps
        f_tan_LLM = data_LLM['f_tan'] / area
        n = 50 - 1

        label = f'LL model'
        color = colors[(i + 1) % len(colors)]

        # Plot LLM on primary axis
        ax1.plot(
            r[:n], f_tan_LLM[:n],
            label=label,
            marker='x',
            linestyle='--',
            color=color
        )

        # Interpolate CFD to LLM radial positions for error calculation
        f_cfd_interp = np.interp(r[:n], r_cfd, f_cfd)
        
        # Calculate absolute relative error: |CFD - LLM| / |CFD| * 100%
        rel_error = np.abs(f_cfd_interp - f_tan_LLM[:n]) / np.abs(f_cfd_interp) * 100
        
        # Plot error on secondary axis
        ax2.plot(
            r[:n], rel_error,
            label=f'Error ({label})',
            marker='s',
            linestyle=':',
            color=color,
            alpha=0.7
        )

    # Primary axis formatting
    ax1.set_xlabel('Radius [m]')
    ax1.set_ylabel(r'$F_{tan}$ [N/m]', color='black')
    ax1.tick_params(axis='y', labelcolor='black')
    ax1.grid(True, linestyle='--', linewidth=0.5)
    ax1.legend(loc='upper left')

    # Secondary axis formatting
    ax2.set_ylabel('Absolute Relative Error [%]')
    ax2.tick_params(axis='y')
    ax2.legend(loc='upper right')

    fig.tight_layout()
    fig.savefig('./DesignSpace/test/tan_force_comparison.png', dpi=300)


if __name__ == '__main__':
    llm_paths = [
        './DesignSpace/test/_res.npz',
        './DesignSpace/different_t/_res.npz',

    ]

    plot_axial_force(llm_paths)
    plot_tan_force(llm_paths)