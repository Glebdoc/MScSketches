import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import Normalize
from scipy.stats import gaussian_kde


def filter_non_converged(df: pd.DataFrame, MTOW) -> pd.DataFrame:
    """Filter out non-converged runs based on stall and thrust deviation criteria."""
    thrust_tol = 0.03 * MTOW
    moment_tol = 0.15
    try:
        mask = (
            (df["STALL_HIGH"] <= 0.05)
            & (df["STALL_LOW"] <= 0.05)
            & (abs(df["thrust"] - MTOW) <= thrust_tol)
        )
    except:
        mask = (
            (df["STALL_MAIN_HIGH"] <= 0.05)
            & (df["STALL_MAIN_LOW"] <= 0.05)
            & (abs(df["thrust_main"] - MTOW) <= thrust_tol)
            & (abs(df["torque_main"] - df["moment_created"])/(df['torque_main']) <= moment_tol)
        )
    return df[mask]

def plot_parallel_coordinates(df, dims, MTOW):

    df.rename(columns=lambda c: c.strip(), inplace=True)

    df = filter_non_converged(df, MTOW)

    # --- Use power_loading instead of p_total ---
    # Ensure numeric types
    for c in dims:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=dims)

    # --- Color setup based on Power Loading (PL) ---
    base_cmap = cm.Greys
    cmap = cm.ScalarMappable(norm=None, cmap=cm.colors.LinearSegmentedColormap.from_list(
    'limited_greys', base_cmap(np.linspace(0.2, 0.9, 256))
    )).cmap
    norm = Normalize(vmin=df["power_loading"].min(), vmax=df["power_loading"].max())

    # --- Prepare figure ---
    n_dims = len(dims)
    x = np.arange(n_dims)
    fig, ax = plt.subplots(figsize=(10, 6))

    # --- Value ranges for each axis ---
    vmin = {c: df[c].min() for c in dims}
    vmax = {c: df[c].max() for c in dims}

    def scale_value(col, values):
        lo, hi = vmin[col], vmax[col]
        return (values - lo) / (hi - lo + 1e-12)

    # --- Plot each sample line ---
    for _, row in df.iterrows():
        y_scaled = [scale_value(c, row[c]) for c in dims]
        color = cmap(norm(row["power_loading"]))
        # variable line thickness by PL (inverse if higher PL = better)
        # lw = 1 + 3 * (row["power_loading"] - df["power_loading"].min()) / (
        #     df["power_loading"].max() - df["power_loading"].min()
        # )
        lw =3
        ax.plot(x, y_scaled, color=color, alpha=0.6, linewidth=lw)

    # --- Draw vertical axes and tick labels ---
    for i, c in enumerate(dims):
        ax.vlines(i, 0, 1, color='k', linewidth=1)
        ticks = np.linspace(vmin[c], vmax[c], 2)
        tick_pos = (ticks - vmin[c]) / (vmax[c] - vmin[c] + 1e-12)
        ax.set_xticks(x)
        ax.set_xticklabels(dims, fontsize=12)
        for tp, tv in zip(tick_pos, ticks):
            ax.text(i - 0.05, tp + 0.02, f"{tv:.2f}", va='center', ha='right', fontsize=12)

    # --- Colorbar (Power Loading) ---
    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, pad=0.02)
    cbar.set_label("Power Loading (PL)", rotation=270, labelpad=15)

    # --- Cleanup and styling ---
    #ax.set_xlim(-0.5, n_dims - 0.5)
    ax.set_xlim(0, n_dims -1 )
    ax.set_ylim(0, 1)
    ax.grid(True, axis='x', linestyle=':')

    # Remove the 0–1 left axis
    ax.set_yticks([])
    ax.spines['left'].set_visible(False)

    plt.tight_layout()
    plt.savefig('./Factorial_trial/factorial_data/helicopter_factorial_parallel_coordinates.png', dpi=300)
    #plt.savefig('./Factorial_trial/factorial_data/quadcopter_factorial_parallel_coordinates.png', dpi=300)
    print('Saved in')
def plot_DL_PL(MTOW):
    """
    Plot Disk Loading vs Power Loading for all performance summary CSVs in the folder,
    highlight and print the full row of the best (highest power-loading) design
    for each configuration.
    """
    csv_files = [f for f in os.listdir('.') if f.startswith('performance_summary') and f.endswith('.csv')]
    if not csv_files:
        print("No performance summary CSV files found.")
        return

    fig, ax = plt.subplots(figsize=(10, 6))
    total_dl, total_pl = [], []

    for csv_file in csv_files:
        df = pd.read_csv(csv_file)
        df.rename(columns=lambda c: c.strip(), inplace=True)
        df = filter_non_converged(df, MTOW=MTOW)
        df = df.sort_values(by='power_loading', ascending=False)

        x = df['disk_loading'].values
        y = df['power_loading'].values

        # Find best design (max power loading)
        best_idx = df['power_loading'].idxmax()
        best_row = df.loc[best_idx]  # ✅ get full DataFrame row

        config_name = csv_file[:-4].replace('performance_summary_', '')

        # --- plot points ---
        sc = ax.scatter(x, y, alpha=0.6, label=config_name, s=50)
        color = sc.get_facecolors()[0]
        ax.scatter(best_row['disk_loading'], best_row['power_loading'],
                   s=200, alpha=1.0, color=color,
                   edgecolors='black', linewidths=2, marker='*', zorder=10)

        total_dl.extend(x)
        total_pl.extend(y)

        # --- print full row in terminal ---
        print(f"\n=== Best design for {config_name} ===")
        print(best_row.to_string(index=True))
        print('-' * 60)

    # --- Fit a 2nd-degree polynomial (parabola) ---
    total_dl = np.array(total_dl).flatten()
    total_pl = np.array(total_pl).flatten()

    coeffs = np.polyfit(total_dl, total_pl, 2)
    poly_func = np.poly1d(coeffs)
    dl_smooth = np.linspace(total_dl.min(), total_dl.max(), 200)
    pl_fit = poly_func(dl_smooth)
    ax.plot(dl_smooth, pl_fit, 'r--', linewidth=1.5, label='Parabolic fit', zorder=5)

    ax.set_xlabel('Disk Loading (N/m²)', fontsize=14)
    ax.set_ylabel('Power Loading (N/W)', fontsize=14)
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend(title="Configuration", loc='best')
    plt.tight_layout()
    plt.savefig('DL_vs_PL_design_space.png', dpi=300)
    plt.show()

plot_DL_PL(MTOW=60)


# helicopter
# df = pd.read_csv('./Factorial_trial/factorial_data/performance_summary_helicopter.csv')
# dims = ["disk_loading","sigma", "taper", "power_loading"]
# plot_parallel_coordinates(df, dims, MTOW=60)

# quadcopter
# df = pd.read_csv('./Factorial_trial/factorial_data/performance_summary_quadcopter.csv')
# dims = ["R", "NB", "sigma", "theta", "diagonal", "torque", "power_loading"]
# plot_parallel_coordinates(df, dims, MTOW=60)

# plot DL vs PL
