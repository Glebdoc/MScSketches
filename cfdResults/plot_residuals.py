#!/usr/bin/env python3
import sys
import os
import math
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

def set_bw_design():
    """
    Configure matplotlib + seaborn for consistent black & white (grayscale) plotting style.
    """
    # Seaborn & matplotlib styles
    sns.set_style("whitegrid")
    plt.style.use("seaborn-v0_8")
    sns.set_palette("Greys_r")  # grayscale palette
    
    # Global rcParams for consistent looks
    plt.rcParams.update({
        "figure.figsize": (6, 4),
        "figure.dpi": 200,
        "axes.edgecolor": "black",
        "axes.labelweight": "bold",
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.color": "black",
        "grid.linewidth": 0.5,
        "legend.frameon": True,
        "legend.fancybox": False,
        "legend.shadow": False,  # cleaner for B&W
        "lines.linewidth": 1.5,
        "lines.markersize": 3,
    })
    
    # Line styles / markers / colors for B&W distinction
    design = {
        "line_styles": ['-', '--', '-.'],
        "markers": ['o', 's', '^'],
        "colors": ['black', '0.4', '0.7']  # black, dark gray, light gray
    }
    return design
def read_pairs(path):
    """Read a 2-column file (iteration, residual). Ignore non-numeric lines."""
    iters, resids = [], []
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for ln in f:
            ln = ln.strip().replace(',', ' ')  # tolerate commas
            if not ln:
                continue
            parts = ln.split()
            if len(parts) < 2:
                continue
            try:
                i = float(parts[0])
                r = float(parts[1])
            except ValueError:
                continue
            iters.append(i)
            resids.append(r)
    if not iters:
        raise ValueError(f"No numeric (iter,residual) pairs found in {path}")
    return iters, resids

def split_series(iters, resids):
    """
    Split stacked data into consecutive series whenever iteration 'resets'
    (i.e., next iteration < previous iteration). Returns list of (it,res) lists.
    """
    series = []
    cur_i, cur_r = [iters[0]], [resids[0]]
    for i, r in zip(iters[1:], resids[1:]):
        if i < cur_i[-1]:  # iteration reset → new series
            series.append((cur_i, cur_r))
            cur_i, cur_r = [i], [r]
        else:
            cur_i.append(i); cur_r.append(r)
    series.append((cur_i, cur_r))
    return series

def merge_by_label(series, labels):
    """
    Assign labels cyclically to series (continuity, Ux, Uy, Uz, k, omega),
    and merge multiple chunks of the same label (in case of multiple solver runs).
    """
    by_label = {lab: {"iter": [], "res": []} for lab in labels}
    for idx, (it, rs) in enumerate(series):
        lab = labels[idx % len(labels)]
        # If this label already has data, append with a gap separator (NaN) to break lines
        if by_label[lab]["iter"]:
            by_label[lab]["iter"].append(float('nan'))
            by_label[lab]["res"].append(float('nan'))
        by_label[lab]["iter"].extend(it)
        by_label[lab]["res"].extend(rs)
    return by_label

def main():
    # Usage: python plot_residuals.py residuals_3k
    path = sys.argv[1] if len(sys.argv) > 1 else "cfdResults/helicopter/output/my_deleted"
    if not os.path.isfile(path):
        sys.exit(f"File not found: {path}")

    it, rs = read_pairs(path)
    series = split_series(it, rs)

    labels = ["continuity", "Ux", "Uy", "Uz", "k", "omega"]
    alphas = [1.0, 1.0, 0.6, 0.4, 0.8, 0.8]
    linestyles = ['--', '-', '-', '-', ':', '-.']
    data = merge_by_label(series, labels)

    plt.figure(figsize=(8,5))
    design = set_bw_design()
    for lab in labels:
        x = data[lab]["iter"]
        y = data[lab]["res"]
        if not x:
            continue
        # semilogy; skip non-positive residuals
        x_plot = []
        y_plot = []
        for xi, yi in zip(x, y):
            if (isinstance(xi, float) and math.isnan(xi)) or (isinstance(yi, float) and math.isnan(yi)):
                # break line on NaN separators
                if x_plot:
                    plt.semilogy(x_plot[:-2], y_plot[:-2], label=lab if lab not in plt.gca().get_legend_handles_labels()[1] else None, color='black',  linestyle=linestyles[labels.index(lab)], alpha=alphas[labels.index(lab)])
                    x_plot, y_plot = [], []
                continue
            if yi > 0:
                x_plot.append(xi); y_plot.append(yi)
        if x_plot:
            plt.semilogy(x_plot[:-2], y_plot[:-2], label=lab if lab not in plt.gca().get_legend_handles_labels()[1] else None, color='black',  linestyle=linestyles[labels.index(lab)], alpha=alphas[labels.index(lab)])


    plt.xlabel("Iteration")
    plt.ylabel("Residual")
    #plt.title("CFD Residuals by Field")
    plt.grid(True, which="both", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    out_png = os.path.splitext(os.path.basename(path))[0] + "_residuals.png"
    plt.savefig(out_png, dpi=200)
    print(f"Saved plot to {out_png}")
    plt.show()

def plot_force_history():
    axial = np.genfromtxt("cfdResults/helicopter/output/axial-rfile_2_1.out", skip_header=3)
    tangential = np.genfromtxt("cfdResults/helicopter/output/drag-rfile_2_1.out", skip_header=3)
    #axial_small = np.genfromtxt("results/thrust_small-rfile_1_1.out", skip_header=3)
    plt.figure(figsize=(8,5))
    design = set_bw_design()
    # plt.plot(axial[:,0], axial[:,1], label="Axial Force", color='black', linestyle='-')
    # plt.semilogy(tangential[:,0], -tangential[:,1], label="Tangential Force", color='black', linestyle='--')
    plt.semilogy(axial[:-1,0], abs(axial[1:,1]-axial[:-1, 1]),  '--', label=r'error $F_{axial}$', color='black' )
    plt.semilogy(tangential[:-1,0], abs(tangential[1:,1]-tangential[:-1, 1]),  '-', label=r'error $F_{tan}$', color='black', alpha=0.7)
    #plt.semilogy(axial_small[:-1,0], abs(axial_small[1:,1]-axial_small[:-1, 1]),  ':', label=r'error $F_{axial, small}$', color='black', alpha=0.4)
    plt.xlabel("Iteration")
    plt.ylabel("Error in Force (N)")
    plt.legend()
    plt.show()
if __name__ == "__main__":
    #plot_force_history()
    main()

