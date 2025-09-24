#!/usr/bin/env python3
# plot_fourbar_report.py
import argparse
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def plot_line(x, y, xlabel, ylabel, title, outfile, show=False):
    fig, ax = plt.subplots()
    ax.plot(x, y)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(outfile, dpi=150)
    if show:
        plt.show()
    plt.close(fig)

def main():
    ap = argparse.ArgumentParser(description="Plot forces, torques, and MA from fourbar_report.csv")
    ap.add_argument("--csv", type=str, default="fourbar_report.csv",
                    help="Path to CSV exported by sample_mechanics")
    ap.add_argument("--outdir", type=str, default="plots", help="Output directory for PNGs")
    ap.add_argument("--show", action="store_true", help="Show plots interactively")
    args = ap.parse_args()

    if not os.path.isfile(args.csv):
        raise FileNotFoundError(f"CSV not found: {args.csv}")

    os.makedirs(args.outdir, exist_ok=True)

    df = pd.read_csv(args.csv)

    # Basic angles in degrees for readability
    theta_deg = np.degrees(df["theta2_rad"].values)

    # Plot: Mechanical Advantage
    plot_line(theta_deg, df["MA"].values,
              xlabel="theta2 (deg)", ylabel="Mechanical Advantage (|tau4/tau2|)",
              title="Mechanical Advantage vs Input Angle",
              outfile=os.path.join(args.outdir, "MA_vs_theta2.png"),
              show=args.show)

    # Plot: Output torque tau4
    plot_line(theta_deg, df["tau4_Nm"].values,
              xlabel="theta2 (deg)", ylabel="tau4 (N·m)",
              title="Output Torque at D vs Input Angle",
              outfile=os.path.join(args.outdir, "tau4_vs_theta2.png"),
              show=args.show)

    # Plot: Angular velocity ratio w4/w2
    plot_line(theta_deg, df["w4_over_w2"].values,
              xlabel="theta2 (deg)", ylabel="w4 / w2 (-)",
              title="Angular Velocity Ratio vs Input Angle",
              outfile=os.path.join(args.outdir, "w4_over_w2_vs_theta2.png"),
              show=args.show)

    # Reaction magnitudes at pins
    for pin in ["A","B","C","D"]:
        Fx = df[f"{pin}x"].values
        Fy = df[f"{pin}y"].values
        Fmag = np.hypot(Fx, Fy)
        plot_line(theta_deg, Fmag,
                  xlabel="theta2 (deg)", ylabel=f"|F_{pin}| (N)",
                  title=f"Reaction Magnitude at {pin} vs Input Angle",
                  outfile=os.path.join(args.outdir, f"F{pin}_mag_vs_theta2.png"),
                  show=args.show)

        # Also plot components per pin (separate files, one chart each)
        plot_line(theta_deg, Fx,
                  xlabel="theta2 (deg)", ylabel=f"F{pin}x (N)",
                  title=f"{pin} Reaction X-Component vs Input Angle",
                  outfile=os.path.join(args.outdir, f"F{pin}x_vs_theta2.png"),
                  show=args.show)

        plot_line(theta_deg, Fy,
                  xlabel="theta2 (deg)", ylabel=f"F{pin}y (N)",
                  title=f"{pin} Reaction Y-Component vs Input Angle",
                  outfile=os.path.join(args.outdir, f"F{pin}y_vs_theta2.png"),
                  show=args.show)

    print(f"Saved plots to: {os.path.abspath(args.outdir)}")

if __name__ == "__main__":
    main()
