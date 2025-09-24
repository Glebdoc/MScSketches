#!/usr/bin/env python3
# fourbar_anim.py
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import argparse
from dataclasses import dataclass
import json, numpy as np
import csv

@dataclass
class FourBar:
    A: np.ndarray         # Fixed pivot A = [xA, yA]
    D: np.ndarray         # Fixed pivot D = [xD, yD]
    r2: float             # AB
    r3: float             # BC
    r4: float             # CD
    config: str = "open"  # "open" or "crossed"

    def __post_init__(self):
        self.A = np.array(self.A, dtype=float)
        self.D = np.array(self.D, dtype=float)
        self.g = self.D - self.A             # ground vector AD
        self.r1 = np.linalg.norm(self.g)     # ground length
        if self.r1 == 0:
            raise ValueError("A and D cannot coincide.")
        self.g_hat = self.g / self.r1

    def solve_positions(self, theta2):
        """
        Given input angle theta2 of link AB (angle from +x),
        compute positions B, C and angles theta3, theta4 by circle intersection.
        Returns (ok, B, C, theta3, theta4)
        """
        # Point B from A with angle theta2
        B = self.A + self.r2 * np.array([np.cos(theta2), np.sin(theta2)])

        # Intersection of circles: center B radius r3, center D radius r4
        C_candidates = circle_intersections(B, self.r3, self.D, self.r4)
        if C_candidates is None:
            return False, None, None, None, None

        C1, C2 = C_candidates
        # Choose configuration
        # "open": C above the line BD (left-hand side when walking B->D),
        # "crossed": C below. If BD is near-zero length, fall back to y-position.
        BD = self.D - B
        if np.linalg.norm(BD) < 1e-12:
            # Degenerate; choose by y
            C = C1 if (C1[1] >= C2[1]) == (self.config == "open") else C2
        else:
            # Sign of z-component of cross product (2D)
            def side(P):
                v = P - B
                return np.cross(np.append(BD,0), np.append(v,0))[2]  # z
            s1, s2 = side(C1), side(C2)
            if self.config == "open":
                C = C1 if s1 >= s2 else C2
            else:
                C = C1 if s1 <= s2 else C2

        # Angles of links BC and DC
        theta3 = np.arctan2(C[1]-B[1], C[0]-B[0])
        theta4 = np.arctan2(C[1]-self.D[1], C[0]-self.D[0])
        return True, B, C, theta3, theta4

    # def feasible(self, theta2):
    #     """
    #     Quick feasibility test: distance DB must satisfy |r4 - r3| <= |DB| <= r3 + r4
    #     """
    #     B = self.A + self.r2 * np.array([np.cos(theta2), np.sin(theta2)])
    #     d = np.linalg.norm(self.D - B)
    #     return (abs(self.r4 - self.r3) - 1e-12) <= d <= (self.r3 + self.r4 + 1e-12)

    def feasible(self, theta2, eps=1e-9):
        B = self.A + self.r2 * np.array([np.cos(theta2), np.sin(theta2)])
        d = np.linalg.norm(self.D - B)
        return (abs(self.r4 - self.r3) - eps) <= d <= (self.r3 + self.r4 + eps)

def circle_intersections(c0, r0, c1, r1, eps=1e-9):
    c0 = np.array(c0, dtype=float)
    c1 = np.array(c1, dtype=float)
    d = np.linalg.norm(c1 - c0)
    if d > r0 + r1 + eps:           # separate
        return None
    if d < abs(r0 - r1) - eps:      # contained
        return None
    if d < eps and abs(r0 - r1) < eps:
        # Nearly coincident circles: pick an arbitrary chord direction to return a pair
        # to keep the solver going (rare, but avoids hard None).
        # Choose a unit perpendicular vector:
        u = np.array([1.0, 0.0])
        i1 = c0 + r0 * u
        i2 = c0 - r0 * u
        return i1, i2

    # standard two-point intersection
    a = (r0**2 - r1**2 + d**2) / (2*d) if d > eps else 0.0
    h_sq = max(r0**2 - a**2, 0.0)
    h = np.sqrt(h_sq)

    p2 = c0 + a * (c1 - c0) / (d if d > eps else 1.0)
    perp = np.array([-(c1 - c0)[1], (c1 - c0)[0]]) / (d if d > eps else 1.0)

    i1 = p2 + h * perp
    i2 = p2 - h * perp
    return i1, i2

def velocity_ratio(mech, theta2, h=1e-6):
    """
    Numerically estimate instantaneous angular velocity ratio w4/w2 at angle theta2,
    using small perturbation h and solving positions twice.
    Returns (ok, w4_over_w2). w2 is taken as +1 rad/s reference.
    """
    ok0, B0, C0, th3_0, th4_0 = mech.solve_positions(theta2)
    ok1, B1, C1, th3_1, th4_1 = mech.solve_positions(theta2 + h)
    if not (ok0 and ok1):
        return False, None
    # Unwrap to avoid 2π jumps
    dth4 = np.unwrap([th4_0, th4_1])[1] - np.unwrap([th4_0, th4_1])[0]
    w4_over_w2 = dth4 / h  # since w2 = dtheta2/dt = 1
    return True, w4_over_w2

def torque_transmission(tau2, w4_over_w2):
    """
    Power balance (lossless quasi-static): tau2*w2 + tau4*w4 = 0, with w2=1.
    => tau4 = -tau2 / (w4/w2) = -tau2 / w4_over_w2
    Mechanical advantage (torque) MA = |tau4/tau2| = |1/(w4/w2)|.
    """
    if abs(w4_over_w2) < 1e-12:
        return np.inf, np.sign(-tau2)*np.inf  # lock-up (toggle) condition
    tau4 = -tau2 / w4_over_w2
    MA = abs(tau4 / tau2)
    return MA, tau4

def solve_joint_forces(mech, theta2, tau2, tau4):
    """
    Solve pin reactions at A,B,C,D for static equilibrium under torques tau2 at A and tau4 at D.
    No other external forces. Returns dict with Ax,Ay,Bx,By,Cx,Cy,Dx,Dy.
    Method: write equilibrium for each rigid link (AB, BC, CD, ground AD) and
    solve the linear system. Pins transmit equal/opposite forces between links.
    """
    ok, B, C, th3, th4 = mech.solve_positions(theta2)
    if not ok:
        return None

    # Unknowns vector U = [Ax, Ay, Bx, By, Cx, Cy, Dx, Dy]^T
    # We’ll write 3 equations per link: ΣFx=0, ΣFy=0, ΣMz=0 about a convenient point
    # Link AB: forces at A (Ax,Ay) and B (−Bx,−By) act; torque tau2 acts at A.
    # Link BC: forces at B (Bx,By) and C (−Cx,−Cy).
    # Link CD: forces at C (Cx,Cy) and D (−Dx,−Dy); torque tau4 at D.
    # Ground AD: forces at A (−Ax,−Ay) and D (Dx,Dy), and external torques −tau2 at A, −tau4 at D (equal/opposite on ground). Its equations are redundant with others; we can skip ground or use as a check.
    # Using AB, BC, CD (9 eqns) is enough to solve 8 unknowns in least-squares; we’ll use ΣFx, ΣFy for each and one moment eqn per link to total 6+3 = 9 equations -> solve LS.

    A = mech.A; D = mech.D

    def moment2D(rp, force_point, Fx, Fy):
        # moment of force (Fx,Fy) applied at force_point about rp (z out of plane)
        r = force_point - rp
        return r[0]*Fy - r[1]*Fx

    rows = []
    rhs = []

    # Helper to append an equation row mapping to U=(Ax,Ay,Bx,By,Cx,Cy,Dx,Dy)
    def add_eq(coeffs, b):
        rows.append(coeffs); rhs.append(b)

    # --- Link AB ---
    # ΣFx = 0: Ax - Bx = 0
    add_eq([1,0, -1,0, 0,0, 0,0], 0.0)
    # ΣFy = 0: Ay - By = 0
    add_eq([0,1, 0,-1, 0,0, 0,0], 0.0)
    # ΣMz about A: M(A) = tau2 + M_B = 0  => tau2 + r_AB x F_B(on AB) = 0
    # Force at B on AB is (-Bx,-By) applied at B
    mB = lambda Ax,Ay,Bx,By,Cx,Cy,Dx,Dy: moment2D(A, B, -Bx, -By)
    # The row coefficients for moment wrt unknowns: only Bx,By appear
    coeff = [0,0, 0,0, 0,0, 0,0]
    # coefficient of Bx in moment: ∂/∂Bx moment2D(A,B,-Bx,-By) = -∂/∂Bx [ (Bx)*(B_y-A_y) - (By)*(B_x-A_x) ]?
    # Simpler: write explicitly: M = r_x * (-By) - r_y * (-Bx) = -r_x*By + r_y*Bx
    r = B - A; rx, ry = r[0], r[1]
    # So coefficients: Bx -> +ry ; By -> -rx
    coeff[2] = +ry  # Bx
    coeff[3] = -rx  # By
    add_eq(coeff, -tau2)

    # --- Link BC ---
    # ΣFx = 0:  Bx - Cx = 0
    add_eq([0,0, 1,0, -1,0, 0,0], 0.0)
    # ΣFy = 0:  By - Cy = 0
    add_eq([0,0, 0,1, 0,-1, 0,0], 0.0)
    # ΣMz about B: M(B) = r_BC x F_C(on BC) + r_BB x F_B(on BC)=0; torque=0
    # On BC the forces are: at B → (-Bx,-By); at C → (Cx,Cy)
    # About B, the B force passes through pivot so no moment; only C contributes.
    r = C - B; rx, ry = r[0], r[1]
    coeff = [0,0, 0,0, 0,0, 0,0]
    # Moment of (Cx,Cy) at point C about B: M = r_x*Cy - r_y*Cx
    # So coeffs: Cx -> -ry ; Cy -> +rx
    coeff[4] = -ry  # Cx
    coeff[5] = +rx  # Cy
    add_eq(coeff, 0.0)

    # --- Link CD ---
    # ΣFx = 0:  Cx - Dx = 0
    add_eq([0,0, 0,0, 1,0, -1,0], 0.0)
    # ΣFy = 0:  Cy - Dy = 0
    add_eq([0,0, 0,0, 0,1, 0,-1], 0.0)
    # ΣMz about D: tau4 + r_DC x F_C(on CD) = 0
    # On CD, forces are: at C → (-Cx,-Cy) acting on CD; at D → (Dx,Dy) at pivot (no moment about D).
    r = C - D; rx, ry = r[0], r[1]
    coeff = [0,0, 0,0, 0,0, 0,0]
    # Moment of (-Cx,-Cy) applied at C about D: M = r_x*(-Cy) - r_y*(-Cx) = -rx*Cy + ry*Cx
    # Coeffs: Cx -> +ry ; Cy -> -rx
    coeff[4] = +ry  # Cx
    coeff[5] = -rx  # Cy
    add_eq(coeff, -tau4)

    # Solve least squares
    M = np.array(rows, dtype=float)
    b = np.array(rhs, dtype=float)
    U, *_ = np.linalg.lstsq(M, b, rcond=None)
    Ax, Ay, Bx, By, Cx, Cy, Dx, Dy = U
    return {
        "Ax": Ax, "Ay": Ay,
        "Bx": Bx, "By": By,
        "Cx": Cx, "Cy": Cy,
        "Dx": Dx, "Dy": Dy
    }

def sample_mechanics(mech, thetas, tau2=1.0, csv_path="fourbar_report.csv"):
    """
    For a sequence of theta2, compute (w4/w2), MA, tau4, and joint reactions.
    Save CSV and return dict of arrays.
    """
    out = {
        "theta2": [], "w4_over_w2": [], "MA": [], "tau4": [],
        "Ax": [], "Ay": [], "Bx": [], "By": [], "Cx": [], "Cy": [], "Dx": [], "Dy": []
    }
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["theta2_rad","w4_over_w2","MA","tau4_Nm","Ax","Ay","Bx","By","Cx","Cy","Dx","Dy"])
        for th in thetas:
            ok, ratio = velocity_ratio(mech, th)
            if not ok:
                continue
            MA, tau4 = torque_transmission(tau2, ratio)
            forces = solve_joint_forces(mech, th, tau2, tau4)
            if forces is None:
                continue
            row = [th, ratio, MA, tau4,
                   forces["Ax"],forces["Ay"],forces["Bx"],forces["By"],
                   forces["Cx"],forces["Cy"],forces["Dx"],forces["Dy"]]
            w.writerow(row)
            for k,v in zip(out.keys(), row):
                out[k].append(v)
    # Convert lists to numpy arrays
    for k in out:
        out[k] = np.array(out[k], dtype=float)
    return out


def find_motion_interval(mech, n=2000):
    """
    Scan theta2 in [0, 2π) for feasible configurations, return the largest continuous
    interval [theta_start, theta_end] with feasibility. Handles wrap-around.
    """
    thetas = np.linspace(0, 2*np.pi, n, endpoint=False)
    feas = np.array([mech.feasible(t) for t in thetas])
    if not np.any(feas):
        return None

    # Identify contiguous feasible segments
    segments = []
    start = None
    for i, ok in enumerate(feas):
        if ok and start is None:
            start = i
        if not ok and start is not None:
            segments.append((start, i-1))
            start = None
    if start is not None:
        segments.append((start, len(feas)-1))

    # Handle wrap segment (if both first and last indices are feasible)
    if feas[0] and feas[-1]:
        # Merge first and last segments
        first = segments[0]
        last = segments[-1]
        merged = (last[0] - len(feas), first[1])  # indices modulo n
        segments = [merged] + segments[1:-1]

    # Choose segment with maximum length
    def seg_length(seg):
        i0, i1 = seg
        if i1 >= i0:
            L = i1 - i0 + 1
        else:
            L = (i1 + len(feas)) - i0 + 1
        return L
    best = max(segments, key=seg_length)

    # Convert to angles
    i0, i1 = best
    if i1 >= i0:
        theta_seq = thetas[i0:i1+1]
    else:
        theta_seq = np.concatenate([thetas[i0:], thetas[:i1+1]])

    return theta_seq



def animate_fourbar(
    A=(0.0, 0.0),
    D=(1.0, 0.0),
    r2=0.6,
    r3=0.8,
    r4=0.7,
    config="open",
    start_angle=None,
    frames=300,
    save_gif="fourbar.gif",
    show=True,
    input_torque=1.0,
    report_csv="fourbar_report.csv",
    theta_limits=None,
    direction=+1   # <--- NEW (tuple in degrees, e.g. (30,120))
):
    mech = FourBar(A, D, r2, r3, r4, config=config)

    theta_seq = find_motion_interval(mech, n=3000)
    if theta_seq is None or len(theta_seq) < 2:
        raise RuntimeError("No feasible motion interval found for the given geometry.")
    
    if theta_limits is not None:
        th_start_deg, th_end_deg = theta_limits
        th_start = np.deg2rad(th_start_deg)
        th_end   = np.deg2rad(th_end_deg)

        # Normalize angles to [0, 2π)
        theta_seq = (theta_seq + 2*np.pi) % (2*np.pi)
        th_start = (th_start + 2*np.pi) % (2*np.pi)
        th_end   = (th_end + 2*np.pi) % (2*np.pi)

        # Ensure start < end (no wrap-around for now)
        if th_end <= th_start:
            raise ValueError("theta_limits must have end > start (no wrap-around allowed).")

        # Mask to keep only values in [start,end]
        mask = (theta_seq >= th_start) & (theta_seq <= th_end)
        theta_seq = theta_seq[mask]

        if len(theta_seq) < 2:
            raise RuntimeError(f"No feasible theta2 values in the requested range {theta_limits}.")

    # If user provided start_angle, rotate the sequence to start near it.
    if start_angle is not None:
        start_angle = (start_angle + 2*np.pi) % (2*np.pi)
        idx = np.argmin(np.abs(((theta_seq - start_angle + np.pi) % (2*np.pi)) - np.pi))
        theta_seq = np.concatenate([theta_seq[idx:], theta_seq[:idx]])

    # Downsample or interpolate theta sequence to the requested frame count, and ensure we go
    # Downsample or interpolate theta sequence to the requested frame count
    theta_lin = np.linspace(0, 1, len(theta_seq))
    # sweep forward or backward
    if direction < 0:
        t_target = np.linspace(1, 0, frames)   # backwards
    else:
        t_target = np.linspace(0, 1, frames)   # forwards
    thetas = np.interp(t_target, theta_lin, theta_seq)

    mech = FourBar(A, D, r2, r3, r4, config=config)  # if not already constructed above
    report = sample_mechanics(mech, thetas, tau2=input_torque, csv_path=report_csv)

    # Precompute positions for axis limits
    pts = []
    for th in thetas:
        ok, B, C, _, _ = mech.solve_positions(th)
        if not ok:
            continue
        pts.extend([B, C])
    pts = np.array(pts)
    x_all = np.concatenate([[mech.A[0], mech.D[0]], pts[:,0]] if len(pts) else [[mech.A[0], mech.D[0]]])
    y_all = np.concatenate([[mech.A[1], mech.D[1]], pts[:,1]] if len(pts) else [[mech.A[1], mech.D[1]]])

    pad = 0.1 * max(1.0, np.max([np.ptp(x_all), np.ptp(y_all)]))
    xmin, xmax = np.min(x_all)-pad, np.max(x_all)+pad
    ymin, ymax = np.min(y_all)-pad, np.max(y_all)+pad

    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_title("4-Bar Linkage")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True, alpha=0.3)

    # Artists
    ground_line, = ax.plot([], [], lw=3)
    link2_line,  = ax.plot([], [], lw=3)
    link3_line,  = ax.plot([], [], lw=3)
    link4_line,  = ax.plot([], [], lw=3)
    joints_scatter = ax.scatter([], [], s=30, zorder=5)

    ground_line.set_data([mech.A[0], mech.D[0]], [mech.A[1], mech.D[1]])

    def init():
        link2_line.set_data([], [])
        link3_line.set_data([], [])
        link4_line.set_data([], [])
        joints_scatter.set_offsets(np.empty((0, 2)))
        return link2_line, link3_line, link4_line, joints_scatter

    def update(i):
        th = thetas[i]
        ok, B, C, _, _ = mech.solve_positions(th)
        if not ok:
            return link2_line, link3_line, link4_line, joints_scatter
        
        try:
            # find closest index (in case of any skips)
            j = i if i < len(report["MA"]) else -1
            ax.set_title(f"4-Bar Linkage  |  MA≈{report['MA'][j]:.3g}")
        except Exception:
            pass

        # AB
        link2_line.set_data([mech.A[0], B[0]], [mech.A[1], B[1]])
        # BC
        link3_line.set_data([B[0], C[0]], [B[1], C[1]])
        # CD
        link4_line.set_data([C[0], mech.D[0]], [C[1], mech.D[1]])

        joints_scatter.set_offsets(np.vstack([mech.A, B, C, mech.D]))
        return link2_line, link3_line, link4_line, joints_scatter

    anim = FuncAnimation(fig, update, init_func=init, frames=len(thetas), interval=1000/60, blit=True, repeat=False)

    # Save GIF
    if save_gif:
        try:
            anim.save(save_gif, writer=PillowWriter(fps=60))
            print(f"Saved animation to {save_gif}")
        except Exception as e:
            print(f"Could not save GIF: {e}")

    if show:
        plt.show()
    plt.close(fig)

def parse_args():
    p = argparse.ArgumentParser(description="Simulate a 4-bar linkage and animate motion from limit to limit.")
    p.add_argument("--Ax", type=float, default=0.0, help="x of fixed pivot A")
    p.add_argument("--Ay", type=float, default=0.0, help="y of fixed pivot A")
    p.add_argument("--Dx", type=float, default=1.0, help="x of fixed pivot D")
    p.add_argument("--Dy", type=float, default=0.0, help="y of fixed pivot D")
    p.add_argument("--r2", type=float, default=0.6, help="Length AB")
    p.add_argument("--r3", type=float, default=0.8, help="Length BC")
    p.add_argument("--r4", type=float, default=0.7, help="Length CD")
    p.add_argument("--config", choices=["open", "crossed"], default="open", help="Assembly configuration")
    p.add_argument("--start_angle_deg", type=float, default=None, help="Optional start angle for AB in degrees")
    p.add_argument("--frames", type=int, default=300, help="Number of animation frames")
    p.add_argument("--gif", type=str, default="fourbar.gif", help="Output GIF filename (set empty to skip saving)")
    p.add_argument("--no_show", action="store_true", help="Do not display the animation window")
    return p.parse_args()

if __name__ == "__main__":
    # read params.json
    with open("params.json", "r") as f:
        params = json.load(f)

    A = tuple(params.get("A", [0.0, 0.0]))
    D = tuple(params.get("D", [1.0, 0.0]))
    r2 = params.get("r2", 0.6)
    r3 = params.get("r3", 0.8)
    r4 = params.get("r4", 0.7)
    config = params.get("config", "open")
    start_angle_deg = params.get("start_angle_deg")
    frames = params.get("frames", 300)
    gif = params.get("gif", "fourbar.gif")
    show = params.get("show", True)

    start_angle = None if start_angle_deg is None else np.deg2rad(start_angle_deg)

    animate_fourbar(
    A=A, D=D, r2=r2, r3=r3, r4=r4,
    config=config,
    start_angle=start_angle,
    frames=frames,
    save_gif=gif,
    show=show,
    input_torque=params.get("input_torque", 1.0),
    report_csv=params.get("report_csv", "fourbar_report.csv"),
    theta_limits=params.get("theta_limits"),
    direction=-1
)