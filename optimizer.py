# --- imports you already have ---
import os, json, hashlib
import numpy as np
from scipy.optimize import differential_evolution
from bemUtils import update_config
from trimmer_drone import DroneTrimmer
from strategies import DroneStrategy

class DroneObjective:
    def __init__(self, out_dir, mtow, stall_thr, thrust_tol, moment_tol,
                 main_rpm0, small_rpm0, bounds_main, bounds_mom):
        self.out_dir = out_dir
        self.mtow = float(mtow)
        self.stall_thr = float(stall_thr)
        self.thrust_tol = float(thrust_tol)
        self.moment_tol = float(moment_tol)
        self.main_rpm0 = float(main_rpm0)
        self.small_rpm0 = float(small_rpm0)
        self.bounds_main = bounds_main
        self.bounds_mom  = bounds_mom
        os.makedirs(self.out_dir, exist_ok=True)
        self.cache = {}  # per-process memo

    @staticmethod
    def _hash_vec(x):
        return hashlib.md5(np.asarray(x, dtype=np.float64).tobytes()).hexdigest()

    def __call__(self, x):
        key = self._hash_vec(x)
        if key in self.cache:
            return self.cache[key]

        # 1) write config for this design vector (order must match your model)
        design_row = np.asarray(x, dtype=float)
        path = os.path.join(self.out_dir, f"eval_{key}")
        os.makedirs(path, exist_ok=True)
        cfg_path = update_config(design_row, aircraft_type='drone', path=path)

        # 2) run internal trimmer (enforces thrust & moment balance)
        trimmer = DroneTrimmer(DroneStrategy(), mtow=self.mtow, output_dir=path)
        try:
            rpm_main, rpm_small, solver = trimmer.trim_thrust(
                main_RPM=self.main_rpm0,
                config=cfg_path,
                rpm_small=self.small_rpm0,
                bounds_aux=self.bounds_main,
                bounds_moment=self.bounds_mom,
                mtow=self.mtow
            )
            solver.save_results(path=path)  
        except Exception:
            self.cache[key] = 1e9
            return self.cache[key]

        # 3) read performance and compute penalized objective
        try:
            with open(os.path.join(path, "performance.json"), "r") as f:
                perf = json.load(f)
        except Exception:

            self.cache[key] = 1e9
            return self.cache[key]

        thrust_err = abs(perf.get('thrust_main', 0.0) - self.mtow)
        moment_err = abs(perf.get('moment_created', 0.0) - perf.get('torque_main', 0.0))

        stall_vals = [
            perf.get('STALL_MAIN_HIGH', 0.0),
            perf.get('STALL_MAIN_LOW',  0.0),
            perf.get('STALL_SMALL_HIGH',0.0),
            perf.get('STALL_SMALL_LOW', 0.0),
        ]
        stall_violation = sum(max(0.0, s - self.stall_thr) for s in stall_vals)

        power = perf.get('power_required', np.inf)

        penalty = (
            1e6 * stall_violation +
            1e6 * max(0.0, thrust_err - self.thrust_tol) +
            1e6 * max(0.0, moment_err - self.moment_tol)
        )
        value = power + penalty

        # optional: per-process log
        with open(os.path.join(self.out_dir, "log.csv"), "a") as lf:
            lf.write(
                f"{key}," + ",".join(f"{v:.6g}" for v in x) + "," +
                f"{power:.6f},{thrust_err:.3e},{moment_err:.3e},{stall_violation:.3e},{value:.6f}\n"
            )

        self.cache[key] = value
        return value



# ---- config (same as you had) ----
MTOW = 60.0
STALL_THR = 0.15
THRUST_TOL = 1e-2
MOMENT_TOL = 1e-3
OUT_DIR = "./Opt_OneRun"

BOUNDS = [
    (10.0,   60.0),   # DL
    (0.04,   0.14),   # sigma_main
    (5.0e4,  1.5e5),  # Re_min
    (0.075,    0.2),    # lambda_p
    (60,    70),    # pitch_small
    (0.5,    1.2),    # taper
]

MAIN_RPM0  = 340.0
SMALL_RPM0 = 13000.0
BOUNDS_MAIN  = (250.0, 500.0)
BOUNDS_MOM   = (5_000.0, 20_000.0)

def run_single_design_optimization(seed=42, workers=2):
    obj = DroneObjective(
        out_dir=OUT_DIR,
        mtow=MTOW,
        stall_thr=STALL_THR,
        thrust_tol=THRUST_TOL,
        moment_tol=MOMENT_TOL,
        main_rpm0=MAIN_RPM0,
        small_rpm0=SMALL_RPM0,
        bounds_main=BOUNDS_MAIN,
        bounds_mom=BOUNDS_MOM
    )

    result = differential_evolution(
        obj,
        bounds=BOUNDS,
        strategy="best1bin",
        maxiter=80,
        popsize=6,
        tol=1e-3,
        mutation=(0.5, 1.0),
        recombination=0.7,
        polish=True,
        seed=seed,
        updating="deferred",
        workers=workers   # parallel evaluations
    )
    return result

if __name__ == "__main__":
    import argparse
    import os

    parser = argparse.ArgumentParser(description="Global optimization of drone design variables.")
    parser.add_argument("--workers", type=int, default=min(4, os.cpu_count() or 2),
                        help="Number of parallel workers (for differential_evolution).")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility.")
    args = parser.parse_args()

    print(f"\n[Optimizer] Starting with {args.workers} workers ...")

    result = run_single_design_optimization(seed=args.seed, workers=args.workers)

    print("\n=== Optimization complete ===")
    print("Status: ", "Success" if result.success else "Failed")
    print("Best design vector (DL, sigma_main, Re_min, lambda_p, pitch_small, taper):")
    print(result.x)
    print(f"Objective (Power Required + Penalties): {result.fun:.6f}")
    print(f"Total iterations: {result.nit}")
    print(f"Function evaluations: {result.nfev}")
    print(f"Results saved to: {OUT_DIR}\n")
