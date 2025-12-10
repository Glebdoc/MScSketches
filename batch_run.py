import os
for var in [
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "BLIS_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS"
]:
    os.environ[var] = "1"
os.environ["OMP_DYNAMIC"] = "false"
import argparse
import json


from typing import Tuple, List
import numpy as np
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

from bemUtils import update_config
from geometry import defineDrone
from trimmer_helicopter import HelicopterTrimmer
from trimmer_quadcopter import QuadcopterTrimmer
from trimmer_drone import DroneTrimmer
from strategies import HelicopterStrategy, QuadcopterStrategy, DroneStrategy

MTOW = 60  # N
#BATCH_FOLDER = './Factorial_trial'
BATCH_FOLDER = './AzimuthStudy11000Clean'
RUN_HELICOPTER = False
RUN_QUADCOPTER = False
RUN_DRONE = False
RUN_AZIMUTH_STUDY = True


def ensure_header(csv_path: str, header: str) -> None:
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        with open(csv_path, 'w') as f:
            f.write(header)


def parse_indices(spec: str) -> List[int]:
    """Parse '3,7-9,12' -> [3,7,8,9,12]."""
    out: List[int] = []
    for tok in spec.split(','):
        tok = tok.strip()
        if '-' in tok:
            a, b = tok.split('-', 1)
            out.extend(range(int(a), int(b) + 1))
        elif tok:
            out.append(int(tok))
    return sorted(set(out))


# --------------------------
# WORKERS (run in subprocess)
# --------------------------

def helicopter_worker(i: int, design_row: np.ndarray, performance_summary_path: str) -> Tuple[int, str]:
    path = f'{BATCH_FOLDER}/helicopter/DP{i}'
    os.makedirs(path, exist_ok=True)

    path_json = update_config(design_row, aircraft_type='helicopter', path=path)

    trimmer = HelicopterTrimmer(HelicopterStrategy(), mtow=MTOW, output_dir=path)
    solver, iters, err, main_RPM = trimmer.trim_thrust(
        path_json, target_thrust=trimmer.mtow, main_RPM=400, bounds=(300, 500)
    )
    solver.main_RPM = main_RPM
    solver.save_results(path)

    with open(f'{path}/performance.json', 'r') as f:
        performance = json.load(f)

    csv_line = (
        f"DP{i}, {design_row[0]}, {design_row[1]}, {design_row[2]}, "
        f"{performance['thrust']}, {performance['torque']}, {performance['p_induced']}, "
        f"{performance['p_profile']}, {performance['p_total']}, {performance['p_ideal']}, "
        f"{performance['power_loading']}, {performance['figure_of_merit']}, {performance['STALL_HIGH']}, "
        f"{performance['STALL_LOW']}, {performance['main_RPM']}, {performance['disk_loading']}\n"
    )
    return i, csv_line


def quadcopter_worker(i: int, design_row: np.ndarray, performance_summary_path: str) -> Tuple[int, str]:
    path = f'{BATCH_FOLDER}/quadcopter/DP{i}'
    os.makedirs(path, exist_ok=True)

    path_json = update_config(design_row, aircraft_type='quadcopter', path=path)


    trimmer = QuadcopterTrimmer(QuadcopterStrategy(), mtow=MTOW, output_dir=path)
    solver, iters, err, main_RPM = trimmer.trim_thrust(
        path_json, main_RPM=4000, bounds=(3000, 6000), target_thrust=trimmer.mtow
    )
    solver.main_RPM = main_RPM
    solver.save_results(path)

    with open(f'{path}/performance.json', 'r') as f:
        performance = json.load(f)

    csv_line = (
        f"DP{i}, {design_row[0]}, {design_row[1]}, {design_row[2]}, {design_row[3]}, "
        f"{performance['thrust']}, {performance['torque']}, {performance['p_induced']}, "
        f"{performance['p_profile']}, {performance['p_total']}, {performance['p_ideal']}, "
        f"{performance['power_loading']}, {performance['figure_of_merit']}, {performance['STALL_HIGH']}, "
        f"{performance['STALL_LOW']}, {performance['main_RPM']}, {performance['disk_loading']}\n"
    )
    return i, csv_line


def drone_worker(i: int, design_row: np.ndarray, performance_summary_path: str) -> Tuple[int, str]:
    path = f'{BATCH_FOLDER}/drone/DP{i}'
    os.makedirs(path, exist_ok=True)

    path_json = update_config(design_row, aircraft_type='drone', path=path)

    trimmer = DroneTrimmer(DroneStrategy(), mtow=MTOW, output_dir=path)
    rpm_main, rpm_small, solver = trimmer.trim_thrust(
        main_RPM=390,   
        config=path_json,
        rpm_small=13000.0,
        bounds_aux=(250.0, 500.0),
        bounds_moment=(5_000.0, 20_000.0),
        mtow=60.0
    )
    solver.save_results(path)

    with open(f'{path}/performance.json', 'r') as f:
        performance = json.load(f)
    ##DL, sigma_main, Re_min, lambda_p, pitch_small, taper
    csv_line = (
        f"DP{i}, {design_row[0]}, {design_row[1]}, {design_row[2]}, {design_row[3]}, {design_row[4]}, "
        f"{performance['thrust_main']}, {performance['thrust_small']}, {performance['torque_main']}, {performance['torque_small']}, "
        f"{performance['moment_created']}, {performance['power_required']}, "
        f"{performance['p_induced']}, {performance['p_profile']}, {performance['p_total']}, {performance['p_ideal']}, "
        f"{performance['power_loading']}, {performance['figure_of_merit']}, "
        f"{performance['STALL_MAIN_HIGH']}, {performance['STALL_MAIN_LOW']}, "
        f"{performance['STALL_SMALL_HIGH']}, {performance['STALL_SMALL_LOW']}, "
        f"{rpm_main}, {rpm_small}, {performance['disk_loading']}\n"
    )
    return i, csv_line

def drone_azimuth_worker(i: int, design_row: np.ndarray, performance_summary_path: str) -> Tuple[int, str]:
    path = f'{BATCH_FOLDER}/drone/DP{i}'
    os.makedirs(path, exist_ok=True)

    path_json = update_config(design_row, aircraft_type='azimuth', path=path)
    print('File path json:', path_json)
    #trimmer = DroneTrimmer(DroneStrategy(), mtow=MTOW, output_dir=path)
    trimmer = HelicopterTrimmer(HelicopterStrategy(), mtow=MTOW, output_dir=path)
    print('Trimmer created')
    solver, iter, err = trimmer.trim_velocity(
        config=path_json,
        main_RPM=11000,
        # small_RPM=11000,
    )
    print('Trimmer trimmed')
    solver.save_results(path)
    print('After trim velocity')
    solver.plot_self(save=True)

    with open(f'{path}/performance.json', 'r') as f:
        performance = json.load(f)
    ##DL, sigma_main, Re_min, lambda_p, pitch_small, taper
    csv_line = (
        f"DP{i}, {design_row[0]}, "
        f"{performance['thrust_main']}, {performance['thrust_small']}, {performance['torque_main']}, {performance['torque_small']}, "
        f"{performance['moment_created']}, {performance['power_required']}, "
        f"{performance['p_induced']}, {performance['p_profile']}, {performance['p_total']}, {performance['p_ideal']}, "
        f"{performance['power_loading']}, {performance['figure_of_merit']}, "
        f"{performance['STALL_MAIN_HIGH']}, {performance['STALL_MAIN_LOW']}, "
        f"{performance['STALL_SMALL_HIGH']}, {performance['STALL_SMALL_LOW']}, "
        f"{0}, {rpm_small}, {performance['disk_loading']}\n"
    )
    return i, csv_line


def drone_sensitivity_worker(i: int, design_row: np.ndarray, performance_summary_path: str) -> Tuple[int, str]:
    path = f'{BATCH_FOLDER}/drone/DP{i}'
    os.makedirs(path, exist_ok=True)

    path_json = update_config(design_row, aircraft_type='azimuth', path=path)

    trimmer = DroneTrimmer(DroneStrategy(), mtow=MTOW, output_dir=path)
    solver, iter, err = trimmer.trim_velocity(
        config=path_json,
        main_RPM=390,
        small_RPM=11000,
    )
    solver.save_results(path)
    solver.plot_self(save=True)

    with open(f'{path}/performance.json', 'r') as f:
        performance = json.load(f)
    ##DL, sigma_main, Re_min, lambda_p, pitch_small, taper
    csv_line = (
        f"DP{i}, {design_row[0]}, "
        f"{performance['thrust_main']}, {performance['thrust_small']}, {performance['torque_main']}, {performance['torque_small']}, "
        f"{performance['moment_created']}, {performance['power_required']}, "
        f"{performance['p_induced']}, {performance['p_profile']}, {performance['p_total']}, {performance['p_ideal']}, "
        f"{performance['power_loading']}, {performance['figure_of_merit']}, "
        f"{performance['STALL_MAIN_HIGH']}, {performance['STALL_MAIN_LOW']}, "
        f"{performance['STALL_SMALL_HIGH']}, {performance['STALL_SMALL_LOW']}, "
        f"{0}, {rpm_small}, {performance['disk_loading']}\n"
    )
    return i, csv_line




# --------------------------
# SEQUENTIAL ORCHESTRATION
# --------------------------

def run_block(
    name: str,
    designs: np.ndarray,
    indices: List[int],
    header: str,
    csv_path: str,
    worker,
    workers: int
) -> None:
    """Run a set of DPs possibly in parallel, then write CSV lines sequentially."""
    ensure_header(csv_path, header)

    if workers == 1:
        for i in indices:
            try:
                _, line = worker(i, designs[i], os.path.dirname(csv_path))
                with open(csv_path, 'a') as f:
                    f.write(line)
                print(f"{name} DP{i} completed.")
            except Exception as e:
                print(f"[ERROR] {name} DP{i} failed: {e}")
        return

    print(f"[{name}] Running {len(indices)} DPs with {workers} workers ...")
    futures = {}
    with ProcessPoolExecutor(max_workers=workers) as ex:
        for i in indices:
            print('design[i]', designs[i])
            futures[ex.submit(worker, i, designs[i], os.path.dirname(csv_path))] = i

        results: List[Tuple[int, str]] = []
        for fut in as_completed(futures):
            i = futures[fut]
            try:
                res = fut.result()
                results.append(res)
                print(f"{name} DP{i} completed.")
            except Exception as e:
                print(f"[ERROR] {name} DP{i} failed: {e}")

    results.sort(key=lambda t: t[0])
    with open(csv_path, 'a') as f:
        for _, line in results:
            f.write(line)


# --------------------------
# MAIN EXECUTION
# --------------------------

if __name__ == "__main__":
    start_time = time.time()
    parser = argparse.ArgumentParser(description="Batch run factorial designs with resume / selective rerun.")
    parser.add_argument("--start", type=int, default=None,
                        help="Start from this DP index (inclusive) and run to the end.")
    parser.add_argument("--only", type=str, default=None,
                        help="Comma-separated list/ranges of DP indices to run (e.g., '3,7-9'). Overrides --start.")
    parser.add_argument("--workers", type=int, default=min(4, (os.cpu_count() or 2)),
                        help="Number of parallel workers for DP evaluation (use 1 for sequential).")
    args = parser.parse_args()

    performance_summary_path = f'{BATCH_FOLDER}/factorial_data/'

    if RUN_HELICOPTER:
        helicopter_designs = np.loadtxt(f'{BATCH_FOLDER}/factorial_data/helicopter_factorial_design.csv',
                                        delimiter=',', skiprows=1)
        csv_path = f'{performance_summary_path}/performance_summary_helicopter.csv'
        header = ('DP, DL, sigma_main, taper, thrust, torque, p_induced, p_profile, p_total, '
                  'p_ideal, power_loading, figure_of_merit, STALL_HIGH, STALL_LOW, main_RPM, disk_loading\n')

        if args.only:
            indices = parse_indices(args.only)
        else:
            start = args.start if args.start is not None else 0
            indices = list(range(start, len(helicopter_designs)))

        run_block("HELICOPTER", helicopter_designs, indices, header, csv_path, helicopter_worker, args.workers)

    if RUN_QUADCOPTER:
        quadcopter_designs = np.loadtxt(f'{BATCH_FOLDER}/factorial_data/quadcopter_factorial_design.csv',
                                        delimiter=',', skiprows=1)
        csv_path = f'{performance_summary_path}/performance_summary_quadcopter.csv'
        header = ('DP, DL, diagonal, taper, sigma, thrust, torque, p_induced, p_profile, '
                  'p_total, p_ideal, power_loading, figure_of_merit, STALL_HIGH, STALL_LOW, main_RPM, disk_loading\n')

        if args.only:
            indices = parse_indices(args.only)
        else:
            start = args.start if args.start is not None else 0
            indices = list(range(start, len(quadcopter_designs)))

        run_block("QUADCOPTER", quadcopter_designs, indices, header, csv_path, quadcopter_worker, args.workers)

    if RUN_DRONE:
        drone_designs = np.loadtxt(f'{BATCH_FOLDER}/factorial_data/drone_factorial_design.csv',
                                   delimiter=',', skiprows=1)
        csv_path = f'{performance_summary_path}/performance_summary_drone.csv'
        #DL, sigma_main, Re_min, lambda_p, pitch_small, taper
        header = ('DP, DL, sigma_main, Re_min, lambda_p, pitch_small, taper, thrust_main, thrust_small, torque_main, torque_small, '
                  'moment_created, power_required, '
                  'p_induced, p_profile, p_total, p_ideal, power_loading, figure_of_merit, '
                  'STALL_MAIN_HIGH, STALL_MAIN_LOW, STALL_SMALL_HIGH, STALL_SMALL_LOW, '
                  'main_RPM, small_RPM, disk_loading\n')

        if args.only:
            indices = parse_indices(args.only)
        else:
            start = args.start if args.start is not None else 0
            indices = list(range(start, len(drone_designs)))

        run_block("DRONE", drone_designs, indices, header, csv_path, drone_worker, args.workers)

    if RUN_AZIMUTH_STUDY:
        drone_designs = np.loadtxt(f'{BATCH_FOLDER}/factorial_data/azimuth_factorial_design.csv',
                                   delimiter=',', skiprows=1)

        csv_path = f'{performance_summary_path}/performance_summary_azimuth_drone.csv'
        #DL, sigma_main, Re_min, lambda_p, pitch_small, taper
        header = ('DP, DL, sigma_main, Re_min, lambda_p, pitch_small, taper, thrust_main, thrust_small, torque_main, torque_small, '
                  'moment_created, power_required, '
                  'p_induced, p_profile, p_total, p_ideal, power_loading, figure_of_merit, '
                  'STALL_MAIN_HIGH, STALL_MAIN_LOW, STALL_SMALL_HIGH, STALL_SMALL_LOW, '
                  'main_RPM, small_RPM, disk_loading\n')

        if args.only:
            indices = parse_indices(args.only)
            print('I\'m here')
        else:
            start = args.start if args.start is not None else 0
            indices = list(range(start, len(drone_designs)))

            print('I\'m here',start, indices)

        run_block("DRONE", drone_designs, indices, header, csv_path, drone_azimuth_worker, args.workers)

    end_time = time.time()
    print(f"Batch run completed in {(end_time - start_time)/60:.2f} minutes.")
