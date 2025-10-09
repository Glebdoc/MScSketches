import argparse
import numpy as np
import json, os
from bemUtils import update_config
from geometry import defineDrone
from trimmer_helicopter import HelicopterTrimmer
from trimmer_quadcopter import QuadcopterTrimmer
from trimmer_drone import DroneTrimmer
from strategies import HelicopterStrategy, QuadcopterStrategy, DroneStrategy

MTOW = 60  # N
BATCH_FOLDER = './Factorial_trial'
RUN_HELICOPTER = False
RUN_QUADCOPTER = False
RUN_DRONE = True

def ensure_header(csv_path: str, header: str) -> None:
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        with open(csv_path, 'w') as f:
            f.write(header)

def parse_indices(spec: str) -> list[int]:
    """Parse '3,7-9,12' -> [3,7,8,9,12]."""
    out = []
    for tok in spec.split(','):
        tok = tok.strip()
        if '-' in tok:
            a, b = tok.split('-', 1)
            out.extend(range(int(a), int(b) + 1))
        elif tok:
            out.append(int(tok))
    # unique & sorted
    return sorted(set(out))

def run_helicopter_dp(i: int, design_row: np.ndarray, performance_summary_path: str) -> None:
    path = f'{BATCH_FOLDER}/helicopter/DP{i}'
    os.makedirs(path, exist_ok=True)

    # create config for this DP
    path_json = update_config(design_row, aircraft_type='helicopter', path=path)

    # trim & solve
    trimmer = HelicopterTrimmer(HelicopterStrategy(), mtow=MTOW)
    solver, iters, err, main_RPM = trimmer.trim_thrust(path_json, target_thrust=trimmer.mtow, main_RPM=400, bounds=(300, 600))
    solver.main_RPM = main_RPM
    solver.save_results(path)

    # append summary
    with open(f'{path}/performance.json', 'r') as f:
        performance = json.load(f)

    with open(f'{performance_summary_path}/performance_summary_helicopter.csv', 'a') as f:
        f.write((
            f"DP{i}, {design_row[0]}, {int(design_row[1])}, {design_row[2]}, {design_row[3]}, "
            f"{performance['thrust']}, {performance['torque']}, {performance['p_induced']}, "
            f"{performance['p_profile']}, {performance['p_total']}, {performance['p_ideal']}, "
            f"{performance['power_loading']}, {performance['figure_of_merit']}, {performance['STALL_HIGH']}, "
            f"{performance['STALL_LOW']}, {performance['main_RPM']}, {performance['disk_loading']}\n"
        ))

    print(
        f"Completed DP{i} | main_RPM: {performance['main_RPM']:.2f}, "
        f"thrust: {performance['thrust']:.2f}, power: {performance['p_total']:.2f}, "
        f"FoM: {performance['figure_of_merit']:.3f}"
    )

def run_quadcopter_dp(i: int, design_row: np.ndarray, performance_summary_path: str) -> None:
    path = f'{BATCH_FOLDER}/quadcopter/DP{i}'
    os.makedirs(path, exist_ok=True)

    path_json = update_config(design_row, aircraft_type='quadcopter', path=path)

    trimmer = QuadcopterTrimmer(QuadcopterStrategy(), mtow=MTOW)
    solver, iters, err, main_RPM = trimmer.trim_thrust(path_json, main_RPM=4000, 
                                                       bounds=(3000, 6000), 
                                                       target_thrust=trimmer.mtow)
    solver.main_RPM = main_RPM
    solver.save_results(path)

    with open(f'{path}/performance.json', 'r') as f:
        performance = json.load(f)

    with open(f'{performance_summary_path}/performance_summary_quadcopter.csv', 'a') as f:
        f.write((
            f"DP{i}, {design_row[0]}, {design_row[1]}, {int(design_row[2])}, {design_row[3]}, {design_row[4]}, "
            f"{performance['thrust']}, {performance['torque']}, {performance['p_induced']}, "
            f"{performance['p_profile']}, {performance['p_total']}, {performance['p_ideal']}, "
            f"{performance['power_loading']}, {performance['figure_of_merit']}, {performance['STALL_HIGH']}, "
            f"{performance['STALL_LOW']}, {performance['main_RPM']}, {performance['disk_loading']}\n"
        ))

    print(
        f"Completed DP{i} | main_RPM: {performance['main_RPM']:.2f}, "
        f"thrust: {performance['thrust']:.2f}, power: {performance['p_total']:.2f}, "
        f"FoM: {performance['figure_of_merit']:.3f}"
    )

def run_drone_dp(i: int, design_row: np.ndarray, performance_summary_path: str) -> None:
    path = f'{BATCH_FOLDER}/drone/DP{i}'
    print('Running DP', i, 'at', path)
    os.makedirs(path, exist_ok=True)
    print('Design row:', design_row)

    path_json = update_config(design_row, aircraft_type='drone', path=path)
    print('Config path:', path_json)

    trimmer = DroneTrimmer(DroneStrategy(), mtow=MTOW, output_dir=path)
    print('Trimming DP', i)

    rpm_main, rpm_small, solver = trimmer.trim_thrust( main_RPM=340,
        config=path_json,
        rpm_small=13000.0,
        bounds_aux=(250.0, 500.0),
        bounds_moment=(5_000.0, 20_000.0),
        mtow=60.0
    )
    # solver.main_RPM = rpm_main
    # solver.small_RPM = rpm_small
    solver.save_results(path)


    with open(f'{path}/performance.json', 'r') as f:
        performance = json.load(f)

    with open(f'{performance_summary_path}/performance_summary_drone.csv', 'a') as f:
        f.write((
            f"DP{i}, {design_row[0]}, {design_row[1]}, {design_row[2]}, {int(design_row[3])}, {design_row[4]}, "
            f"{performance['thrust_main']}, {performance['thrust_small']}, {performance['torque_main']}, {performance['torque_small']}, "
            f"{performance['moment_created']}, {performance['power_required']}, "
            f"{performance['p_induced']}, {performance['p_profile']}, {performance['p_total']}, {performance['p_ideal']}, "
            f"{performance['power_loading']}, {performance['figure_of_merit']}, "
            f"{performance['STALL_MAIN_HIGH']}, {performance['STALL_MAIN_LOW']}, "
            f"{performance['STALL_SMALL_HIGH']}, {performance['STALL_SMALL_LOW']}, "
            f"{rpm_main}, {rpm_small}, {performance['disk_loading']}\n"
        ))

        #     f'{performance_summary_path}/performance_summary_drone.csv',
        #     header=('DP, R, pitch, R_small, NB_small, sigma_small, thrust_main, thrust_small, torque_main, torque_small, '
        #             'moment_created, power_required '
        #             'p_induced, p_profile, p_total, p_ideal, power_loading, figure_of_merit, '
        #             'STALL_MAIN_HIGH, STALL_MAIN_LOW, STALL_SMALL_HIGH, STALL_SMALL_LOW, '
        #             'main_RPM, small_RPM, disk_loading\n')
        # )


    print(
        f"Completed DP{i} | main_RPM: {performance['main_RPM']:.2f}, small_RPM: {performance['small_RPM']:.2f}, "
        f"thrust: {performance['thrust_main'] + performance['thrust_small']:.2f}, power: {performance['p_total']:.2f}, "
        f"FoM: {performance['figure_of_merit']:.3f}"
    )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Batch run factorial designs with resume / selective rerun.")
    parser.add_argument("--start", type=int, default=None,
                        help="Start from this DP index (inclusive) and run to the end.")
    parser.add_argument("--only", type=str, default=None,
                        help="Comma-separated list/ranges of DP indices to run (e.g., '3,7-9'). Overrides --start.")
    args = parser.parse_args()

    performance_summary_path = f'{BATCH_FOLDER}/factorial_data/'

    if RUN_HELICOPTER:
        helicopter_designs = np.loadtxt(f'{BATCH_FOLDER}/factorial_data/helicopter_factorial_design.csv',
                                        delimiter=',', skiprows=1)

        ensure_header(
            f'{performance_summary_path}/performance_summary_helicopter.csv',
            header=('DP, R, NB, sigma, theta, thrust, torque, p_induced, p_profile, p_total, '
                    'p_ideal, power_loading, figure_of_merit, STALL_HIGH, STALL_LOW, main_RPM, disk_loading\n')
        )

        # decide which indices to run
        if args.only:
            indices = parse_indices(args.only)
        else:
            start = args.start if args.start is not None else 0
            indices = list(range(start, len(helicopter_designs)))

        for i in indices:
            try:
                run_helicopter_dp(i, helicopter_designs[i], performance_summary_path)
            except Exception as e:
                print(f"[ERROR] DP{i} failed: {e}")

    if RUN_QUADCOPTER:
        quadcopter_designs = np.loadtxt(f'{BATCH_FOLDER}/factorial_data/quadcopter_factorial_design.csv',
                                        delimiter=',', skiprows=1)

        ensure_header(
            f'{performance_summary_path}/performance_summary_quadcopter.csv',
            header=('DP, R, diagonal, NB, sigma, theta, thrust, torque, p_induced, p_profile, '
                    'p_total, p_ideal, power_loading, figure_of_merit, STALL_HIGH, STALL_LOW, main_RPM, disk_loading\n')
        )

        if args.only:
            indices = parse_indices(args.only)
        else:
            start = args.start if args.start is not None else 0
            indices = list(range(start, len(quadcopter_designs)))

        for i in indices:
            try:
                run_quadcopter_dp(i, quadcopter_designs[i], performance_summary_path)
            except Exception as e:
                print(f"[ERROR] DP{i} failed: {e}")
            
    if RUN_DRONE:
        drone_designs = np.loadtxt(f'{BATCH_FOLDER}/factorial_data/drone_factorial_design.csv',
                                        delimiter=',', skiprows=1)

        ensure_header(
            f'{performance_summary_path}/performance_summary_drone.csv',
            header=('DP, R, pitch, R_small, NB_small, sigma_small, thrust_main, thrust_small, torque_main, torque_small, '
                    'moment_created, power_required, '
                    'p_induced, p_profile, p_total, p_ideal, power_loading, figure_of_merit, '
                    'STALL_MAIN_HIGH, STALL_MAIN_LOW, STALL_SMALL_HIGH, STALL_SMALL_LOW, '
                    'main_RPM, small_RPM, disk_loading\n')
        )

        if args.only:
            indices = parse_indices(args.only)
        else:
            start = args.start if args.start is not None else 0
            indices = list(range(start, len(drone_designs)))
        for i in indices:
            try:
                run_drone_dp(i, drone_designs[i], performance_summary_path)
            except Exception as e:
                print(f"[ERROR] DP{i} failed: {e}")
                