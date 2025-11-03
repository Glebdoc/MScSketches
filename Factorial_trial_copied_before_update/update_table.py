import pandas as pd
import numpy as np

#DP, DL, sigma_main, Re_min, lambda_p, pitch_small, thrust_main, 
# thrust_small, torque_main, torque_small, moment_created, 
# power_required, p_induced, p_profile, p_total, p_ideal, 
# power_loading, figure_of_merit, STALL_MAIN_HIGH, STALL_MAIN_LOW, STALL_SMALL_HIGH, STALL_SMALL_LOW, 
# main_RPM, small_RPM, disk_loading

output_path = "./factorial_data/performance_summary_drone.csv"
factorial_data_path = "./factorial_data/drone_factorial_designs.csv"

def update_performance_summary():
    # Load existing performance summary
    try:
        df_summary = pd.read_csv(output_path)
    except FileNotFoundError:
        df_summary = pd.DataFrame()

    # Load factorial design data
    df_designs = pd.read_csv(factorial_data_path)

    N = 40 # Number of new designs to process
    new_rows = []
    for i in range(N):
        design_row = df_designs.iloc[i].values
        performance = np.load(f"./drone/DP{i}/_.json")
        csv_line = (
        f"DP{i}, {design_row[0]}, {design_row[1]}, {design_row[2]}, {design_row[3]}, "
        f"{performance['thrust']}, {performance['torque']}, {performance['p_induced']}, "
        f"{performance['p_profile']}, {performance['p_total']}, {performance['p_ideal']}, "
        f"{performance['power_loading']}, {performance['figure_of_merit']}, {performance['STALL_HIGH']}, "
        f"{performance['STALL_LOW']}, {performance['main_RPM']}, {performance['disk_loading']}\n"
    )
        new_rows.append(csv_line)
    
    # Append new rows to the summary DataFrame
    with open(output_path, 'a') as f:
        for row in new_rows:
            f.write(row)
    print(f"Updated performance summary with {N} new designs.")

update_performance_summary()
