import pandas as pd
import json
import os

output_path = "./factorial_data/performance_summary_drone.csv"
factorial_data_path = "./factorial_data/drone_factorial_design.csv"

# Columns you expect in the summary
COLUMNS = [
    "DP", "DL", "sigma_main", "Re_min", "lambda_p", "pitch_small",
    "thrust_main", "thrust_small", "torque_main", "torque_small",
    "moment_created", "power_required", "p_induced", "p_profile",
    "p_total", "p_ideal", "power_loading", "figure_of_merit",
    "STALL_MAIN_HIGH", "STALL_MAIN_LOW", "STALL_SMALL_HIGH", "STALL_SMALL_LOW",
    "main_RPM", "small_RPM", "disk_loading"
]

def update_performance_summary():
    # Load factorial designs
    df_designs = pd.read_csv(factorial_data_path)

    # Prepare list of new results
    new_rows = []
    N = 47

    for i in range(N):
        design = df_designs.iloc[i]
        json_path = f"./drone/DP{i}/performance.json"

        if not os.path.exists(json_path):
            print(f"Warning: {json_path} not found, skipping.")
            continue

        with open(json_path, "r") as f:
            performance = json.load(f)
            print(f"Loaded performance for DP{i}", performance.keys())
            
        # disk loading,sigma_main,Re_min,lambda_p,pitch_small,taper

        row = {
            "DP": f"DP{i}",
            "DL": design.get("disk loading", None),
            "sigma_main": design.get("sigma_main", None),
            "Re_min": design.get("Re_min", None),
            "lambda_p": design.get("lambda_p", None),
            "pitch_small": design.get("pitch_small", None),
            "thrust_main": performance.get("thrust_main"),
            "thrust_small": performance.get("thrust_small"),
            "torque_main": performance.get("torque_main"),
            "torque_small": performance.get("torque_small"),
            "moment_created": performance.get("moment_created"),
            "power_required": performance.get("power_required"),
            "p_induced": performance.get("p_induced"),
            "p_profile": performance.get("p_profile"),
            "p_total": performance.get("p_total"),
            "p_ideal": performance.get("p_ideal"),
            "power_loading": performance.get("power_loading"),
            "figure_of_merit": performance.get("figure_of_merit"),
            "STALL_MAIN_HIGH": performance.get("STALL_MAIN_HIGH"),
            "STALL_MAIN_LOW": performance.get("STALL_MAIN_LOW"),
            "STALL_SMALL_HIGH": performance.get("STALL_SMALL_HIGH"),
            "STALL_SMALL_LOW": performance.get("STALL_SMALL_LOW"),
            "main_RPM": performance.get("RPM_main"),
            "small_RPM": performance.get("RPM_small"),
            "disk_loading": performance.get("disk_loading"),
        }

        new_rows.append(row)

    if not new_rows:
        print("No new valid results found.")
        return

    df_new = pd.DataFrame(new_rows, columns=COLUMNS)

    # Append to CSV or create a new one
    if os.path.exists(output_path):
        df_new.to_csv(output_path, mode='a', header=False, index=False)
    else:
        df_new.to_csv(output_path, mode='w', header=True, index=False)

    print(f"✅ Updated performance summary with {len(df_new)} new designs.")

update_performance_summary()
