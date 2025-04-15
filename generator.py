import json
import itertools
import os

def set_nested_value(d, path, value):
    keys = path.split('.')
    for k in keys[:-1]:
        d = d.setdefault(k, {})
    d[keys[-1]] = value

def get_param_string(combo):
    return "_".join([f"{k.split('.')[-1]}{v}" for k, v in combo.items()])

def compute_extras(config):
    for key in ["main_propeller", "small_propellers"]:
        prop = config[key]
        diameter = prop["diameter"]
        radius = diameter / 2
        chord_mean = (prop["chord_root"] + prop["chord_tip"]) / 2
        NB = prop["NB"]

        # Compute Aspect Ratio and Rotor Solidity
        AR = radius**2 / (radius*chord_mean)
        solidity = (NB * chord_mean) / (3.14159 * radius)

        # Add computed values to config
        prop["aspect_ratio"] = round(AR, 4)
        print(f"Aspect Ratio for {key}: {prop['aspect_ratio']}")
        prop["rotor_solidity"] = round(solidity, 4)
        print(f"Rotor Solidity for {key}: {prop['rotor_solidity']}")

def generate_flexible_configs(base_config, variables_to_vary=None, case="def"):
    file_names = []
    if variables_to_vary:
        keys, values_list = zip(*variables_to_vary.items())
        for values in itertools.product(*values_list):
            config = json.loads(json.dumps(base_config))  # deep copy
            combo = dict(zip(keys, values))
            for k, v in combo.items():
                set_nested_value(config, k, v)
            
            compute_extras(config)

            param_str = get_param_string(combo)
            os.makedirs("configs", exist_ok=True)
            file_names.append(f"{case}_{param_str}.json")
            with open(f"./configs/{case}_{param_str}.json", "w") as f:
                json.dump(config, f, indent=2)
    else:
        with open(f"{case}.json", "w") as f:
            json.dump(base_config, f, indent=2)

    return file_names

# --- Use your full base config ---
base_config = {
    "main_propeller": {
        "POSITIONING": "__",
        "position": [0, 0, 0],
        "angles": [0, 0, 0],
        "GEOMETRY": "__",
        "airfoil": "a18sm",
        "hub": 0.1,
        "diameter": 2.2,
        "NB": 3,
        "chord_root": 0.10,
        "chord_tip": 0.05,
        "pitch_root": 12,
        "pitch_tip": 4.5,
        "OPERATION": "__",
        "rpm": 370,
        "REFINEMENT": "__",
        "n": 30,
        "wake_length": 2,
        "INITIAL": "__",
        "uWake": 2,
        "optimal_AoA": 5.0
    },
    "small_propellers": {
        "POSITIONING": "__",
        "angle": 90,
        "GEOMETRY": "__",
        "diameter": 0.15,
        "NB": 2,
        "chord_root": 0.02,
        "chord_tip": 0.005,
        "hub": 0.03,
        "pitch_root": 88,
        "pitch_tip": 20,
        "AoA": 4.5,
        "OPERATION": "__",
        "rpm": 10000,
        "REFINEMENT": "__",
        "n": 10,
        "wake_length": 5,
        "INITIAL": "__",
        "uWake": 30
    },
    "settings": {
        "contraction": True,
        "reynolds": False,
        "wind_speed": 0,
        "wind_angle": 0
    },
    "results": {
        "Thrust": 31.27629225772396,
        "Torque": 2.5314177114138916,
        "FM": 0.7383810294162881
    }
}

# --- Define what parameters to sweep ---
variables_to_test = {
    "main_propeller.NB": [2, 3, 4]
}

generate_flexible_configs(base_config, variables_to_test, case="study")
