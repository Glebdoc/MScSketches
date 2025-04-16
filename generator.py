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

