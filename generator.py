import json 

def generate_config(NB=None, small_prop_angle=None, case="def"):

    if NB != None:
        for i in range(len(NB)):
            for j in range(len(small_prop_angle)):
                config = {
                    "main_propeller": {
                        "position": [0, 0, 0],
                        "angles": [0, 0, 0],
                        "hub": 0.2,
                        "diameter": 3,
                        "number_of_blades": NB[i],
                        "pitch": 0.2,
                        "rpm": 350,
                        "chord": 0.3,
                        "n": 40
                    },
                    "small_propellers": {
                        "angle": small_prop_angle[j],
                        "diameter": 0.3,
                        "number_of_blades": 3,
                        "rpm": 10000,
                        "chord": 0.05,
                        "n": 10
                    },
                }
                with open(f"./configs/{case}_NB{NB[i]}_inc{small_prop_angle[j]}.json", "w") as f:
                    json.dump(config, f)
    else:
        config = {
            "main_propeller": {
                "position": [0, 0, 0],
                "angles": [0, 0, 0],
                "hub": 0.3,
                "diameter": 3,
                "number_of_blades": 3,
                "pitch": 0.2,
                "rpm": 350,
                "chord": 0.3,
                "n": 10
            },
            "small_propellers": {
                "angle": 60,
                "diameter": 0.3,
                "number_of_blades": 3,
                "rpm": 10000,
                "chord": 0.05,
                "n": 5
            },
        }

        with open(f"{case}.json", "w") as f:
            json.dump(config, f)

generate_config(NB=[3, 4], small_prop_angle=[90, 75, 60], case="def")  # Generates 3 files: default_NB3.json, default_NB4.json, default_NB5.json