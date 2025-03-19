import json

##############################################
# This is a mass breakdown for the drone 
# The mass is broken down into the following categories:

# 1. Electronics

# 2. Structure

# 3. Payload

##############################################

with open("INPUT.json", "r") as file:
    data = json.load(file)

engines_mass = data["ENGINE"]["MASS"]* data["INPUT"]["NB"]
console_mass = data["INPUT"]["D"]*data["INPUT"]["CONSOLE_mass"]*data["INPUT"]["NB"]

battery_config = data["INPUT"]["BATTERY_config"]
battery_mass = data["BATTERY"][battery_config]["MASS_g"]/1000*data["BATTERY"][battery_config]["N"]

payload_mass = data["VAR"]["PAYLOAD"]    

total_mass = engines_mass + console_mass + battery_mass + payload_mass

total_mass += total_mass*0.3

print("Total mass of the drone is: ", total_mass, "kg", total_mass*9.81, "N")
