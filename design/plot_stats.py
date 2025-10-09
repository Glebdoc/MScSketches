import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

def set_bw_design():
    """
    Configure matplotlib + seaborn for consistent black & white (grayscale) plotting style.
    """
    # Seaborn & matplotlib styles
    sns.set_style("whitegrid")
    plt.style.use("seaborn-v0_8")
    sns.set_palette("Greys_r")  # grayscale palette
    
    # Global rcParams for consistent looks
    plt.rcParams.update({
        "figure.figsize": (6, 4),
        "figure.dpi": 200,
        "axes.edgecolor": "black",
        "axes.labelweight": "bold",
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.color": "black",
        "grid.linewidth": 0.5,
        "legend.frameon": True,
        "legend.fancybox": False,
        "legend.shadow": False,  # cleaner for B&W
        "lines.linewidth": 1.5,
        "lines.markersize": 3,
    })
    
    # Line styles / markers / colors for B&W distinction
    design = {
        "line_styles": ['-', '--', '-.'],
        "markers": ['o', 's', '^'],
        "colors": ['black', '0.4', '0.7']  # black, dark gray, light gray
    }
    return design

# Load the CSV file
df = pd.read_csv('./quad_stats.csv')

MTOW = df['MTOW [kg]']
PAYLOAD = df['Max Payload [kg]']
PROP_DIAMETER = df['Prop Diameter [m]']
AREA = 4 * np.pi*(PROP_DIAMETER/2)**2
DL = MTOW*9.81/AREA


# fit a parabolic line 
#m, b, c= np.polyfit(MTOW, DL, 2)

fig, = plt.figure(), 
design = set_bw_design()    

plt.scatter(MTOW*9.81, DL)
m, b = np.polyfit(MTOW*9.81, DL, 1)
MTOW = np.array(MTOW*9.81)
MTOW.sort()
plt.plot(MTOW, m*MTOW + b, color='red')
# plot a point where MTOW=60N, print a corresponding DL value
plt.scatter(60, m*60 + b, color='white', edgecolor='black', s=25, zorder=10000, linewidths=2)
plt.text(60 - 5, m*60 + b + 5 -2, f'DL: {m*60 + b:.2f} N/m²', fontsize=12, verticalalignment='bottom', horizontalalignment='center')

plt.xlabel('MTOW [N]')
plt.ylabel('Disk Loading [N/m²]')
plt.show()

# fig, = plt.figure(), 
# design = set_bw_design()    

# plt.scatter(MTOW, PAYLOAD)
# m, b = np.polyfit(MTOW, PAYLOAD, 1)
# plt.plot(MTOW, m*MTOW + b, color='red')
# # plot a point where PAYLOAD = 1 kg, print a corresponding MTOW value 
# plt.scatter((1-b)/m, 1, color='white', edgecolor='black', s=25, zorder=10000, linewidths=2)
# plt.text((1-b)/m - 0.3, 1, f'MTOW: {(1-b)/m:.2f} kg', fontsize=12, verticalalignment='bottom', horizontalalignment='right')

# plt.xlabel('MTOW [kg]')
# plt.ylabel('Max Payload [kg]')
# plt.show()



# Display basic statistics
