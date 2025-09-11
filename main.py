import numpy as np
import pyvista as pv
import os, json, csv
from bemUtils import*
import time
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from bemUtils import*
from geometry import*
import plotter as myPlt
from solver import*
from generator import *
from numpy.polynomial import Polynomial

plt.ion() 
# --- Configuration ---
FLAGS = {
    "just_display": False,
    "plot_results": False,
    "save_results": True,
    "run_optimization": False,
    "thrust_optimization": False,
    "helicopter": False,
    "rpm_optimization": False, 
    "display_convergence": False
}

# --- Inputs ---
PATH = 'configs/base.json'
MTOW = 60  # Max Take-Off Weight in N

VARIABLE_SPACE = {
    #"small_propellers.diameter":[0.14,0.15,0.16],  # Main pro,peller angle
    "small_propellers.pitch_root":[85],  # Reynolds number

}

ERR_VEL  = 1e-4
ERR_MOMENT = 1e-1
ERR_THRUST = 1e-1
IT_MAX = 30
WEIGHT_VEL = 0.995 # The heigher the more stable
STALL_TRESHOLD = 0.2
TITLE= 'drone8040_mp_spitch'  # Title for the plot

#SWE - small wake effect
#MWE - main wake effect : no effect
#DSW - downwash on a small propeller

# --- Functions ---
def loadConfig(path):
    with open(path, 'r') as f:
        config = json.load(f)
    return config

def updateVisualization(v, vOld, mainNB, im, cbar, smallNB):
    if smallNB is not None:
        shape = mainNB * smallNB
    else:
        shape = mainNB
    data = abs((v - vOld)).reshape(shape, -1)
    im.set_data(data)
    im.set_clim(vmin=data.min(), vmax=data.max())  # <-- important line
    #cbar.update_normal(im)                         # <-- refresh colorbar limits
    im.axes.figure.canvas.draw()
    im.axes.figure.canvas.flush_events()
    plt.pause(0.01)

def low_pass_filter(signal, cutoff_ratio=0.1):
    fft = np.fft.rfft(signal)
    fft[int(cutoff_ratio*len(fft)):] = 0
    return np.fft.irfft(fft, n=len(signal))

def moving_average(x, w=5):
    return np.convolve(x, np.ones(w)/w, mode='same')

def initializeVaxial(drone, helicopter):
    main_prop_n = drone.main_prop.n
    main_prop_NB = drone.main_prop.NB
    if not helicopter:
        small_prop_NB = drone.small_props[0].NB
        small_prop_n = drone.small_props[0].n
        nsm = (main_prop_n-1)*main_prop_NB
        v_axial_old_main = np.linspace(0, -1, nsm)
        v_axial_old_small = np.linspace(0, -1, int(drone.nPoints+1 - nsm))
        return v_axial_old_main, v_axial_old_small, main_prop_NB, nsm, small_prop_NB, main_prop_n, small_prop_n
    else:
        v_axial_old_main = np.linspace(0, -1, int(drone.nPoints)+1)
        return v_axial_old_main, None, main_prop_NB, None, None, main_prop_n, None
    
def computeVelocityError(v, vOldMain, vOldSmall, helicopter, nsm, mainNB, smallNB, weight, im1, cbar1, im2, cbar2, line3, line4, Gammas, k=0.05, mainN=None, smallN=None, ax=None, updateVisualizationFLAG=False):
    if not helicopter:
        vMain = v[:nsm]
        vSmall = v[nsm:]

        vMainNorm = vMain/np.linalg.norm(vMain)
        vSmallNorm = vSmall/np.linalg.norm(vSmall)

        errMain = np.abs(vOldMain - vMainNorm)
        errSmall = np.abs(vOldSmall - vSmallNorm)

        kMain = max(1, int(k*len(errMain)))
        kSmall = max(1, int(k*len(errSmall)))

        # Find top k errors
        topMainIndices = np.argsort(errMain)[-kMain:]
        topSmallIndices = np.argsort(errSmall)[-kSmall:]

        # Create cpies 
        vMainNewNorm = np.copy(vMainNorm)
        vSmallNewNorm = np.copy(vSmallNorm)

        if updateVisualizationFLAG:
            if im1 is not None:
                updateVisualization(vMainNorm, vOldMain, mainNB, im1, cbar1, smallNB=None)
            if im2 is not None:
                updateVisualization(vSmallNorm, vOldSmall, mainNB, im2, cbar2, smallNB)

            if line3 is not None:
                line3.set_ydata(Gammas[:mainN])
                line4.set_ydata(Gammas[nsm: nsm + smallN])
                ax[2].relim()
                ax[2].autoscale_view()


        errMain = np.linalg.norm(vOldMain - vMainNorm)
        errSmall = np.linalg.norm(vOldSmall - vSmallNorm)

        err = errMain + errSmall
        vMainNewNorm[topMainIndices] = (1 - weight) * vMainNorm[topMainIndices] + weight * vOldMain[topMainIndices] 
        vSmallNewNorm[topSmallIndices] = (1 - weight) * vSmallNorm[topSmallIndices] + weight * vOldSmall[topSmallIndices] 

        vMainNew = vMainNewNorm * np.linalg.norm(vMain)
        vSmallNew = vSmallNewNorm * np.linalg.norm(vSmall)

        vMainOld = vMainNorm
        vSmallOld = vSmallNorm

        return np.concatenate((vMainNew, vSmallNew)), err, vMainOld, vSmallOld, im1, im2, line3, line4
    else:
        vMain = v.copy()
        vNorm = vMain/np.linalg.norm(vMain)
        if im1 is not None:
            updateVisualization(vNorm, vOldMain, mainNB, im1, cbar1, smallNB=None)
        err = np.linalg.norm(vOldMain - vNorm)
        
        vMainNew = (1 - weight) * vMain + weight * vOldMain * np.linalg.norm(vMain)
        vMainOld = vNorm

        return vMainNew, err, vMainOld, None, im1, None, None, None

def bisectingMethod(createdMoment, torque, upperBound, lowerBound, RPM_small):
    torque = abs(torque)
    createdMoment = abs(createdMoment)
    if createdMoment > torque:
        upperBound = RPM_small
        midpoint = (upperBound + lowerBound)/2
        RPM_small = midpoint
    if createdMoment < torque:
        lowerBound = RPM_small
        midpoint = (upperBound + lowerBound)/2
        RPM_small = midpoint

    return RPM_small, upperBound, lowerBound



def main(RPM_MAIN, RPM_SMALL, config=None, savePath=None):
    if config is None:
        # Load base config 
        baseConfig = loadConfig(PATH)
        # Generate test configurations
        configFiles = generate_flexible_configs(baseConfig, VARIABLE_SPACE, case=TITLE)
        #
    else:
        configFiles = [config]

    for config in configFiles:

        if FLAGS["just_display"]:
            # Define the drone
            drone = defineDrone(config, main_RPM=RPM_MAIN, small_RPM=RPM_SMALL)
            # Display the drone
            drone.display(color_main='gray', color_small='green', extra_points=None, extra_lines=None)
            break


        errVel, iterVel = 1, 0
        while errVel > ERR_VEL and iterVel < IT_MAX: 
            drone = defineDrone(config, main_RPM=RPM_MAIN, small_RPM=RPM_SMALL)
            FLAGS["helicopter"] = drone.helicopter

            if iterVel == 0:
                # Initialize the axial velocities
                vOldMain, vOldSmall, mainNB, nsm, smallNB, mainN, smallN= initializeVaxial(drone, FLAGS["helicopter"])
                # initialize convergence plot
                if FLAGS['display_convergence']:
                    plt.close()
                    fig, ax = plt.subplots(1,3, figsize=(12, 6))

                    ax[0].set_title('Velocity Error Main Propeller')
                    im1 = ax[0].imshow(vOldMain.reshape(mainNB, -1), cmap='viridis')
                    cbar1 = plt.colorbar(im1, ax=ax[0])
                    if smallNB is not None:
                        ax[1].set_title('Velocity Error Small Propeller')
                        im2 = ax[1].imshow(vOldSmall.reshape(mainNB*smallNB, -1), cmap='viridis')
                        cbar2 = plt.colorbar(im2, ax=ax[1])
                        (line3,) = ax[2].plot(np.linspace(0,1, mainN), label='Gammas', marker='o')
                        (line4,) = ax[2].plot(vOldMain[:smallN], label='Small Gammas', marker='x')
                        ax[2].set_title('Gamma Distribution')
                        ax[2].legend()
                    else:
                        im2 = None
                        cbar2 = None
                        line3 = None
                        line4 = None
                else:
                    im1 = None
                    cbar1 = None
                    im2 = None
                    cbar2 = None
                    line3 = None
                    line4 = None
                    ax = None

            # Solve the LL problem 
            Gammas, _, createdMoment, torque, thrust, power_required, _,_, v, STALL_FACTOR = solve(drone, case=f'{config}', updateConfig=True, save=True, savePath=savePath)

            if STALL_FACTOR[2] > STALL_TRESHOLD:
                print('WARNING: The tip small propeller is stalling, reduce RPM_SMALL')
                break
            if STALL_FACTOR[3] > STALL_TRESHOLD:
                print('WARNING: The root small propeller is stalling, increase RPM_SMALL')
                break
            # compute velocity error
            v, errVel, vOldMain, vOldSmall, im1, im2, line3, line4 = computeVelocityError(v, 
                                                                    vOldMain, 
                                                                    vOldSmall, 
                                                                    FLAGS['helicopter'], 
                                                                    nsm, 
                                                                    mainNB, 
                                                                    smallNB,
                                                                    WEIGHT_VEL, 
                                                                    im1, cbar1,
                                                                    im2, cbar2, line3, line4, 
                                                                    Gammas, mainN=mainN, smallN=smallN, ax=ax, updateVisualizationFLAG=FLAGS['display_convergence'])
            np.savetxt('./auxx/v_axial.txt', v)
            iterVel += 1
            print(f'Iteration: {iterVel}, Error: {errVel}, weight: {WEIGHT_VEL}')
        
        os.remove('./auxx/v_axial.txt')

        return thrust, torque, power_required, createdMoment, STALL_FACTOR, iterVel


def nested_optimization(config, x_0, bounds, results=False, savePlots=False, saveInnerLog=None):   
    """
    Hardoced optimization of RPM_MAIN and RPM_SMALL using double bisection and the STALL check. 

    """
    # Initial guess + initialbounds 
    RPM_MAIN, RPM_SMALL = x_0
    lowerBoundMain, upperBoundMain = bounds[0]
    lowerBoundSmall, upperBoundSmall = bounds[1]

    # define errors
    err_moment = 1
    err_thrust = 1

    print(saveInnerLog)

    # Create log file
    if saveInnerLog is not None:
        log_file = saveInnerLog + "/trim_log.csv"
        print('Im here')
    with open(log_file, "w") as f:
            f.write("RPM_MAIN,RPM_SMALL,thrust,torque,power_required,createdMoment,STALL_FACTOR,RPM_MAIN_bound,RPM_SMALL_bound, iterVel\n")

    # optimize for equilibrium of thrust
    iter_T = 0
    IT_MAX_T = 10
    while err_thrust > ERR_THRUST and iter_T < IT_MAX_T:
        # optimize for equilibrium of moments
        iter = 0
        IT_MAX_M = 10
        err_moment = 1
        lowerBoundSmall, upperBoundSmall = bounds[1]
        while err_moment > ERR_MOMENT and iter < IT_MAX_M:
            
            # Run the case and check if tip-propeller stalls 

            thrust, torque, power_required, createdMoment, STALL_FACTOR, iterVel = main(RPM_MAIN, RPM_SMALL, config=config, savePath=saveInnerLog)

            with open(log_file, "a") as f:
                f.write(f"{RPM_MAIN},{RPM_SMALL},{thrust},{torque},{power_required},{createdMoment},{STALL_FACTOR},{(lowerBoundMain, upperBoundMain)},{(lowerBoundSmall, upperBoundSmall)},{iterVel}\n")
            
            if STALL_FACTOR[2] > STALL_TRESHOLD :
                if createdMoment < torque:
                    # log warning
                    with open(log_file, "a") as f:
                        f.write('WARNING: The tip small propeller is stalling, reduce RPM_SMALL\n')
                    break
                # The tip rotor experiences too high AoA 
                upperBoundSmall = RPM_SMALL
                midpoint = (upperBoundSmall + lowerBoundSmall)/2
                RPM_SMALL = midpoint 
                thrust, torque, power_required, createdMoment, STALL_FACTOR, iterVel = main(RPM_MAIN, RPM_SMALL, config=config, savePath=saveInnerLog)

                with open(log_file, "a") as f:
                    f.write(f"{RPM_MAIN},{RPM_SMALL},{thrust},{torque},{power_required},{createdMoment},{STALL_FACTOR},{(lowerBoundMain, upperBoundMain)},{(lowerBoundSmall, upperBoundSmall)},{iterVel}\n")
            
            
            if STALL_FACTOR[3] > STALL_TRESHOLD :
                if createdMoment > torque:
                    break
                # The tip rotor experiences too low AoA
                lowerBoundSmall = RPM_SMALL
                midpoint = (upperBoundSmall + lowerBoundSmall)/2
                RPM_SMALL = midpoint
                thrust, torque, power_required, createdMoment, STALL_FACTOR, iterVel = main(RPM_MAIN, RPM_SMALL, config=config, savePath=saveInnerLog)

                with open(log_file, "a") as f:
                    f.write(f"{RPM_MAIN},{RPM_SMALL},{thrust},{torque},{power_required},{createdMoment},{STALL_FACTOR},{(lowerBoundMain, upperBoundMain)},{(lowerBoundSmall, upperBoundSmall)}, {iterVel}\n")
            
            if torque < 0:
                torque = 0
            err_moment = abs(createdMoment - torque)
            RPM_SMALL, upperBoundSmall, lowerBoundSmall = bisectingMethod(createdMoment, torque, upperBoundSmall, lowerBoundSmall, RPM_SMALL)
            iter += 1
        if err_moment > ERR_MOMENT and createdMoment < torque:
            print('WARNING: createdMoment is less than torque, unfeasible solution')
        


        with open(log_file, "a") as f:
                f.write(f"{RPM_MAIN},{RPM_SMALL},{thrust},{torque},{power_required},{createdMoment},{STALL_FACTOR},{(lowerBoundMain, upperBoundMain)},{(lowerBoundSmall, upperBoundSmall)}, {iterVel}\n")
                # add empty line to separate iterations
                f.write("\n")
        # optimize for thrust equilibrium
        # check if main-rotor stalls 
        if STALL_FACTOR[0] > STALL_TRESHOLD :
            # The tip rotor experiences too high AoA 
            upperBoundMain = RPM_MAIN
            midpoint = (upperBoundMain + lowerBoundMain)/2
            RPM_MAIN = midpoint 
            thrust, torque, power_required, createdMoment, STALL_FACTOR, iterVel = main(RPM_MAIN, RPM_SMALL, config=config, savePath=saveInnerLog)

            with open(log_file, "a") as f:
                f.write(f"{RPM_MAIN},{RPM_SMALL},{thrust},{torque},{power_required},{createdMoment},{STALL_FACTOR},{(lowerBoundMain, upperBoundMain)},{(lowerBoundSmall, upperBoundSmall)},{iterVel}\n")
        
        if STALL_FACTOR[1] > STALL_TRESHOLD :
            # The tip rotor experiences too low AoA
            lowerBoundMain = RPM_MAIN
            midpoint = (upperBoundMain + lowerBoundMain)/2
            RPM_MAIN = midpoint
            thrust, torque, power_required, createdMoment, STALL_FACTOR, iterVel = main(RPM_MAIN, RPM_SMALL, config=config, savePath=saveInnerLog)

            with open(log_file, "a") as f:
                f.write(f"{RPM_MAIN},{RPM_SMALL},{thrust},{torque},{power_required},{createdMoment},{STALL_FACTOR},{(lowerBoundMain, upperBoundMain)},{(lowerBoundSmall, upperBoundSmall)},{iterVel}\n")
        
        err_thrust = abs(thrust - MTOW)
        RPM_MAIN, upperBoundMain, lowerBoundMain = bisectingMethod(thrust, MTOW, upperBoundMain,  lowerBoundMain,  RPM_MAIN)
        iter_T += 1
        
    if savePlots:
        if FLAGS['display_convergence']:
            plt.ioff()
        myPlt.plot([f'{saveInnerLog}/_res.csv'], show=False, title=TITLE, helicopter=FLAGS["helicopter"], QBlade=False, path=saveInnerLog)
        
    return RPM_MAIN, RPM_SMALL, thrust, torque, power_required, createdMoment, STALL_FACTOR

if __name__ == "__main__":
    start_time = time.time()
    x_0 = [400, 9500]  # Initial guess for RPM_MAIN and RPM_SMALL
    bounds = [(300, 500), (7000, 12000)]  # Bounds for RPM_MAIN and RPM_SMALL
    nested_optimization(x_0=x_0, bounds=bounds, config=None, results=False, savePlots=False, saveInnerLog='./results/one_shot', )
    end_time = time.time()
    print(f"Execution time: {end_time - start_time:.2f} seconds")