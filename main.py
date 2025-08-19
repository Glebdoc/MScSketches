import numpy as np
import pyvista as pv
import os, json
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
    "plot_results": True,
    "save_results": True,
    "run_optimization": False,
    "thrust_optimization": False,
    "helicopter": False,
    "rpm_optimization": False
}

# --- Inputs ---
PATH = 'configs/base.json'
MTOW = 100  # Max Take-Off Weight in N

VARIABLE_SPACE = {
    #"small_propellers.diameter":[0.14,0.15,0.16],  # Main pro,peller angle
    "settings.reynolds":[True],  # Reynolds number

}

ERR_VEL  = 1e-2
ERR_MOMENT = 1e-2
ERR_THRUST = 1e-1
IT_MAX = 50
WEIGHT_VEL = 0.995 # The heigher the more stable
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
    
def computeVelocityError(v, vOldMain, vOldSmall, helicopter, nsm, mainNB, smallNB, weight, im1, cbar1, im2, cbar2, line3, line4, Gammas, k=0.05, mainN=None, smallN=None, ax=None):
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

        # print(f'Top {kMain} errors in main propeller: {topMainIndices}')
        # print(f'Top {kSmall} errors in small propeller: {topSmallIndices}')

        # Create cpies 
        vMainNewNorm = np.copy(vMainNorm)
        vSmallNewNorm = np.copy(vSmallNorm)


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

        # vMainNew[topMainIndices] = (1 - weight) * vMain[topMainIndices] + weight * vOldMain[topMainIndices] * np.linalg.norm(vMain[topMainIndices])
        # vSmallNew[topSmallIndices] = (1 - weight) * vSmall[topSmallIndices] + weight * vOldSmall[topSmallIndices] * np.linalg.norm(vSmall[topSmallIndices])
        vMainNewNorm[topMainIndices] = (1 - weight) * vMainNorm[topMainIndices] + weight * vOldMain[topMainIndices] 
        vSmallNewNorm[topSmallIndices] = (1 - weight) * vSmallNorm[topSmallIndices] + weight * vOldSmall[topSmallIndices] 

        vMainNew = vMainNewNorm * np.linalg.norm(vMain)
        vSmallNew = vSmallNewNorm * np.linalg.norm(vSmall)

        # start = int(mainN * 0.6) 

        # x_tip = np.linspace(0, 1, len(vMainNew[start:mainN]))
        # print(f'Starting smoothing from index 0 to {mainN}')

        # y_start = vMainNew[start-1]
        # y_tip_shifted = vMainNew[start:mainN] - vMainNew[start] + y_start
        # x_shifted = x_tip - x_tip[0] + 1e-6  # Avoid division by zero
        # # Fit a polynomial to the last 30% of the data
        # #p = Polynomial.fit(x_tip, vMainNew[start:mainN], deg=3)
        # p = Polynomial.fit(x_tip, y_tip_shifted, deg=2)
        # smoothed_values = p(x_tip)
        # # Replace the last 30% of the data with the smoothed values
        # vMainNew[start:mainN] = smoothed_values
        # vMainNew = np.tile(vMainNew[:mainN], mainNB)

        # plt.plot(x_tip, smoothed_values, label='Smoothed', color='red')
        
        
    
        # vMainNew = moving_average(vMainNew.copy(), w=3)
        # vSmallNew = moving_average(vSmallNew.copy(), w=3)
        # line3.set_ydata(vMainNew[:mainN])
        # vMainNew = low_pass_filter(vMainNew.copy(), cutoff_ratio=0.4)

        # vSmallNew = low_pass_filter(vSmallNew.copy(), cutoff_ratio=0.1)

        #vMainNew

        # filter results 
        # vMainNew[1:] = (vMainNew[:-1] + 2*vMainNew[1:]  + vMainNew[:-1])/4
        # print('vmainNew', vMainNew.shape)


        # vMainNew = (1 - weight) * vMain + weight * vOldMain * np.linalg.norm(vMain)
        # vSmallNew = (1 - weight) * vSmall + weight * vOldSmall * np.linalg.norm(vSmall)

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

def main():
    RPM_MAIN = 400  # RPM of the main propeller
    RPM_SMALL = 7000  # RPM of the small propeller
    LOWER = 5000
    # Load base config 
    baseConfig = loadConfig(PATH)
    # Generate test configurations
    configFiles = generate_flexible_configs(baseConfig, VARIABLE_SPACE, case=TITLE)
    #

    for config in configFiles:
        errThrust, iterThrust = 1, 0
        upperBoundMain, lowerBoundMain = 800, 250

        if FLAGS["just_display"]:
            # Define the drone
            drone = defineDrone(config, main_RPM=RPM_MAIN, small_RPM=RPM_SMALL)
            # Display the drone
            drone.display(color_main='gray', color_small='green', extra_points=None, extra_lines=None)
            break
        while errThrust > ERR_THRUST and iterThrust < IT_MAX:

            errMoment, iterMoment = 1, 0
            upperBoundSmall, lowerBoundSmall = RPM_SMALL, LOWER

            while errMoment > ERR_MOMENT and iterMoment < IT_MAX:
              
                errVel, iterVel = 1, 0
                while errVel > ERR_VEL and iterVel < IT_MAX: 
                    drone = defineDrone(config, main_RPM=RPM_MAIN, small_RPM=RPM_SMALL)
                    FLAGS["helicopter"] = drone.helicopter

                    if iterVel == 0:
                        # Initialize the axial velocities
                        vOldMain, vOldSmall, mainNB, nsm, smallNB, mainN, smallN= initializeVaxial(drone, FLAGS["helicopter"])
                        # initialize convergence plot
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

                    # Solve the LL problem 
                    Gammas, _, createdMoment, torque, thrust, power_required, _,_, v = solve(drone, case=f'{config}', updateConfig=True)

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
                                                                           Gammas, mainN=mainN, smallN=smallN, ax=ax)
                    np.savetxt('./auxx/v_axial.txt', v)
                    iterVel += 1
                    print(f'Iteration: {iterVel}, Error: {errVel}, weight: {WEIGHT_VEL}')
                os.remove('./auxx/v_axial.txt')
                if FLAGS['helicopter']:
                    errMoment = 0
                else:
                    if FLAGS['rpm_optimization']:
                        # compute moment error
                        errMoment = np.abs(abs(createdMoment) - torque)
                        # upadate RPM_small based on the moment error
                        RPM_SMALL, upperBoundSmall, lowerBoundSmall = bisectingMethod(createdMoment, torque, upperBoundSmall, lowerBoundSmall, RPM_SMALL)
                        iterMoment += 1
                        print(f'Iteration: {iterMoment}, Moment Error: {errMoment}, RPM_small: {RPM_SMALL}')
                    else:
                        errMoment = 0
                iterMoment += 1

            # exit outer while loop if thrust is not taken into account
            if not FLAGS["thrust_optimization"]:
                errThrust = 0
            else:
                errThrust = np.abs(thrust - MTOW)
                RPM_MAIN, upperBoundMain, lowerBoundMain = bisectingMethod(thrust, MTOW, upperBoundMain,  lowerBoundMain,  RPM_MAIN)
            print(f'Iteration: {iterThrust}, Thrust Error: {errThrust}, RPM_main: {RPM_MAIN}')
            iterThrust += 1
        
        if FLAGS["save_results"]:
            _, FM, created_moment, Torque, Thrust, power_required, _,_, _= solve(drone, case=f'{config}', updateConfig=False, save=True)
            #u, v, w = computeVelocityField(plane='XY', shift=0.0, discretization=100, plotting=True)

        os.remove('./auxx/v_axial.txt')
    if FLAGS["plot_results"]:
        plt.ioff()  # Turn off interactive mode
        for i in range(len(configFiles)):
            configFiles[i] = configFiles[i].replace('.json', '')
        myPlt.plot(configFiles, show = True, title=TITLE, helicopter=FLAGS['helicopter'], QBlade=False)
        #drone.display(color_main='gray', color_small='green', extra_points=None, extra_lines=None)

    # clean up, delete the auxx files
    #os.remove('./auxx/v_axial.txt')

if __name__ == "__main__":
    start_time = time.time()
    main()
    end_time = time.time()
    print(f"Execution time: {end_time - start_time:.2f} seconds")