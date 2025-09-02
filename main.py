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
    "run_optimization": True,
    "thrust_optimization": False,
    "helicopter": False,
    "rpm_optimization": False, 
    "display_convergence": True
}

# --- Inputs ---
PATH = 'configs/base.json'
MTOW = 60  # Max Take-Off Weight in N

VARIABLE_SPACE = {
    #"small_propellers.diameter":[0.14,0.15,0.16],  # Main pro,peller angle
    "small_propellers.pitch_root":[85],  # Reynolds number

}

ERR_VEL  = 1e-3
ERR_MOMENT = 1e-1
ERR_THRUST = 1e-1
IT_MAX = 15
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

def main(config=None):
    RPM_MAIN = 480  # RPM of the main propeller
    RPM_SMALL = 9000  # RPM of the small propeller
    LOWER = 5000
    if config is None:
        # Load base config 
        baseConfig = loadConfig(PATH)
        # Generate test configurations
        configFiles = generate_flexible_configs(baseConfig, VARIABLE_SPACE, case=TITLE)
        #
    else:
        configFiles = [config]

    for config in configFiles:
        errThrust, iterThrust = 1, 0
        upperBoundMain, lowerBoundMain = 480, 350

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
                    Gammas, _, createdMoment, torque, thrust, power_required, _,_, v, STALL_FACTOR = solve(drone, case=f'{config}', updateConfig=True)

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
                    print(f'Iteration: {iterVel}, Error: {errVel}, weight: {WEIGHT_VEL}, Stall Factor: {STALL_FACTOR:.4f}')
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
                        print(f'Iteration: {iterMoment}, Moment Error: {errMoment:.2f}, RPM_small: {RPM_SMALL:.1f}, RPM_main: {RPM_MAIN:.1f}, Stall Factor: {STALL_FACTOR:.4f}')
                    else:
                        errMoment = 0
                iterMoment += 1

            # exit outer while loop if thrust is not taken into account
            if not FLAGS["thrust_optimization"]:
                errThrust = 0
            else:
                errThrust = np.abs(thrust - MTOW)
                RPM_MAIN, upperBoundMain, lowerBoundMain = bisectingMethod(thrust, MTOW, upperBoundMain,  lowerBoundMain,  RPM_MAIN)

                ###############
                # Guess RPM_SMALL based on the new RPM_MAIN
                if not FLAGS['helicopter']:
                    guessed_Ct_small = 0.4  # initial guess for small propeller thrust coefficient
                    thrust_required = abs(torque) / (0.5*drone.main_prop.diameter) / drone.main_prop.NB  # thrust required from each small propeller to counteract the torque

                    n = np.sqrt(thrust_required / (guessed_Ct_small * 1.225 * drone.small_props[0].diameter**4))
                    LOWER = n * 60
                    guessed_Ct_small = 0.05
                    n = np.sqrt(thrust_required / (guessed_Ct_small * 1.225 * drone.small_props[0].diameter**4))
                    RPM_SMALL = n * 60
                    if RPM_SMALL > 25_000:
                    #     print('WARNING: RPM_SMALL is too high, setting to 25,000 RPM')
                    #     decision = input('Do you want to continue? (y/n): ')
                    #     if decision.lower() != 'y':
                    #         print('Exiting...')
                    #         exit()
                        RPM_SMALL = 25_000

                    print('Lower bound for small propeller RPM', LOWER)
                    print('Guessed RPM small', RPM_SMALL)
                    # k = 1.15  # empirical factor to account for non-idealities
                    # Cd0 = 0.01  # profile drag coefficient, typical value for small rotors
                    # nrev_main = RPM_MAIN / 60
                    # Ct_main = thrust/ (drone.main_prop.rho * nrev_main**2 * drone.main_prop.diameter**4)
                    # Cp = k* Ct_main**(3/2) / np.sqrt(2) + drone.main_prop.sigma * Cd0 / 8
                    # P 
                    # print('sigma',drone.main_prop.sigma)

                ##############

            print(f'Iteration: {iterThrust}, Thrust Error: {errThrust:.2f}, RPM_main: {RPM_MAIN:.1f}, RPM_small: {RPM_SMALL:.1f}')
            iterThrust += 1
        
        if FLAGS["save_results"]:
            _, FM, created_moment, Torque, Thrust, power_required, _,_, _,_= solve(drone, case=f'{config}', updateConfig=False, save=True)
            #u, v, w = computeVelocityField(plane='XY', shift=0.0, discretization=100, plotting=True)

        os.remove('./auxx/v_axial.txt')
    if FLAGS["plot_results"]:
        if FLAGS['display_convergence']:
            plt.ioff()  # Turn off interactive mode
        for i in range(len(configFiles)):
            configFiles[i] = configFiles[i].replace('.json', '')
        myPlt.plot(configFiles, show = True, title=TITLE, helicopter=FLAGS['helicopter'], QBlade=False)
        #drone.display(color_main='gray', color_small='green', extra_points=None, extra_lines=None)

    # clean up, delete the auxx files
    #os.remove('./auxx/v_axial.txt')
    if FLAGS["rpm_optimization"]:
        return Thrust, power_required, RPM_MAIN, RPM_SMALL, STALL_FACTOR

if __name__ == "__main__":
    start_time = time.time()
    main()
    end_time = time.time()
    print(f"Execution time: {end_time - start_time:.2f} seconds")