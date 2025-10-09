import numpy as np
from typing import Protocol, Any, Dict, Tuple, Optional
from strategies import DroneStrategy
import os
import matplotlib.pyplot as plt

class DroneTrimmer:
    def __init__(self, strategy, mtow: float,
                 vel_tol: float = 1e-4, vel_itmax: int = 30, vel_blend: float = 0.7,
                 thrust_tol: float = 1e-1, thrust_itmax: int = 10,
                 moment_tol: float = 1e-1, moment_itmax: int = 10,
                 has_aux_rotors: bool = True, output_dir: str = "."):
        
        self.strategy = strategy
        self.mtow = mtow
        self.vel_tol = vel_tol
        self.vel_itmax = vel_itmax
        self.vel_blend = vel_blend
        self.thrust_tol = thrust_tol
        self.thrust_itmax = thrust_itmax
        self.moment_tol = moment_tol
        self.moment_itmax = moment_itmax
        self.has_aux_rotors = has_aux_rotors
        self.k = 0.05  
        self.output_dir = output_dir


    def trim_velocity(self,config, main_RPM, small_RPM: Dict | str,):
        """
        Trim wake
        """
        errVel, iter = 1, 0
        weight = self.vel_blend
        while errVel > self.vel_tol and iter < self.vel_itmax:
            aircraft = self.strategy.build_aircraft(config, main_RPM, small_RPM)
            solver = self.strategy.build_solver(aircraft, output_dir=self.output_dir)
            solver.solve()  # your BaseSolver iterates gammas
            if iter == 0:
                v_axial_old = solver.v_axial
                v_main_old = v_axial_old[:solver.npM]
                v_small_old = v_axial_old[solver.npM:]
                iter += 1
                continue
            v_axial = solver.v_axial
            v_main = v_axial[:solver.npM]
            v_small = v_axial[solver.npM:]

            v_main_norm = v_main/np.linalg.norm(v_main)
            v_small_norm = v_small/np.linalg.norm(v_small)

            err_main = np.abs(v_main_old - v_main_norm)
            err_small = np.abs(v_small_old - v_small_norm)

            k_main = max(1, int(self.k*len(err_main)))
            k_small = max(1, int(self.k*len(err_small)))

            topMainIndices = np.argsort(err_main)[-k_main:]
            topSmallIndices = np.argsort(err_small)[-k_small:]

            v_main_new_norm = np.copy(v_main_norm)
            v_small_new_norm = np.copy(v_small_norm)

            err_main = np.linalg.norm(v_main_old - v_main_norm)
            err_small = np.linalg.norm(v_small_old - v_small_norm)

            errVel = err_main + err_small

            v_main_new_norm[topMainIndices] = (1 - weight) * v_main_norm[topMainIndices] + weight * v_main_old[topMainIndices] 
            v_small_new_norm[topSmallIndices] = (1 - weight) * v_small_norm[topSmallIndices] + weight * v_small_old[topSmallIndices] 

            v_main_new = v_main_new_norm * np.linalg.norm(v_main)
            v_small_new = v_small_new_norm * np.linalg.norm(v_small)

            v_main_old = v_main_norm
            v_small_old = v_small_norm

            result = np.concatenate((v_main_new, v_small_new))
            np.savetxt('./auxx/v_axial.txt', result)
            
            iter += 1
            # print('npSmall:', solver.npS, 'npM:', solver.npM, 'Velocity error:', errVel, 'Iter:', iter)
            # plt.plot( solver.gammas[:solver.main_n -1], label='Main rotor')
            # plt.plot( solver.gammas[solver.npM: solver.npM + solver.small_n -1], label='Aux rotor')
            # print('gammas', solver.gammas[solver.npM:])
            # plt.legend()
            # plt.show()
            #solver.plot_self()
            print('Velocity error:', errVel)
        return solver, iter, errVel

    def trim_moment(self,
                    config: Dict | str,
                    rpm_small: float,
                    main_RPM: float,
                    bounds_aux: Tuple[float, float]) -> Tuple[float, Any, list]:

        lo, hi = bounds_aux
        moment_err, iter = 1, 0
        STALL_THRESHLOD: float = 0.15
        while moment_err > self.moment_tol and iter < self.moment_itmax:
            solver, _, _ = self.trim_velocity(config, main_RPM, rpm_small)
            # read generated moment and torque 
            created_moment = abs(solver.moment_created)
            torque_main = abs(solver.torque_main)

            if solver.STALL_SMALL_HIGH > STALL_THRESHLOD:
                print("Warning: High small rotor stall detected (>5% elements). Decreasing small rotor RPM.")
                if created_moment < torque_main:
                    print(f'Created moment: {created_moment}, required torque: {torque_main}')
                    try:
                        os.remove('./auxx/v_axial.txt')
                    except:
                        print('No such file')
                    print('Moment error:', moment_err, '| RPM small:', rpm_small, '| created moment:', created_moment, '| torque main:', torque_main)
                    return rpm_small, solver
                
                hi = rpm_small
                rpm_small, lo, hi = self._bisect(measured=created_moment, target=torque_main,
                                   lo=lo, hi=hi, current=rpm_small)
                solver, _, _ = self.trim_velocity(config, main_RPM, rpm_small)
                # read generated moment and torque 
                created_moment = abs(solver.moment_created)
                torque_main = abs(solver.torque_main)
                
            if solver.STALL_SMALL_LOW > STALL_THRESHLOD:
                print("Warning: Low small rotor stall detected (>5% elements). Increasing small rotor RPM.")
                if created_moment > torque_main:
                    try:
                        os.remove('./auxx/v_axial.txt')
                    except:
                        print('No such file')
                    print('Moment error:', moment_err, '| RPM small:', rpm_small, '| created moment:', created_moment, '| torque main:', torque_main)
                    return rpm_small, solver
                lo = rpm_small
                rpm_small, lo, hi = self._bisect(measured=created_moment, target=torque_main,
                                   lo=lo, hi=hi, current=rpm_small)
                solver, _, _ = self.trim_velocity(config, main_RPM, rpm_small)
                # read generated moment and torque 
                created_moment = abs(solver.moment_created)
                torque_main = abs(solver.torque_main)
            
            moment_err = abs(created_moment - torque_main)
            if moment_err < self.moment_tol:
                print('Moment error:', moment_err, '| RPM small:', rpm_small, '| created moment:', created_moment, '| torque main:', torque_main)
                return rpm_small, solver
            # bisection step
            
            rpm_small, lo, hi = self._bisect(measured=created_moment, target=torque_main,
                                   lo=lo, hi=hi, current=rpm_small)
            iter += 1
            print('Moment error:', moment_err, '| RPM small:', rpm_small, '| created moment:', created_moment, '| torque main:', torque_main)
        try:
            os.remove('./auxx/v_axial.txt')
        except:
            print('No such file')
        return rpm_small, solver
            
    def trim_thrust(self,
                    config: Dict | str,
                    rpm_small: float,
                    main_RPM: float,
                    mtow,
                    bounds_moment: Tuple[float, float],
                    
                    bounds_aux: Tuple[float, float]) -> Tuple[float, Any, list]:
        lo, hi = bounds_aux
        thrust_err, iter = 1, 0
        rpm_small0 = rpm_small
        rpm_main = main_RPM

        STALL_THRESHLOD: float = 0.15,

        while thrust_err > self.thrust_tol and iter < self.thrust_itmax:
            rpm_small, solver = self.trim_moment(config, rpm_small0, rpm_main, bounds_aux=bounds_moment)
            created_thrust = solver.thrust_main
            if solver.STALL_MAIN_HIGH > STALL_THRESHLOD:
                print("Warning: High main rotor stall detected (>5% elements). Decreasing main RPM.")
                if created_thrust < mtow:
                    try:
                        os.remove('./auxx/v_axial.txt')
                    except:
                        print('No such file')
                    return rpm_main, rpm_small, solver
                hi = rpm_main
                rpm_main, lo, hi = self._bisect(measured=created_thrust, target=self.mtow,
                                      lo=lo, hi=hi, current=rpm_main)
                solver, _, _ = self.trim_velocity(config, rpm_main, rpm_small)
                iter += 1
            if solver.STALL_MAIN_LOW > STALL_THRESHLOD:
                print("Warning: Low main rotor stall detected (>5% elements). Increasing main RPM.")
                if created_thrust > mtow:
                    try:
                        os.remove('./auxx/v_axial.txt')
                    except:
                        print('No such file')
                    return rpm_main, rpm_small, solver
                lo = rpm_main
                rpm_main, lo, hi = self._bisect(measured=created_thrust, target=self.mtow,
                                        lo=lo, hi=hi, current=rpm_main)
                solver, _, _ = self.trim_velocity(config, rpm_main, rpm_small)
                iter += 1
            thrust_err = abs(created_thrust - mtow)
            if thrust_err < self.thrust_tol:
                try:
                    os.remove('./auxx/v_axial.txt')
                except:
                    print('No such file')
                return rpm_main, rpm_small, solver
            # bisection step
            print('Bisection step for thrust trim')
            print('Current RPM main:', rpm_main, '| created thrust:', created_thrust, '| target mtow:', mtow)
            rpm_main, lo, hi = self._bisect(measured=created_thrust, target=self.mtow,
                                      lo=lo, hi=hi, current=rpm_main)
            print('Thrust error:', thrust_err, '| RPM main:', rpm_main, '| created thrust:', created_thrust)
            iter += 1
        #os.remove('./auxx/v_axial.txt')
        return rpm_main, rpm_small, solver
    # ---------- Helper: single bisection step ----------
    @staticmethod
    def _bisect(measured: float, target: float,
                lo: float, hi: float, current: float) -> Tuple[float, float, float]:
        if measured > target:
            hi = current
        else:
            lo = current
        return 0.5 * (lo + hi), lo, hi
    
    




if __name__ == "__main__":
    strategy = DroneStrategy()
    

    #path = "/home/glebdoc/PythonProjects/MScSketches/Factorial_trial/drone/DP0/_.json"
    path = "/home/glebdoc/PythonProjects/MScSketches/DesignSpace/real_size_test/_.json"

    trimmer  = DroneTrimmer(strategy, mtow=60.0, output_dir="/home/glebdoc/PythonProjects/MScSketches/DesignSpace/real_size_test")

    #test: velocity trim at fixed RPMs
    # solver, n_it, err = trimmer.trim_velocity( main_RPM=370, small_RPM=7745,
    #     config="configs/base.json"
    # )
    # solver.plot_self()
    # solver.aircraft.display()

    # test: moment trim at fixed main RPM
    rpm_small, solver = trimmer.trim_moment( main_RPM=390,
        config=path,
        rpm_small=8000.0,
        bounds_aux=(7000.0, 18000.0)
    )
    print(solver.thrust_main, solver.power_required, solver.p_total)    

    solver.plot_self()
    solver.aircraft.display()   
    # test: thrust trim
    # _, solver = trimmer.trim_thrust( main_RPM=340,
    #     config="./configs/base.json",
    #     rpm_small=13000.0,
    #     bounds_aux=(300.0, 500.0),
    #     bounds_moment=(5000.0, 15000.0),
    #     mtow=60.0
    # )

    # print("Done. iters:", n_it, "err:", f"{err:.3e}",
    #       "| thrust:", getattr(solver, "thrust_main", None),
    #       "| power:", getattr(solver, "p_total", None))