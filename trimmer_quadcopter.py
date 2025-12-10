import numpy as np 
import os
from typing import Protocol, Any, Dict, Tuple, Optional
from strategies import QuadcopterStrategy

class QuadcopterTrimmer:
    def __init__(self, strategy, mtow: float,
                 vel_tol: float = 1e-4, vel_itmax: int = 10, vel_blend: float = 0.7,
                 thrust_tol: float = 1e-2, thrust_itmax: int = 10, output_dir='.'):
        
        self.strategy = strategy
        self.mtow = mtow
        self.vel_tol = vel_tol
        self.vel_itmax = vel_itmax
        self.vel_blend = vel_blend
        self.thrust_tol = thrust_tol
        self.thrust_itmax = thrust_itmax
        self.output_dir = output_dir

    def trim_velocity(self,config, main_RPM):
        """
        Trim wake
        """
        errVel, iter = 1, 0
        weight = self.vel_blend

        while errVel > self.vel_tol and iter < self.vel_itmax:
            aircraft = self.strategy.build_aircraft(config, main_RPM)
            solver = self.strategy.build_solver(aircraft, output_dir=self.output_dir)
            solver.solve() 
            if iter == 0:
                v_main_old = solver.v_axial
                iter += 1
                continue
            v_main = solver.v_axial
            v_norm = v_main / np.linalg.norm(v_main)
            errVel = np.linalg.norm(v_main_old - v_norm)
            
            v_main_new = (1 - weight) * v_main + weight * v_main_old * np.linalg.norm(v_main)
            v_main_old = v_norm
  
            np.savetxt('./auxx/v_axial.txt', v_main_new)
            
            iter += 1
            print('Velocity error:', errVel)
        return solver, iter, errVel
    
    def trim_thrust(self, config, main_RPM, target_thrust, bounds):
        """
        Trim main rotor thrust
        """
        errThrust, iter = 1, 0
        lo, hi  = bounds
        while errThrust > self.thrust_tol and iter < self.thrust_itmax:
            solver, _, _ = self.trim_velocity(config, main_RPM=main_RPM)
            thrust = solver.thrust
            if solver.STALL_HIGH > 0.05:
                print("Warning: High stall detected, decreasing RPM")
                hi = main_RPM
                if thrust < target_thrust:
                    os.remove('./auxx/v_axial.txt')
                    return solver, iter, errThrust, main_RPM
            if solver.STALL_LOW > 0.05:
                print("Warning: Low stall detected, increasing RPM")
                lo = main_RPM
                if thrust > target_thrust:
                    os.remove('./auxx/v_axial.txt')
                    return solver, iter, errThrust, main_RPM
                
            errThrust = np.abs(thrust - target_thrust)
            print(f"Iter {iter}: Thrust {thrust:.2f} N, Target {target_thrust:.2f} N, Error {errThrust:.2f} N")
            if errThrust < self.thrust_tol:
                os.remove('./auxx/v_axial.txt')
                return solver, iter, errThrust, main_RPM
            # Bisection method to adjust RPM
            main_RPM, lo, hi = self._bisect(thrust, target_thrust, lo, hi, main_RPM)
            iter += 1
        # delete the auxx velocity
        os.remove('./auxx/v_axial.txt')
        return solver, iter, errThrust, main_RPM
    
    @staticmethod
    def _bisect(measured: float, target: float,
                lo: float, hi: float, current: float) -> Tuple[float, float, float]:
        if measured > target:
            hi = current
        else:
            lo = current
        return 0.5 * (lo + hi), lo, hi 
    
if __name__ == "__main__":
    path = "/home/glebdoc/PythonProjects/MScSketches/DesignSpace/quad_plot/"
    trimmer  = QuadcopterTrimmer(QuadcopterStrategy(), mtow=60.0, output_dir=path)
    solver, iters, err = trimmer.trim_velocity(
        path+'_.json', main_RPM=4000)
    solver.save_results(path=path)
    solver.plot_self()
    solver.aircraft.display()