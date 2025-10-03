from solver_base import BaseSolver
import numpy as np 
from geometry import defineDrone
import xfoilUtil as xf
from matplotlib import pyplot as plt
import json


class DroneSolver(BaseSolver):
    def __init__(self, aircraft) -> None:
        super().__init__(aircraft)

    def compute_number_of_points(self):
        """
        Compute the number of points for the drone based on its geometry.

        """
        main_NB = self.aircraft.main_prop.NB
        main_n = self.aircraft.main_prop.n
        npM = main_NB * (main_n-1)

        small_NB = self.aircraft.small_props[0].NB
        small_n = self.aircraft.small_props[0].n
        npS = small_NB*(small_n-1) # number of collocation points for small prop
        collocN = (npM + npS*main_NB) # total number of collocation points

        self.npM = npM
        self.npS = npS
        self.collocN = collocN
        self.main_NB = main_NB
        self.small_NB = small_NB
        self.main_n = main_n
        self.small_n = small_n
    
    def compute_collocation_points(self):
        main_NB = self.main_NB
        main_n = self.main_n
        small_NB = self.small_NB
        small_n = self.small_n
        npS = self.npS

        mainCollocPoints = np.zeros((self.npM, 3))
        main_colloc = np.array(self.aircraft.main_prop.collocationPoints)
        for i in range(main_NB):
            mainCollocPoints[i * (main_n - 1):(i + 1) * (main_n - 1)] = main_colloc[i].T
        for i in range(main_NB):
            mainCollocPoints[i * (main_n - 1):(i + 1) * (main_n - 1)] = main_colloc[i].T
            smallCollocPoints = np.zeros((npS*main_NB, 3))
            for i in range(main_NB):
                small_colloc = np.array(self.aircraft.small_props[i].collocationPoints)
                for j in range(small_NB):
                    smallCollocPoints[i * small_NB * (small_n - 1) + j * (small_n - 1): i * small_NB * (small_n - 1) + (j + 1) * (small_n - 1)] = small_colloc[j].T
            total_colloc_points = np.concatenate((mainCollocPoints, smallCollocPoints))
            return total_colloc_points

    def compute_azimuth_and_origin(self):
        collocN = self.collocN
        npM = self.npM
        npS = self.npS
        main_NB = self.main_NB
        drone = self.aircraft

        n_azimuth = np.zeros((collocN, 3))
        n_origin = np.zeros((collocN, 3))

        n_azimuth[:npM] = np.tile(drone.main_prop.azimuth, (npM, 1))
        n_origin[:npM] = np.array([drone.main_prop.origin.x, drone.main_prop.origin.y, drone.main_prop.origin.z])

        for i in range(main_NB):
            n_local_prop = drone.small_props[i].azimuth
            n_azimuth[npM + i * npS: npM + (i + 1) * npS] = np.tile(n_local_prop, (npS, 1))
            n_origin[npM + i * npS: npM + (i + 1) * npS] = np.array([drone.small_props[i].origin.x, 
                                                                    drone.small_props[i].origin.y, 
                                                                    drone.small_props[i].origin.z])
        self.n_azimuth = n_azimuth
        self.n_origin = n_origin

    def compute_twist(self):
        collocN = self.collocN
        npM = self.npM
        drone = self.aircraft
        twist = np.zeros((collocN, 1))
        twist[:npM, 0] = drone.main_prop.pitch
        twist[:npM, 0] = drone.main_prop.pitch
        self.twist = twist

    def compute_omega(self):
        collocN = self.collocN
        npM = self.npM
        drone = self.aircraft
        omega = np.zeros((collocN, 1))
        omega[:npM, 0] = drone.main_prop.RPM*2*np.pi/60
        omega[npM:] = drone.small_props[0].RPM*2*np.pi/60
        self.omega = omega

    def compute_chords(self):
        collocN = self.collocN
        npM = self.npM
        main_NB = self.main_NB
        main_n = self.main_n
        small_NB = self.small_NB
        drone = self.aircraft

        chords = np.zeros((collocN, 1)) 
        ch_mid = 0.5*(drone.main_prop.chord[1:main_n] + drone.main_prop.chord[:main_n-1]) # Chords corresponding to the control points of the main prop
        chords[:npM, 0] = np.tile(ch_mid, (main_NB))
        chords_small = drone.small_props[0].chord
        chords_small = 0.5*(chords_small[1:] + chords_small[:-1])
        chords[npM: , 0] = np.tile(chords_small, (main_NB*small_NB))
        self.chords = chords

    def set_airfoil_array(self):
        collocN = self.collocN
        npM = self.npM
        main_airfoil = self.aircraft.main_prop.airfoil
        small_airfoil = self.aircraft.small_airfoil

        airfoil_array = np.array([main_airfoil]*npM + [small_airfoil]*(collocN - npM))
        self.airfoil_array = airfoil_array

    def compute_v_rotational(self):
        npM = self.npM
        npS = self.npS
        total_colloc_points = self.total_colloc_points
        n_azimuth = self.n_azimuth
        n_origin = self.n_origin
        Omega = self.omega
        drone = self.aircraft
        main_NB = self.main_NB

        v_rotational_small = np.zeros((npS*main_NB, 3))
        azimuthVector = drone.main_prop.azimuth
        v_rotational_main = np.cross(total_colloc_points[:npM], azimuthVector*Omega[0])

        v_rotational_small = np.cross(total_colloc_points[npM:] - n_origin[npM:], n_azimuth[npM:]*Omega[npM:]) + np.cross(total_colloc_points[npM:], azimuthVector*Omega[0]) 

        v_rotational = np.concatenate( (v_rotational_main, v_rotational_small))
        self.v_rotational = v_rotational

    def compute_r_steps(self):
        drone = self.aircraft
        main_NB = self.main_NB
        small_NB = self.small_NB
        r_main =  drone.main_prop.r
        r_main = (r_main[1:] - r_main[:-1])
        r_main = np.tile(r_main, (main_NB))

        r_small = drone.small_props[0].r
        r_small = (r_small[1:] - r_small[:-1])
        r_small= np.tile(r_small, (main_NB*small_NB, 1))
        r_steps = np.concatenate((r_main, r_small), axis=None)

        self.r_steps = r_steps

    def compute_r(self):
        drone = self.aircraft
        main_NB = self.main_NB
        small_NB = self.small_NB
        r_main =  drone.main_prop.r
        r_main = 0.5*(r_main[1:] + r_main[:-1])
        r_main = np.tile(r_main, (main_NB))

        r_small = drone.small_props[0].r
        r_small = 0.5*(r_small[1:]+r_small[:-1])
        r_small= np.tile(r_small, (main_NB*small_NB, 1))
        r = np.concatenate((r_main, r_small), axis=None)
        self.r = r

    def compute_forces(self):
        # F axial 
        f_axial = self.lift * np.cos(self.inflowangle.flatten()) - self.drag * np.sin(self.inflowangle.flatten())
        # F tangential
        f_tan = self.lift * np.sin(self.inflowangle.flatten()) + self.drag * np.cos(self.inflowangle.flatten())
        # Thrust main 
        thrust_main = np.sum(f_axial[:self.npM])
        # Torque main
        torque_main = np.sum(f_tan[:self.npM]*self.r[:self.npM])
        # Induced power
        p_induced = (-self.v_axial[:self.npM].flatten() * f_axial[:self.npM].flatten()).sum()
        # Profile power
        p_profile = np.sum(0.5*1.225*(abs(self.omega[:self.npM]).flatten()*self.r[:self.npM].flatten())**3 * self.Cd0[:self.npM].flatten()*self.r_steps[:self.npM].flatten()*self.chords[:self.npM].flatten())
        # Total power
        p_total = p_induced + p_profile
        # Ideal power
        p_ideal = thrust_main * np.sqrt(abs(thrust_main)/(2*(self.aircraft.main_prop.diameter**2)*np.pi*1.225/4))
        # Power loading 
        power_loading = thrust_main/p_total
        # Figure of merit
        figure_of_merit = p_ideal/p_total
        # thrust_small
        thrust_small = np.sum(f_axial[self.npM:])
        # torque_small
        torque_small = np.sum(f_tan[self.npM:]*self.r_steps[self.npM:])
        # created moment 
        moment_created = thrust_small*self.aircraft.main_prop.diameter/2
        # power required 
        power_required = torque_small*self.aircraft.small_props[0].RPM*2*np.pi/60*4

        #raise NotImplementedError('Add small prop powers and moments')

        self.f_axial = f_axial
        self.f_tan = f_tan
        self.thrust_main = thrust_main
        self.torque_main = torque_main
        self.p_induced = p_induced
        self.p_profile = p_profile
        self.p_total = p_total
        self.p_ideal = p_ideal
        self.power_loading = power_loading
        self.figure_of_merit = figure_of_merit
        
        self.thrust_small = thrust_small
        self.torque_small = torque_small
        self.moment_created = moment_created
        self.power_required = power_required
    
    def save_results(self, path):
        header_names = [
            "r", "v_axial", "v_tangential", "inflowangle", "alpha",
            "f_axial", "f_tan", "gammas", "Cl", "Cd", "chords", "twist"
        ]

        # build structured array
        data = np.core.records.fromarrays([
            self.r.flatten(),
            self.v_axial.flatten(),
            self.v_tangential.flatten(),
            self.inflowangle.flatten(),
            self.alpha.flatten(),
            self.f_axial.flatten(),
            self.f_tan.flatten(),
            self.gammas.flatten(),
            self.Cl.flatten(),
            self.Cd.flatten(),
            self.chords.flatten(),
            self.twist.flatten()
        ], names=",".join(header_names))

        np.savez_compressed(f"{path}/_res.npz", data=data)
        print(f"Saved structured results to {path}/_res.npz")

        # Store performance metrics 
        performance = {
            "thrust_main": self.thrust_main,
            "torque_main": self.torque_main,
            "p_induced": self.p_induced,
            "p_profile": self.p_profile,
            "p_total": self.p_total,
            "p_ideal": self.p_ideal,
            "power_loading": self.power_loading,
            "figure_of_merit": self.figure_of_merit
        }

        with open(f"{path}/performance.json", "w") as f:
            json.dump(performance, f, indent=4)
        #print(f'Saved performance metrics in {path}')

    def plot(self):
        data = np.load('_res.npz')
        data = data['data']
        r = data['r']
        alpha = data['alpha']
        main_n = self.main_n
        small_n = self.small_n
        plt.plot(r[:main_n-1], alpha[:main_n-1], label='Main prop')
        plt.show()

# if __name__ == "__main__":
#     drone = defineDrone('./base.json',main_RPM=400, small_RPM=8000)
#     solver = DroneSolver(drone)
#     solver.solve()
#     solver.plot()