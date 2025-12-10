from solver_base import BaseSolver
import numpy as np 
import xfoilUtil as xf
from matplotlib import pyplot as plt
from geometry import defineDrone
import json


class HelicopterSolver(BaseSolver):
    def __init__(self, aircraft, output_dir) -> None:
        super().__init__(aircraft)
        self.output_dir = output_dir
        self.main_RPM = None
    def compute_number_of_points(self):
        main_NB = self.aircraft.main_prop.NB
        main_n = self.aircraft.main_prop.n
        npM = main_NB * (main_n-1)
        
        self.main_NB = main_NB
        self.main_n = main_n
        self.npM = npM
        self.collocN = npM
    def compute_collocation_points(self):
        main_NB = self.main_NB
        main_n = self.main_n
        npM = self.npM

        mainCollocPoints = np.zeros((npM, 3))
        main_colloc = np.array(self.aircraft.main_prop.collocationPoints)
        for i in range(main_NB):
            mainCollocPoints[i * (main_n - 1):(i + 1) * (main_n - 1)] = main_colloc[i].T
        return mainCollocPoints
    def compute_azimuth_and_origin(self):
        collocN = self.collocN
        npM = self.npM
        drone = self.aircraft
        n_azimuth = np.zeros((collocN, 3))
        n_origin = np.zeros((collocN, 3))

        n_azimuth[:npM] = np.tile(drone.main_prop.azimuth, (npM, 1))
        n_origin[:npM] = np.array([drone.main_prop.origin.x, drone.main_prop.origin.y, drone.main_prop.origin.z])
        self.n_azimuth = n_azimuth
        self.n_origin = n_origin
    def compute_twist(self):
        collocN = self.collocN
        npM = self.npM
        drone = self.aircraft
        twist = np.zeros((collocN, 1))
        twist[:npM, 0] = drone.main_prop.pitch
        self.twist = twist
    def compute_omega(self):
        drone = self.aircraft
        omega = np.ones((self.collocN, 1))* (drone.main_prop.RPM*2*np.pi/60)
        self.omega = omega
    def compute_chords(self):
        collocN = self.collocN
        npM = self.npM
        main_NB = self.main_NB
        main_n = self.main_n
        drone = self.aircraft

        chords = np.zeros((collocN, 1)) 
        ch_mid = 0.5*(drone.main_prop.chord[1:main_n] + drone.main_prop.chord[:main_n-1]) # Chords corresponding to the control points of the main prop
        chords[:npM, 0] = np.tile(ch_mid, (main_NB))
        self.chords = chords
    
    def compute_v_rotational(self):
        npM = self.npM
        total_colloc_points = self.total_colloc_points
        Omega = self.omega[0]
        drone = self.aircraft
        azimuthVector = drone.main_prop.azimuth
        v_rotational_main = np.cross(total_colloc_points[:npM], azimuthVector*Omega)
        self.v_rotational = v_rotational_main

    def set_airfoil_array(self):
        npM = self.npM
        main_airfoil = self.aircraft.main_prop.airfoil

        airfoil_array = np.array([main_airfoil]*npM)
        self.airfoil_array = airfoil_array

    def compute_r_steps(self):
        drone = self.aircraft
        main_NB = self.main_NB
        r_main =  drone.main_prop.r
        r_main = (r_main[1:] - r_main[:-1])
        r_main = np.tile(r_main, (main_NB))
        self.r_steps = r_main

    def compute_r(self):
        drone = self.aircraft
        main_NB = self.main_NB
        r_main =  drone.main_prop.r
        r_main = 0.5*(r_main[1:] + r_main[:-1])

        r_main = np.tile(r_main, (main_NB))
        self.r = r_main

    def compute_forces(self):
        # F axial 
        f_axial = self.lift * np.cos(self.inflowangle.flatten()) - self.drag * np.sin(self.inflowangle.flatten())
        # F tangential
        f_tan = self.lift * np.sin(self.inflowangle.flatten()) + self.drag * np.cos(self.inflowangle.flatten())
        # Thrust helicopter 
        thrust= np.sum(f_axial)
        # Torque helicopter 
        torque = np.sum(f_tan*self.r)
        # induced power
        p_induced = (-self.v_axial.flatten() * f_axial.flatten()).sum()
        # Profile power 
        p_profile = np.sum(0.5*1.225*(abs(self.omega).flatten()*self.r.flatten())**3 * self.Cd0.flatten()*self.r_steps.flatten()*self.chords.flatten())
        # Total power
        p_total = p_induced + p_profile
        # Ideal power
        p_ideal = thrust * np.sqrt(abs(thrust)/(2*(self.aircraft.main_prop.diameter**2)*np.pi*1.225/4))
        # Power loading 
        power_loading = thrust/p_total
        # Figure of merit
        figure_of_merit = p_ideal/p_total

        # save 
        self.f_axial = f_axial
        self.f_tan = f_tan
        self.thrust = thrust
        self.torque = torque
        self.p_induced = p_induced
        self.p_profile = p_profile
        self.p_total = p_total
        self.p_ideal = p_ideal
        self.power_loading = power_loading
        self.figure_of_merit = figure_of_merit
    
    def check_stall(self):
        self.STALL_HIGH = np.sum(self.alpha>=15)/self.alpha.shape[0]
        self.STALL_LOW = np.sum(self.alpha<=-5)/self.alpha.shape[0]
  
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

        # compute disk area 
        A = np.pi * (self.aircraft.main_prop.diameter / 2) ** 2
        DL = self.thrust / A  # Disk Loading

        # Store performance metrics 
        performance = {
            "thrust": self.thrust,
            "torque": self.torque,
            "p_induced": self.p_induced,
            "p_profile": self.p_profile,
            "p_total": self.p_total,
            "p_ideal": self.p_ideal,
            "power_loading": self.power_loading,
            "figure_of_merit": self.figure_of_merit,
            "STALL_HIGH": self.STALL_HIGH,
            "STALL_LOW": self.STALL_LOW,
            "main_RPM": self.main_RPM,
            "disk_loading": DL
        }

        with open(f"{path}/performance.json", "w") as f:
            json.dump(performance, f, indent=4)

    def normalize(self, r):
        r = r.flatten()
        res = (r - r.min()) / (r.max() - r.min())
        return res
    
    def plot_self(self):

        r            = self.r
        alpha        = self.alpha
        inflowangle  = self.inflowangle * 180/np.pi
        twist        = self.twist
        v_axial      = self.v_axial
        v_tangential = self.v_tangential
        Re           = self.Re
        Cl           = self.Cl
        Cd           = self.Cd
        gamma        = self.gammas  # circulation

        main_n = self.main_n
        # one-blade span of main rotor, like [0,0]
        #r_main = self.normalize(r[:main_n-1])
        r_main = r[:main_n-1]/np.max(r[:main_n-1])

        # derived
        a = 340.0
        V_infty = np.sqrt(v_axial**2 + v_tangential**2)
        M = np.asarray(V_infty) / a

        fig, axs = plt.subplots(3, 3, figsize=(15, 12))

        # Row 0
        axs[0, 0].plot(r_main, alpha[:main_n-1])
        axs[0, 0].set_title('Angle of Attack (Main Rotor)')
        axs[0, 0].set_xlabel('Radius (norm.)'); axs[0, 0].set_ylabel('Alpha (deg)'); axs[0, 0].grid(True)

        axs[0, 1].plot(r_main, inflowangle[:main_n-1])
        axs[0, 1].set_title('Inflow Angle (Main Rotor)')
        axs[0, 1].set_xlabel('Radius (norm.)'); axs[0, 1].set_ylabel('Phi (deg)'); axs[0, 1].grid(True)

        axs[0, 2].plot(r_main, twist[:main_n-1])
        axs[0, 2].set_title('Twist (Main Rotor)')
        axs[0, 2].set_xlabel('Radius (norm.)'); axs[0, 2].set_ylabel('Twist (deg)'); axs[0, 2].grid(True)

        # Row 1
        axs[1, 0].plot(r_main, v_axial[:main_n-1])
        axs[1, 0].set_title('Axial Velocity (Main Rotor)')
        axs[1, 0].set_xlabel('Radius (norm.)'); axs[1, 0].set_ylabel('v_axial (m/s)'); axs[1, 0].grid(True)

        axs[1, 1].plot(r_main, Re[:main_n-1])
        axs[1, 1].set_title('Reynolds Number (Main Rotor)')
        axs[1, 1].set_xlabel('Radius (norm.)'); axs[1, 1].set_ylabel('Re [-]'); axs[1, 1].grid(True)

        axs[1, 2].plot(r_main, M[:main_n-1])
        axs[1, 2].set_title('Mach Number (Main Rotor)')
        axs[1, 2].set_xlabel('Radius (norm.)'); axs[1, 2].set_ylabel('M [-]'); axs[1, 2].grid(True)

        # Row 2 — requested additions
        axs[2, 0].plot(r_main, Cl[:main_n-1])
        axs[2, 0].set_title('Lift Coefficient $C_l$ (Main Rotor)')
        axs[2, 0].set_xlabel('Radius (norm.)'); axs[2, 0].set_ylabel('$C_l$'); axs[2, 0].grid(True)

        axs[2, 1].plot(r_main, Cd[:main_n-1])
        axs[2, 1].set_title('Drag Coefficient $C_d$ (Main Rotor)')
        axs[2, 1].set_xlabel('Radius (norm.)'); axs[2, 1].set_ylabel('$C_d$'); axs[2, 1].grid(True)

        axs[2, 2].plot(r_main, gamma[:main_n-1])
        axs[2, 2].set_title('Circulation $\\Gamma$ (Main Rotor)')
        axs[2, 2].set_xlabel('Radius (norm.)'); axs[2, 2].set_ylabel('$\\Gamma$'); axs[2, 2].grid(True)

        plt.tight_layout()
        plt.show()

    # if __name__ == "__main__":
    #     drone = defineDrone('./base_helicopter.json',main_RPM=400)
    #     solver = HelicopterSolver(drone)
    #     solver.solve()
    #     solver.plot()