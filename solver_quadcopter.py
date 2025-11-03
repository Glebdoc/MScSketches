from solver_base import BaseSolver
import numpy as np 
from geometry import defineDrone
import xfoilUtil as xf
import json
from matplotlib import pyplot as plt


class QuadSolver(BaseSolver):
    def __init__(self, aircraft, output_dir) -> None:
        super().__init__(aircraft)
        self.output_dir = output_dir
        self.main_RPM = None
    
    def compute_number_of_points(self):
        self.main_NB = self.aircraft.props[0].NB 
        self.main_n = self.aircraft.props[0].n
        self.npM = self.main_NB * (self.main_n - 1)
        self.collocN = self.npM*4 # total number of collocation points for quadcopter

    def compute_collocation_points(self):
        npM = self.npM
        main_NB = self.main_NB
        main_n = self.main_n

        mainCollocPoints = np.zeros((4*npM, 3))
        for j in range(4):
            for i in range(main_NB):
                main_colloc = np.array(self.aircraft.props[j].collocationPoints)
                mainCollocPoints[j*npM + i * (main_n - 1): j*npM + (i + 1) * (main_n - 1)] = main_colloc[i].T
        total_colloc_points = mainCollocPoints
        return total_colloc_points

    def compute_azimuth_and_origin(self):
        collocN = self.collocN
        npM = self.npM
        drone = self.aircraft
        n_azimuth = np.zeros((collocN, 3))
        n_origin = np.zeros((collocN, 3))

        for i in range(4):
            n_azimuth [i*npM:(i+1)*npM] = np.tile(drone.props[0].azimuth, (npM, 1))
            n_origin [i*npM:(i+1)*npM] = np.array([drone.props[i].origin.x, 
                                                   drone.props[i].origin.y, 
                                                   drone.props[i].origin.z])
        self.n_azimuth = n_azimuth
        self.n_origin = n_origin
    def compute_twist(self):
        drone = self.aircraft
        twist = np.tile(drone.props[0].pitch, 4)
        twist = twist.reshape(-1,1)
        self.twist = twist

    def compute_omega(self):
        collocN = self.collocN
        npM = self.npM
        drone = self.aircraft
        omega = np.zeros((collocN, 1))
        for i in range(4):
            omega[i*npM: (i+1)*npM] = drone.props[i].RPM*0.1047
        self.omega = omega
    
    def compute_chords(self):   
        main_NB = self.main_NB
        main_n = self.main_n
        drone = self.aircraft
        chords = drone.props[0].chord[:main_n]
        chords = 0.5*(chords[1:] + chords[:-1])
        chords = np.tile(chords, (main_NB))
        self.chords = np.tile(chords, 4)
    
    def compute_v_rotational(self):
        total_colloc_points = self.total_colloc_points
        n_azimuth = self.n_azimuth
        n_origin = self.n_origin
        Omega = self.omega
        v_rotational = np.cross(total_colloc_points-n_origin, n_azimuth*Omega)
        self.v_rotational =v_rotational
    def set_airfoil_array(self):
        collocN = self.collocN
        main_airfoil = self.aircraft.airfoil

        airfoil_array = np.array([main_airfoil]*collocN)
        self.airfoil_array = airfoil_array
    
    def compute_r_steps(self):
        drone = self.aircraft
        main_NB = self.main_NB
        r_main =  drone.props[0].r
        r_main = (r_main[1:] - r_main[:-1])
        r_main = np.tile(r_main, (main_NB))
        r_steps = np.tile(r_main, 4)
        self.r_steps = r_steps

    def compute_r(self):
        drone = self.aircraft
        main_NB = self.main_NB
        r_main =  drone.props[0].r
        r_main = 0.5*(r_main[1:] + r_main[:-1])
        r_main = np.tile(r_main, (main_NB))
        r = np.tile(r_main, 4)
        print(r)
        self.r = r
        print(r)

    def compute_forces(self):
        #F axial 
        f_axial = self.lift * np.cos(self.inflowangle.flatten()) - self.drag * np.sin(self.inflowangle.flatten())
        # F tangential
        f_tan = self.lift * np.sin(self.inflowangle.flatten()) + self.drag * np.cos(self.inflowangle.flatten())
        # Thrust main 
        thrust= np.sum(f_axial)
        # Torque main
        torque = np.sum(f_tan*self.r)
        # Induced power
        p_induced = (-self.v_axial.flatten() * f_axial.flatten()).sum()
        # Profile power
        p_profile = np.sum(0.5*1.225*(abs(self.omega).flatten()*self.r.flatten())**3 * self.Cd0.flatten()*self.r_steps.flatten()*self.chords.flatten())
        # Total power
        p_total = p_induced + p_profile
        # Ideal power
        A = 4*np.pi*(self.aircraft.props[0].diameter*0.5)**2
        p_ideal = thrust* np.sqrt(abs(thrust)/(2*1.225*A))
        # Power loading 
        power_loading = thrust/p_total
        # Figure of merit
        figure_of_merit = p_ideal/p_total
        # thrust_small

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
  
    def normalize(self, r):
        r = r.flatten()
        res = (r - r.min()) / (r.max() - r.min())
        return res
    
    def save_results(self, path):
        header_names = [
            "r", "v_axial", "v_tangential", "inflowangle", "alpha",
            "f_axial", "f_tan", "gammas", "Cl", "Cd", "chords", "twist",  "azimuth_x", "azimuth_y", "azimuth_z", "omega"
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
            self.twist.flatten(),
            self.n_azimuth[:,0].flatten(),
            self.n_azimuth[:,1].flatten(),
            self.n_azimuth[:,2].flatten(),
            self.omega.flatten()
        ], names=",".join(header_names))

        np.savez_compressed(f"{path}/_res.npz", data=data)
        print(f"Saved structured results to {path}/_res.npz")

        # compute disk area
        A = np.pi * (self.aircraft.props[0].diameter / 2) ** 2 * 4  # total disk area for quadcopter

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
            "disk_loading": self.thrust / A,
        }

        with open(f"{path}/performance.json", "w") as f:
            json.dump(performance, f, indent=4)
        #print(f'Saved performance metrics in {path}')


    def plot_alpha(self):
        data = np.load('_res.npz')
        data = data['data']
        r = data['r']
        alpha = data['alpha']
        main_n = self.main_n
        npM = self.npM
        plt.plot(r[:main_n-1], alpha[:main_n-1], label='Main prop')
        plt.plot(r[npM:npM+main_n-1], alpha[npM:npM+main_n-1], label='Second prop')
        plt.plot(r[2*npM:2*npM+main_n-1], alpha[2*npM:2*npM+main_n-1], label='Third prop')
        plt.plot(r[3*npM:3*npM+main_n-1], alpha[3*npM:3*npM+main_n-1], label='Fourth prop')
        plt.xlabel('Radius (m)')
        plt.ylabel('Angle of attack (deg)')
        plt.legend()
        plt.show()

    def plot_gammas(self):
        data = np.load('_res.npz')
        data = data['data']
        r = data['r']
        gammas = data['gammas']
        main_n = self.main_n
        npM = self.npM
        plt.plot(r[:main_n-1], gammas[:main_n-1], label='First prop')
        plt.plot(r[npM:npM+main_n-1], gammas[npM:npM+main_n-1], label='Second prop')
        plt.plot(r[2*npM:2*npM+main_n-1], gammas[2*npM:2*npM+main_n-1], label='Third prop')
        plt.plot(r[3*npM:3*npM+main_n-1], gammas[3*npM:3*npM+main_n-1], label='Fourth prop')
        plt.xlabel('Radius (m)')
        plt.ylabel('Circulation (m^2/s)')
        plt.legend()
        plt.show()

    def plot_twist(self):
        data = np.load('_res.npz')
        data = data['data']
        r = data['r']
        twist = data['twist']
        main_n = self.main_n
        npM = self.npM
        plt.plot(r[:main_n-1], twist[:main_n-1], label='Main prop')
        plt.plot(r[npM:npM+main_n-1], twist[npM:npM+main_n-1], label='Second prop')
        plt.plot(r[2*npM:2*npM+main_n-1], twist[2*npM:2*npM+main_n-1], label='Third prop')
        plt.plot(r[3*npM:3*npM+main_n-1], twist[3*npM:3*npM+main_n-1], label='Fourth prop')
        plt.xlabel('Radius (m)')
        plt.ylabel('Twist (deg)')
        plt.legend()
        plt.show()

    def plot_inflowangle(self):
        data = np.load('_res.npz')
        data = data['data']
        r = data['r']
        inflowangle = data['inflowangle']
        main_n = self.main_n
        npM = self.npM
        inflowangle = inflowangle*180/np.pi 
        plt.plot(r[:main_n-1], inflowangle[:main_n-1], label='Main prop')
        plt.plot(r[npM:npM+main_n-1], inflowangle[npM:npM+main_n-1], label='Second prop')
        plt.plot(r[2*npM:2*npM+main_n-1], inflowangle[2*npM:2*npM+main_n-1], label='Third prop')
        plt.plot(r[3*npM:3*npM+main_n-1], inflowangle[3*npM:3*npM+main_n-1], label='Fourth prop')
        plt.xlabel('Radius (m)')
        plt.ylabel('Inflow angle (deg)')
        plt.legend()
        plt.show()

    def plot_v_axial(self):
        data = np.load('_res.npz')
        data = data['data']
        r = data['r']
        v_axial = data['v_axial']
        main_n = self.main_n
        npM = self.npM
        plt.plot(r[:main_n-1], v_axial[:main_n-1], label='Main prop')
        plt.plot(r[npM:npM+main_n-1], v_axial[npM:npM+main_n-1], label='Second prop')
        plt.plot(r[2*npM:2*npM+main_n-1], v_axial[2*npM:2*npM+main_n-1], label='Third prop')
        plt.plot(r[3*npM:3*npM+main_n-1], v_axial[3*npM:3*npM+main_n-1], label='Fourth prop')
        plt.xlabel('Radius (m)')
        plt.ylabel('Axial velocity (m/s)')
        plt.legend()
        plt.show()
    
    def plot_v_tangential(self):
        data = np.load('_res.npz')
        data = data['data']
        r = data['r']
        v_tangential = data['v_tangential']
        main_n = self.main_n
        npM = self.npM
        plt.plot(r[:main_n-1], v_tangential[:main_n-1], label='Main prop')
        plt.plot(r[npM:npM+main_n-1], v_tangential[npM:npM+main_n-1], label='Second prop')
        plt.plot(r[2*npM:2*npM+main_n-1], v_tangential[2*npM:2*npM+main_n-1], label='Third prop')
        plt.plot(r[3*npM:3*npM+main_n-1], v_tangential[3*npM:3*npM+main_n-1], label='Fourth prop')
        plt.xlabel('Radius (m)')
        plt.ylabel('Tangential velocity (m/s)')
        plt.legend()
        plt.show()
    
    def plot_azimuth(self):
        data = np.load('_res.npz')
        data = data['data']
        r = data['r']
        azimuth_x = data['azimuth_x']
        azimuth_y = data['azimuth_y']
        azimuth_z = data['azimuth_z']
        main_n = self.main_n
        npM = self.npM
        plt.plot(r[:main_n-1], azimuth_x[:main_n-1], label='Main prop X')
        plt.plot(r[npM:npM+main_n-1], azimuth_x[npM:npM+main_n-1], label='Second prop X')
        plt.plot(r[2*npM:2*npM+main_n-1], azimuth_x[2*npM:2*npM+main_n-1], label='Third prop X')
        plt.plot(r[3*npM:3*npM+main_n-1], azimuth_x[3*npM:3*npM+main_n-1], label='Fourth prop X')
        
        plt.plot(r[:main_n-1], azimuth_y[:main_n-1], label='Main prop Y')
        plt.plot(r[npM:npM+main_n-1], azimuth_y[npM:npM+main_n-1], label='Second prop Y')
        plt.plot(r[2*npM:2*npM+main_n-1], azimuth_y[2*npM:2*npM+main_n-1], label='Third prop Y')
        plt.plot(r[3*npM:3*npM+main_n-1], azimuth_y[3*npM:3*npM+main_n-1], label='Fourth prop Y')

        plt.plot(r[:main_n-1], azimuth_z[:main_n-1], label='Main prop Z')
        plt.plot(r[npM:npM+main_n-1], azimuth_z[npM:npM+main_n-1], label='Second prop Z')
        plt.plot(r[2*npM:2*npM+main_n-1], azimuth_z[2*npM:2*npM+main_n-1], label='Third prop Z')
        plt.plot(r[3*npM:3*npM+main_n-1], azimuth_z[3*npM:3*npM+main_n-1], label='Fourth prop Z')
        plt.xlabel('Radius (m)')
        plt.ylabel('Azimuth components')
        plt.legend()
        plt.show()

    def plot_self(self):
        alpha = self.alpha 
        r = self.r 
        chords = self.chords
        Cl = self.Cl
        Cd = self.Cd
        Re = self.Re
        f_axial = self.f_axial
        f_tan = self.f_tan
        inflowangle = self.inflowangle * 180 / np.pi
        twist = self.twist
        v_axial = self.v_axial
        v_tangential = self.v_tangential
        vel_total = self.vel_total
        gamma = self.gammas
        alpha_ideal = self.alpha_cl32cd
        main_n = self.main_n
        npM = self.npM
        
        r_main = r[:main_n - 1]
        r_main = self.normalize(r_main)
        
        fig, axs = plt.subplots(3, 3, figsize=(15, 12))
        
        # Plot angle of attack (Main Rotor)
        # for i in range(4):
        #     axs[0, 0].plot(r_main, alpha[i*npM:(i)*npM + (main_n-1)], 
        #                 linestyle='--', label=f'rotor {i+1}')
        # axs[0, 0].set_title('Angle of Attack (Main Rotor)')
        # axs[0, 0].set_xlabel('Radius (m)')
        # axs[0, 0].set_ylabel('Alpha (deg)')
        # axs[0, 0].legend()
        # axs[0, 0].grid()
        for i in range(4):
            axs[0, 0].plot(r_main, f_axial[i*npM:(i)*npM + (main_n-1)], 
                        linestyle='--', label=f'rotor {i+1}')
        axs[0, 0].set_title('Angle of Attack (Main Rotor)')
        axs[0, 0].set_xlabel('Radius (m)')
        axs[0, 0].set_ylabel('Alpha (deg)')
        axs[0, 0].legend()
        axs[0, 0].grid()

        for i in range(4):
            axs[0, 1].plot(r_main, f_tan[i*npM:(i)*npM + (main_n-1)], 
                        linestyle='--', label=f'rotor {i+1}')
        axs[0, 1].set_title('Inflow Angle')
        axs[0, 1].set_xlabel('Radius (m)')
        axs[0, 1].set_ylabel('Inflow Angle (deg)')
        axs[0, 1].legend()
        axs[0, 1].grid()
        
        # Plot inflow angle
        # for i in range(4):
        #     axs[0, 1].plot(r_main, inflowangle[i*npM:(i)*npM + (main_n-1)], 
        #                 linestyle='--', label=f'rotor {i+1}')
        # axs[0, 1].set_title('Inflow Angle')
        # axs[0, 1].set_xlabel('Radius (m)')
        # axs[0, 1].set_ylabel('Inflow Angle (deg)')
        # axs[0, 1].legend()
        # axs[0, 1].grid()
        
        # Plot twist
        for i in range(4):
            axs[0, 2].plot(r_main, twist[i*npM:(i)*npM + (main_n-1)], 
                        linestyle='--', label=f'rotor {i+1}')
        axs[0, 2].set_title('Twist')
        axs[0, 2].set_xlabel('Radius (m)')
        axs[0, 2].set_ylabel('Twist (deg)')
        axs[0, 2].legend()
        axs[0, 2].grid()
        
        # Plot axial velocity
        for i in range(4):
            axs[1, 0].plot(r_main, v_axial[i*npM:(i)*npM + (main_n-1)], 
                        linestyle='--', label=f'rotor {i+1}')
        axs[1, 0].set_title('Axial Velocity')
        axs[1, 0].set_xlabel('Radius (m)')
        axs[1, 0].set_ylabel('v_axial (m/s)')
        axs[1, 0].legend()
        axs[1, 0].grid()
        
        # Plot Reynolds number
        for i in range(4):
            axs[1, 1].plot(r_main, Re[i*npM:(i)*npM + (main_n-1)], 
                        linestyle='--', label=f'rotor {i+1}')
        axs[1, 1].set_title('Reynolds Number')
        axs[1, 1].set_xlabel('Radius (m)')
        axs[1, 1].set_ylabel('Re')
        axs[1, 1].legend()
        axs[1, 1].grid()
        
        # Plot Mach number
        a = 340  # m/s
        V_infty = np.sqrt(v_axial**2 + v_tangential**2)
        M = np.array(V_infty) / a
        for i in range(4):
            axs[1, 2].plot(r_main, M[i*npM:(i)*npM + (main_n-1)], 
                        linestyle='--', label=f'rotor {i+1}')
        axs[1, 2].set_title('Mach Number')
        axs[1, 2].set_xlabel('Radius (m)')
        axs[1, 2].set_ylabel('Mach')
        axs[1, 2].legend()
        axs[1, 2].grid()
        
        # Plot Cl
        for i in range(4):
            axs[2, 0].plot(r_main, Cl[i*npM:(i)*npM + (main_n-1)], 
                        linestyle='--', label=f'rotor {i+1}')
        axs[2, 0].set_title('Lift Coefficient')
        axs[2, 0].set_xlabel('Radius (m)')
        axs[2, 0].set_ylabel('Cl')
        axs[2, 0].legend()
        axs[2, 0].grid()
        
        # Plot Cd
        for i in range(4):
            axs[2, 1].plot(r_main, Cd[i*npM:(i)*npM + (main_n-1)], 
                        linestyle='--', label=f'rotor {i+1}')
        axs[2, 1].set_title('Drag Coefficient')
        axs[2, 1].set_xlabel('Radius (m)')
        axs[2, 1].set_ylabel('Cd')
        axs[2, 1].legend()
        axs[2, 1].grid()
        
        # Plot circulation (gamma)
        for i in range(4):
            axs[2, 2].plot(r_main, gamma[i*npM:(i)*npM + (main_n-1)], 
                        linestyle='--', label=f'rotor {i+1}')
        axs[2, 2].set_title('Circulation')
        axs[2, 2].set_xlabel('Radius (m)')
        axs[2, 2].set_ylabel('Gamma (m²/s)')
        axs[2, 2].legend()
        axs[2, 2].grid()
        
        plt.tight_layout()
        plt.show()

    
if __name__ == "__main__":
    drone = defineDrone('./configs/base_quad.json',main_RPM=6_000)
    solver = QuadSolver(drone)
    solver.solve()
    #solver.aircraft.display()
    # solver.plot_twist()
    solver.plot_gammas()
    solver.plot_v_axial()
    # solver.plot_v_tangential()
    # solver.plot_inflowangle()
    # solver.plot_alpha()
    