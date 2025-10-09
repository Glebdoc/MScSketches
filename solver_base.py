import numpy as np 
import glob
import os
import ctypes
from geometry import defineDrone
import xfoilUtil as xf
from matplotlib import pyplot as plt

def preload_airfoil_data(data_folder="./airfoil/data/numpy_smoothed"):
        """
        Preload all airfoil npz files into a dictionary.
        """
        file_paths = glob.glob(os.path.join(data_folder, "*.npz"))
        preloaded_data = {}
        
        for file_path in file_paths:
            data = np.load(file_path, allow_pickle=True)
            airfoil_name = os.path.splitext(os.path.basename(file_path))[0].replace("numpy_", "")
            preloaded_data[airfoil_name] = {key: data[key] for key in data.files}
        
        return preloaded_data 

class BaseSolver:
    def __init__(self, aircraft) -> None:
        self.aircraft = aircraft
        self.name = str(aircraft.type) + " solver"
        self.compute_number_of_points()
        self.preloaded_data = preload_airfoil_data()
        self.REYNOLDS = aircraft.reynolds
        self.total_colloc_points = self.compute_collocation_points()
        self.compute_influence_matrices(plot=False)
        self.compute_azimuth_and_origin()
        self.compute_twist()
        self.compute_omega()
        self.compute_chords()
        self.compute_v_rotational()
        self.gammas = np.ones((self.collocN, 1))
        self.set_airfoil_array()
        self.compute_r_steps()
        self.compute_r()


    def compute_influence_matrices(self, plot=False):
        if os.name == 'nt':
            mylib = ctypes.CDLL("./mylib.dll")  
        else:
            mylib = ctypes.CDLL("./mylib.so")

        collocN = self.collocN
        total_colloc_points = self.total_colloc_points
        u_influences = np.zeros((collocN, collocN))
        v_influences = np.zeros((collocN, collocN))
        w_influences = np.zeros((collocN, collocN))

        table = np.array(self.aircraft.vortexTABLE)

        N = len(total_colloc_points)
        T = len(table)  
        core_size = ctypes.c_float(self.aircraft.core_size)

        collocationPoints_ptr = total_colloc_points.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        vortexTable_ptr = table.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        uInfluence_ptr = u_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        vInfluence_ptr = v_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        wInfluence_ptr = w_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

        # Call the C function
        res = mylib.computeInfluenceMatrices(N, T, collocationPoints_ptr, vortexTable_ptr, uInfluence_ptr, vInfluence_ptr, wInfluence_ptr, core_size)
        if res != 0:
            raise RuntimeError("Error in computeInfluenceMatrices")
        self.u_influences = u_influences
        self.v_influences = v_influences
        self.w_influences = w_influences



    def solve(self, weight=0.05, tolerance=1e-8, max_iterations=3_000):
        err = 1.0 
        iter = 0 
        u_influences = self.u_influences
        v_influences = self.v_influences
        w_influences = self.w_influences

        v_rotational = self.v_rotational
        n_azimuth = self.n_azimuth
        n_origin = self.n_origin
        total_colloc_points = self.total_colloc_points
        chords = self.chords
        twist = self.twist
        gammas = self.gammas
        airfoil_array = self.airfoil_array
        preloaded_data = self.preloaded_data
        Cl = np.zeros((self.collocN, 1))
        Cd = np.zeros((self.collocN, 1))
        r_steps = self.r_steps
        
        while (err > tolerance and iter<max_iterations):
            iter+=1
            if iter % 200 == 0:
                #print(f'Weight: {weight} Iter:{iter} Error:{err:.4f}')
                weight -= weight*0.01
                weight = max(weight, 0.03)
            u = u_influences@self.gammas 
            v = v_influences@self.gammas 
            w = w_influences@self.gammas 

            vel_total_x = u.flatten() + v_rotational[:, 0].flatten() 
            vel_total_y = v.flatten() + v_rotational[:, 1].flatten()
            vel_total_z = w.flatten() + v_rotational[:, 2].flatten()

            vel_total  = np.column_stack((vel_total_x, vel_total_y, vel_total_z))
            v_axial = np.sum(vel_total * n_azimuth, axis=1)

            tan_direction = np.cross(total_colloc_points - n_origin, n_azimuth)
            tan_direction = tan_direction / np.linalg.norm(tan_direction, axis=1)[:, np.newaxis]
            tan_direction = np.nan_to_num(tan_direction)
            
            v_tangential = np.sum(vel_total * tan_direction, axis=1)
            v_mag = np.sqrt(v_axial**2 + v_tangential**2)
            v_mag[v_mag > 300] = 300  # Cap velocity to avoid numerical issues
            Re = 1.225*v_mag.flatten()*chords.flatten()/1.81e-5
            inflowangle = np.arctan(-v_axial/v_tangential)

            alpha = twist.flatten() -  (inflowangle*180/np.pi).flatten()
            # Try catching stall
            alpha[alpha>15] = 15
            alpha[alpha<-5] = -5

            if self.REYNOLDS:
                cl, cd = xf.getPolar_batch(Re, alpha, airfoil_array, preloaded_data)
                Cl[:] = cl.reshape(-1, 1)
                Cd[:] = cd.reshape(-1, 1)

            else:
                raise NotImplementedError("XFOIL polar calculation without preloaded Reynolds number is not implemented yet.")
            
            gammas_old = gammas
            gammas = weight * Cl.flatten() * 0.5 * chords.flatten()* v_mag.flatten() + (1-weight)*gammas_old.flatten()
            self.gammas = gammas.reshape(-1, 1)
            err = np.linalg.norm(gammas - gammas_old) # maybe change 

        if iter == max_iterations:
            print(f"Warning: Maximum iterations reached ({max_iterations}) with error {err:.4f}")   
            #print(f'{self.name} Converged in {iter} iterations with error {err}')
        
        #np.savetxt('./auxx/v_axial.txt', v_axial)
        np.savetxt(f'{self.output_dir}/v_axial.txt', v_axial)
        #print('Saved v_axial to', f'{self.output_dir}/v_axial.txt')

        Lift = 0.5 * 1.225 * Cl.flatten() * (v_mag.flatten()**2) * chords.flatten() * r_steps.flatten()
        Drag = 0.5 * 1.225 * Cd.flatten() * (v_mag.flatten()**2) * chords.flatten() * r_steps.flatten()
        _, Cd0 = xf.getPolar_batch(Re, np.zeros((alpha.shape[0], 1)), airfoil_array, preloaded_data)
        _, alpha_cl32cd = xf.get_cl32cd_batch(Re, airfoil_array, preloaded_data)
        # save results
        self.alpha_cl32cd = alpha_cl32cd
        self.gammas = gammas
        self.v_axial = v_axial
        self.v_tangential = v_tangential
        self.vel_total = vel_total
        self.Cl = Cl
        self.Cd = Cd
        self.alpha = alpha
        self.Re = Re
        self.inflowangle = inflowangle
        self.twist = twist
        self.chords = chords    
        self.lift = Lift
        self.drag = Drag
        self.omega = self.omega
        self.Cd0 = Cd0
        self.compute_forces() 
        self.check_stall()
