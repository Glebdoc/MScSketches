import numpy as np
import pyvista as pv
import bemUtils as bu
import json
import xfoilUtil as xfu
import matplotlib.pyplot as plt
from plotter import set_bw_design

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def translate(self, translation):
        self.x += translation[0]
        self.y += translation[1]
        self.z += translation[2]

    def rotate(self, R):
        new_coords = R @ np.array([self.x, self.y, self.z])
        self.x, self.y, self.z = new_coords
        return self

class Vortex(Point):
    def __init__(self, point1, point2, Gamma=1):
        self.p1 = point1
        self.p2 = point2 
        self.x1 = point1.x      
        self.y1 = point1.y
        self.z1 = point1.z
        self.x2 = point2.x
        self.y2 = point2.y
        self.z2 = point2.z
        self.Gamma = Gamma

    def translate(self, translation):
        self.x1 += translation.x
        self.y1 += translation.y
        self.z1 += translation.z
        self.x2 += translation.x
        self.y2 += translation.y
        self.z2 += translation.z  

    def rotate(self, R):
        self.p1.rotate(R)
        self.p2.rotate(R)

        self.x1 = self.p1.x
        self.y1 = self.p1.y
        self.z1 = self.p1.z
        self.x2 = self.p2.x
        self.y2 = self.p2.y
        self.z2 = self.p2.z

class HorseShoe(Vortex):
    def __init__(self, left, centre, right):
        self.leftset = left
        self.centre = centre
        self.rightset = right
        
    def get_plot_data(self, start_index=0):

        points = []
        lines = []
        index = start_index

        # Add left vortices
        for vortex in self.leftset:
            points.append([vortex.x1, vortex.y1, vortex.z1])
            points.append([vortex.x2, vortex.y2, vortex.z2])
            lines.append([2, index, index + 1])
            index += 2

        # Add central vortex
        points.append([self.centre.x1, self.centre.y1, self.centre.z1])
        points.append([self.centre.x2, self.centre.y2, self.centre.z2])
        lines.append([2, index, index + 1])
        index += 2

        # Add right vortices
        for vortex in self.rightset:
            points.append([vortex.x1, vortex.y1, vortex.z1])
            points.append([vortex.x2, vortex.y2, vortex.z2])
            lines.append([2, index, index + 1])
            index += 2

        return np.array(points), np.array(lines).flatten()
    
    def translate(self, translation):
        for vortex in self.leftset:
            vortex.translate(translation)
        for vortex in self.rightset:
            vortex.translate(translation)

        self.centre.translate(translation)

    def rotate(self, R):
        for vortex in self.leftset:
            vortex.rotate(R)
        for vortex in self.rightset:
            vortex.rotate(R)
        self.centre.rotate(R)


def median_filter(data, kernel_size=5, startFromTheEnd=True, times=1):
    for _ in range(times):
        if startFromTheEnd:
            data = data[::-1]
        for i in range(len(data)):
            if i < kernel_size // 2 or i >= len(data) - kernel_size // 2:
                continue
            data[i] = np.median(data[i - kernel_size // 2:i + kernel_size // 2 + 1])
        if startFromTheEnd:
            data = data[::-1]
    return data

class Propeller():
    def __init__(self, position, angles, hub, diameter, NB, pitch, RPM, chord, n, U=2, wake_length=5, distribution='cosine', bodyIndex=0, small=False, main_rotor=None, 
                 contraction=True, blade1_angle=0, downwash=None, wake_bend = False, output_dir=None):
        
        """
        This is the initialization of a propeller object. For both main and small rotors.
        """
        self.diameter = diameter
        self.position = position
        self.azimuth = np.array([0, 0, 1])
        self.origin = position
        self.angles = np.array(angles)
        self.NB = NB
        self.pitch = pitch
        self.RPM = RPM
        self.n = n
        self.U = U
        self.wake_length = wake_length
        self.ppr=30                         # number of points per revolution in the wake
        self.chord = chord
        self.v_axial = None
        self.Ct = None
        self.vortexTABLE = []
        self.horseShoes = None
        self.bodyIndex = bodyIndex          # Index of the body: 0 for main rotor, 1,2,3... for small rotors (useful for orientation within element domain)
        self.dt = None
        # flags
        self.small = small                  # Unnecessary as bodyIndex can be used
        self.wake_bend = wake_bend          # Flag to enable wake bending for small rotors : currently used by default, so might be removed
        self.downwash = downwash            # Downwash value to be subtracted from the freestream velocity in the wake bending function (poor implementation for now, severe simplification, yet sensitivity showed that it has close to no effect on the results)
        self.distribution = distribution    # Type of radial distribution of the blade elements
        self.output_dir = output_dir


        """
        Current implementation only supports cosine and uniform spacing of the elements

        """

        if self.distribution == 'cosine':
            R = diameter*0.5
            theta = np.linspace(np.arccos(hub/R), np.pi/20,  n) # used 10   1( - close to no effect)   20 - more effect
            spacing = np.cos(0.5*theta)
            spacing = (spacing - np.min(spacing)) / (np.max(spacing) - np.min(spacing)) 
            spacing = spacing *(R - hub) + hub
            self.r = spacing
        else:
            self.r = np.linspace(hub, diameter*0.5 , n)


        #collocationPoints = np.zeros((3, n-1))
        self.collocationPoints = [np.vstack((np.zeros(n-1), (self.r[:-1]+self.r[1:])*0.5, np.zeros(n-1)))]
        #collocationPoints[1,: ] = (self.r[:-1]+self.r[1:])*0.5
        #self.collocationPoints = collocationPoints
        self.sigma = np.average(self.NB*self.chord[:n]/(np.pi*self.r))

        self.assemble(main_rotor=main_rotor, contraction=contraction)
        if self.small:
            """
            We first need to rotate around Z to pic a desired blade position with respect to the main rotor.
            Then we rotate around Y - simulating the naccelle incline 
            Then we bend the wake modifying 
            """
            temp_angles = np.array([0, 0, blade1_angle*np.pi/180])
            self.rotate(extra=True, a=temp_angles[0], b=temp_angles[1], c=temp_angles[2])
            
        self.rotate()
        self.translate(position)

        if self.wake_bend:
                self.bendSmallWake_new(main_rotor)
                #self.bendSmallWake(main_rotor)

    def bendSmallWake_new(self, main_rotor):
        if main_rotor.downwash is not None:
            downwash = main_rotor.downwash
        else:
            downwash = 0.0
        #print("Using downwash of ", downwash, " m/s for small rotor wake bending")
        points = self.vortexTABLE[:, :6]

        # Define helix parameters
        R = np.sqrt(self.position.x**2 + self.position.y**2)
        omega = -2*np.pi*main_rotor.RPM/60  # <--- spin: +CCW, -CW (looking from +Z toward origin)
        a = R
        #b = self.angles[1]  # pitch of the helix (rise per radian)
        #b= -a*omega*np.tan(np.pi/2 - self.angles[1]) - np.tan(downwash/(omega*R))
        b =  a*omega*np.tan(downwash/(omega*R))

        #print("Helix pitch b: ", b)

        angle_addition = np.arctan2(0.05, R)

        phi0 = self.angles[2] + np.pi/2  + angle_addition      # phase shift (radians)

        t = self.dt
        t = np.concatenate([[0], t])  # tile for both sides of the wake
        theta = omega*t + phi0 
        

        # arclength (speed * t); speed = ||r'(t)||
        #s = np.sqrt((a*omega)**2 + b**2) * t

        # --- Frenet frame (vectorized) for r(t) = (a cos θ, a sin θ, b t) ---
        # Unit tangent T = r'(t)/||r'(t)|| with ω
        speed_inv = 1.0 / np.sqrt((a*omega)**2 + b**2)
        T = np.zeros((len(t), 3))
        T[:,0] = -a*omega*np.sin(theta) * speed_inv
        T[:,1] =  a*omega*np.cos(theta) * speed_inv
        T[:,2] =  b * speed_inv

        # Unit normal N (points radially inward)
        N = np.vstack((-np.cos(theta), -np.sin(theta), np.zeros_like(t))).T

        # Unit binormal B = T × N
        B = np.cross(T, N)

        # Curve points
        rx = a*np.cos(theta)
        ry = a*np.sin(theta)
        rz = b*t

        # tile
        # T = np.tile(T, (2*(self.n-1), 1))
        # N = np.tile(N, (2*(self.n-1), 1))
        # B = np.tile(B, (2*(self.n-1), 1))
        # rx = np.tile(rx, 2*(self.n-1))
        # ry = np.tile(ry, 2*(self.n-1))
        # rz = np.tile(rz, 2*(self.n-1))

        T_1 = np.concatenate([T[1:, :], T[:-1, :]], axis=0)
        T_1 = np.tile(T_1, (self.n-1, 1))
        N_1 = np.concatenate([N[1:, :], N[:-1, :]], axis=0)
        N_1 = np.tile(N_1, (self.n-1, 1))
        B_1 = np.concatenate([B[1:, :], B[:-1, :]], axis=0)
        B_1 = np.tile(B_1, (self.n-1, 1))

        rx_1 = np.concatenate([rx[1:], rx[:-1]], axis=0)
        rx_1 = np.tile(rx_1, self.n-1)
        ry_1 = np.concatenate([ry[1:], ry[:-1]], axis=0)
        ry_1 = np.tile(ry_1, self.n-1)
        rz_1 = np.concatenate([rz[1:], rz[:-1]], axis=0)
        rz_1 = np.tile(rz_1, self.n-1)

        T_2 = np.concatenate([T[:-1, :], T[1:, :]], axis=0)
        T_2 = np.tile(T_2, (self.n-1, 1))
        N_2 = np.concatenate([N[:-1, :], N[1:, :]], axis=0)
        N_2 = np.tile(N_2, (self.n-1, 1))
        B_2 = np.concatenate([B[:-1, :], B[1:, :]], axis=0)
        B_2 = np.tile(B_2, (self.n-1, 1))

        rx_2 = np.concatenate([rx[:-1], rx[1:]], axis=0)
        rx_2 = np.tile(rx_2, self.n-1)
        ry_2 = np.concatenate([ry[:-1], ry[1:]], axis=0)
        ry_2 = np.tile(ry_2, self.n-1)
        rz_2 = np.concatenate([rz[:-1], rz[1:]], axis=0)
        rz_2 = np.tile(rz_2, self.n-1)

        # T_2 = np.concatenate([T[1:, :], T[:-1, :]], axis=0)
        # T_2 = np.tile(T_2, (self.n-1, 1))
        # N_2 = np.concatenate([N[1:, :], N[:-1, :]], axis=0)
        # N_2 = np.tile(N_2, (self.n-1, 1))
        # B_2 = np.concatenate([B[1:, :], B[:-1, :]], axis=0)
        # B_2 = np.tile(B_2, (self.n-1, 1))

        # rx_2 = np.concatenate([rx[1:], rx[:-1]], axis=0)
        # rx_2 = np.tile(rx_2, self.n-1)
        # ry_2 = np.concatenate([ry[1:], ry[:-1]], axis=0)
        # ry_2 = np.tile(ry_2, self.n-1)
        # rz_2 = np.concatenate([rz[1:], rz[:-1]], axis=0)
        # rz_2 = np.tile(rz_2, self.n-1)



        # Original axis 
        origin = np.array([self.position.x, self.position.y, self.position.z])
        direction = -self.azimuth / np.linalg.norm(self.azimuth)
        start = 0
        for i in range(self.NB):
                end = start+ 2*(self.n-1)*(len(t)-1)
                for i in range(1,3):
                #for i in range(3,1):
                    ##################################################################
                    v = points[start:end, 3*(i-1):3*i] - origin
                    dir2 = direction @ direction
                    d = v - ((v @ direction)[:, None] / dir2) * direction
                    
                    # d_T = np.einsum('ij,ij->i', d, T)
                    # d_N = np.einsum('ij,ij->i', d, N)
                    # d_B = np.einsum('ij,ij->i', d, B)

                    d_T = np.einsum('ij,ij->i', d, T_1 if i==1 else T_2)
                    d_N = np.einsum('ij,ij->i', d, N_1 if i==1 else N_2)
                    d_B = np.einsum('ij,ij->i', d, B_1 if i==1 else B_2)

                    if i ==1 :
                        points[start: end,  0:3] = d_T[:, None]*T_1 + d_N[:, None]*N_1 + d_B[:, None]*B_1 + np.vstack((rx_1, ry_1, rz_1)).T
                    else:
                        points[start: end,  3:6] = d_T[:, None]*T_2 + d_N[:, None]*N_2 + d_B[:, None]*B_2 + np.vstack((rx_2, ry_2, rz_2)).T
                    
                    # points[start:end, 3*(i-1): 3*(i-1)+3] = d_T[:, None]*T + d_N[:, None]*N + d_B[:, None]*B + np.vstack((rx, ry, rz)).T 



                start = end+(self.n-1)

        self.vortexTABLE[:,:6] = points

    def bendSmallWake(self, downwash=False):

        """
        The current implementation essentially only works for a 90 inclined naccelle.


        
        """

        tempR = np.sqrt(self.position.x**2 + self.position.y**2)
        points = self.vortexTABLE[:, :6]
        points = points.reshape(-1, 3)

        points[:, 1] += tempR
        #theta= np.arctan(points[:, 2]/tempR)
        theta = points[:, 2]/tempR

        r_to_axis = np.sqrt(tempR**2 + points[:, 2]**2)

        r_to_point = np.sqrt((points[:, 2]**2).flatten() + (points[:, 1]**2).flatten())

        deltaR = r_to_point - r_to_axis
        r_new = tempR + deltaR

        dz = r_new * np.sin(theta)
        dy = r_new * np.cos(theta) 

        dy -= tempR
        
        points[:, 2] = dz
        points[:, 1] = dy
        points = points.reshape(-1, 6)

        if downwash:
            omega = 2*np.pi*self.RPM/60
            length = self.diameter*self.wake_length
            total_time = length/self.U
            nrevs = total_time*omega/(2*np.pi)
            total_steps = int(nrevs*self.ppr)
            # total_time = length/self.U
            # nrevs = total_time*omega/(2*np.pi)
            # total_steps = int(nrevs*ppr)

            length = self.diameter*self.wake_length
            t1rev = 60 /self.RPM
            dt_step = t1rev/ self.ppr
            u_local = np.average(self.U)
            time = np.linspace(0, length/u_local, total_steps)
            time = np.tile(time, 2*(self.n-1))  
            start = 0
            for i in range(self.NB):
                end = start+ 2*(self.n-1)*total_steps 

                points[start:end, 0] += time*(-self.downwash)
                points[start:end, 3] += time*(-self.downwash)
                start = end+(self.n-1)

        self.vortexTABLE[:,:6] = points

    def assemble(self, main_rotor=None, contraction=True):

        """
        This function generates the vortex table for the propeller object. 
        Essentially it creates the wake geometry based on the input parameters.

        """

        if main_rotor is not None:
            n_main = main_rotor.n
            main_NB = main_rotor.NB

        ppr = self.ppr 
        omega = 2*np.pi*self.RPM/60
        length = self.diameter*self.wake_length
        total_time = length/self.U
        nrevs = total_time*abs(omega)/(2*np.pi)
        total_steps = int(nrevs*ppr)

        table = []

        dt = np.linspace(0, total_time, total_steps)
        zw = -self.U*dt 

        try:
            #v_axial_orig = np.genfromtxt('./auxx/v_axial.txt')
            #print("Reading induced velocity from ", f'{self.output_dir}/v_axial.txt')
            v_axial_orig = np.genfromtxt(f'{self.output_dir}/v_axial.txt')
            # check if the length of v_axial_orig is compatible with the current propeller
            # if len(v_axial_orig) != int((self.n-1)*self.NB):
            #     v_axial = None 
            #     # exit the try block
            #     raise ValueError("Incompatible v_axial length")
            if self.bodyIndex == 0:
                #print("Using main rotor induced velocity")
                v_axial = median_filter(v_axial_orig[:self.n-1].copy(), kernel_size=3, times=1)
                v_axial = np.tile(v_axial, (self.NB))
                # compute the downwash 
                # average induced velocity in the tip region 
                length_tip = int(0.2*(self.n-1))
                self.downwash = np.mean(v_axial_orig[(self.n-1)-length_tip :self.n-1])
                #print("v_axial tip = ", v_axial_orig[(self.n-1)-length_tip :self.n-1])

            else:
                start = main_NB*(n_main-1) + (self.n-1)*self.NB*(self.bodyIndex-1)
                end = main_NB*(n_main-1) + (self.n-1)*self.NB*self.bodyIndex
                v_axial = median_filter(v_axial_orig[start:end].copy(), kernel_size=3, times=1)
                v_axial = v_axial_orig[start:end]

            



        except:
            v_axial = None

        if v_axial is None:
            #my_U = np.linspace(-1, -3, self.n-1)
            omega = 2*np.pi*abs(self.RPM)/60
            #solidity = self.NB*self.chord[:self.n-1].flatten()/(np.pi*self.r[:self.n-1])
            solidity = 0.1
            twist_local = np.radians(self.pitch[:self.n-1])

            cla = 0.5
            inflowratio = (solidity*cla/16)*(np.sqrt(1 + 32*twist_local*self.r[:-1]/(cla*solidity)) - 1)
            if self.bodyIndex == 0:
                my_U = - inflowratio*omega*self.r[:-1] 
            else:
                my_U = - inflowratio*omega*self.r[:-1] - main_rotor.RPM*0.10472*main_rotor.diameter*0.5

        else:
            my_U = v_axial[:self.n-1]
        self.zw = zw

        if contraction:

            if self.bodyIndex == 0:
                contTheta = np.linspace(0, 2*np.pi*nrevs, total_steps)
                if self.Ct is None:
                    Ct = 0.01
                else:
                    Ct = self.Ct
                A = 0.78
                lmbda = 4.0 *(Ct**0.5)
                mult = A + (1-A)*np.exp(-lmbda*contTheta)
            else:
                mult = np.ones(len(zw))
        else:
            mult = np.ones(len(zw))
        

        for j in range(self.NB):
            if v_axial is not None:
                my_U = v_axial[j*(self.n-1):(j+1)*(self.n-1)]
            shift = j*2*np.pi/self.NB
            if j != 0:
                R = bu.rotation_matrix(0, 0, shift)
                self.collocationPoints.extend(R @ self.collocationPoints[:self.n-1])
                #rotated_collocs = R @ (self.collocationPoints[:, :self.n-1])
                #self.collocationPoints = np.hstack([self.collocationPoints, rotated_collocs])
            else:
                R = np.eye(3)
            
            
            hn = np.arange((j+0)*(self.n-1), (j+1)*(self.n-1))

            modified_pitch_array = np.zeros(self.n)
            modified_pitch_array[:-1] = self.pitch[:self.n-1]
            modified_pitch_array[-1] = self.pitch[-1]  # Ensure the last element is included

            x_0 = np.zeros(self.n)
            x_1 = self.chord[:self.n] * np.cos(np.radians(modified_pitch_array))  # chord length in x direction

            #xwl_m = np.outer(self.r[:-1], np.sin(omega*dt)*mult) + self.chord[:self.n-1].reshape(-1, 1)
            xwl_m = np.outer(self.r[:-1], np.sin(omega*dt[1:])*mult[:-1]) + self.chord[:self.n-1].reshape(-1, 1)
            xwl = np.hstack([x_0[:-1].reshape(-1, 1), x_1[:-1].reshape(-1,1), xwl_m])
            xwl = np.stack((xwl[:, 1:], xwl[:, :-1]), axis=2).reshape(-1, 2)

            hnl = np.repeat(hn, xwl.shape[0]//(self.n-1))

            #xwr_m = np.outer(self.r[1:], np.sin(omega*dt)*mult) + self.chord[1:self.n].reshape(-1, 1)
            xwr_m = np.outer(self.r[1:], np.sin(omega*dt[1:])*mult[:-1]) + self.chord[1:self.n].reshape(-1, 1)
            xwr = np.hstack([x_0[1:].reshape(-1, 1), x_1[1:].reshape(-1,1), xwr_m])
            xwr = np.stack((xwr[:, :-1], xwr[:, 1:]), axis=2).reshape(-1, 2)
            hnr = np.repeat(hn, xwr.shape[0]//(self.n-1))

            xc = np.stack((x_0[:-1], x_0[1:]), axis=1)

            hn = np.hstack([hnl, hnr, hn]).reshape(-1, 1)

            x_coord = np.vstack([xwl, xwr, xc]).reshape(-1, 1)

            y_0 = self.r 

            ywl_m = np.outer(self.r[:-1], np.cos(omega*dt)*mult) #+ self.r[:-1].reshape(-1,1)
            ywl = np.hstack([y_0[:-1].reshape(-1, 1), ywl_m])
            ywl = np.stack((ywl[:, 1:], ywl[:, :-1]), axis=2).reshape(-1, 2)

            ywr_m = np.outer(self.r[1:], np.cos(omega*dt)*mult)
            ywr = np.hstack([y_0[1:].reshape(-1, 1), ywr_m])
            ywr = np.stack((ywr[:, :-1], ywr[:, 1:]), axis=2).reshape(-1, 2)

            yc = np.stack((y_0[:-1], y_0[1:]), axis=1)

            y_coord = np.vstack([ywl, ywr, yc]).reshape(-1, 1)
            
            
            

            z_0 = np.zeros(self.n) 
            z_1 = -self.chord[:self.n] * np.sin(np.radians(modified_pitch_array))
            zwl = np.outer(my_U, dt[1:]) + z_1[:-1].reshape(-1, 1)  # subtract the chord length to get the correct position 
            # insert first column of zeros

            zwl  = np.hstack([z_0[:-1].reshape(-1, 1), z_1[:-1].reshape(-1, 1), zwl])
            zwl = np.stack((zwl[:, 1:], zwl[:, :-1]), axis=2).reshape(-1, 2)
            

            #zwr = np.outer(np.ones(self.n-1), zw)
            zwr = np.outer(my_U, dt[1:])  + z_1[1:].reshape(-1, 1) #+ self.chord[1:self.n].reshape(-1, 1)
            #zwr  = np.hstack([z_0[1:].reshape(-1, 1), zwr])
            zwr  = np.hstack([z_0[1:].reshape(-1, 1), z_1[1:].reshape(-1,1), zwr])
            zwr = np.stack((zwr[:, :-1], zwr[:, 1:]), axis=2).reshape(-1, 2)

            zc = np.stack((z_0[:-1], z_0[1:]), axis=1)

            z_coord = np.vstack([zwl, zwr, zc]).reshape(-1, 1)

            coordinates = np.hstack([x_coord, y_coord, z_coord])
            
            
            coordinates = (R @ coordinates.T).T

            x = coordinates[:, 0].reshape(-1, 2)
            y = coordinates[:, 1].reshape(-1, 2)
            z = coordinates[:, 2].reshape(-1, 2)

            Gamma = np.ones((x.shape[0], 1))


            chunk = np.stack([x[:,0], y[:,0], z[:,0], x[:,1], y[:,1], z[:,1], hn[:, 0], Gamma[:,0]], axis=1)
            table.append(chunk)

        self.vortexTABLE = np.vstack(table)
        self.dt = dt

    def translate(self, translation):
        if self.small:
            addition  =  self.azimuth*(0.05)
            translation.x += addition[0]
            translation.y += addition[1]
            translation.z += addition[2]


        table = self.vortexTABLE
        table[:,0] += translation.x
        table[:,1] += translation.y
        table[:,2] += translation.z
        table[:,3] += translation.x
        table[:,4] += translation.y
        table[:,5] += translation.z

        for i in range(self.NB):
            collocs = np.array(self.collocationPoints[i])
            #collocs = self.collocationPoints[:, i*(self.n - 1) : (i+1)*(self.n - 1)]
            collocs[0,:] += translation.x
            collocs[1,:] += translation.y
            collocs[2,:] += translation.z
            self.collocationPoints[i] = collocs
            #self.collocationPoints[:, i*(self.n - 1) : (i+1)*(self.n - 1)] = collocs


        self.vortexTABLE = table
                

    def rotate(self, extra=False, a = None, b=None, c=None):    
        
        if not extra:
            R = bu.rotation_matrix(*self.angles)
        else:
            R = bu.rotation_matrix(a, b, c)


        table = self.vortexTABLE

        points = table[:, :6].reshape(-1, 3) 
        points = (R @ points.T).T 
        table[:, :6] = points.reshape(-1, 6)
        self.vortexTABLE = table

        for i in range(self.NB):
            self.collocationPoints[i] = R @ self.collocationPoints[i]

        # for i in range(self.NB):
        #     rotated_points = R @ self.collocationPoints[:, i*(self.n - 1) : (i+1)*(self.n - 1)]
        #     self.collocationPoints[:, i*(self.n - 1) : (i+1)*(self.n - 1)] = rotated_points

        self.azimuth = R @ self.azimuth



    def display(self, color):

        all_points = []
        all_lines = []
        start_index = 0

        for i in self.horseShoes:
            points, lines = i.get_plot_data(start_index)
            all_points.extend(points)
            all_lines.extend(lines)
            start_index += len(points)

        plotter = pv.Plotter()
        poly_data = pv.PolyData(np.array(all_points))
        poly_data.lines = np.array(all_lines)
        plotter.add_mesh(poly_data, color=color, line_width=2)

        axis_origins, axis_directions, axis_colors = get_axis_vectors()
        for i in range(3):
            plotter.add_arrows(
                np.array([axis_origins[i]]), 
                np.array([axis_directions[i]]), 
                color=axis_colors[i], 
                mag=0.05
        )

        plotter.show()


class Drone:
    def __init__(self, main_position, main_angles, main_hub, main_diameter,
                 main_NB, main_pitch, main_RPM, main_chord, main_n, main_airfoil, small_airfoil,
                 small_props_angle, small_props_diameter, small_props_NB, small_props_hub,
                 small_props_RPM, small_props_chord, small_props_n, small_props_pitch,
                 mainWakeLength, smallWakeLength, main_U, small_U, main_distribution, 
                 small_distribution, type,
                 helicopter=False, contraction=True, wind=None, reynolds=False, core_size = 1e-5, 
                 blade1_angle=0, downwash=None, wake_bend=False, output_dir=None):
        # Main propeller

        self.main_prop = Propeller(main_position, 
                                   main_angles,
                                   main_hub,
                                   main_diameter,
                                   main_NB, 
                                   main_pitch, 
                                   main_RPM, 
                                   main_chord, 
                                   main_n,
                                   U=main_U,
                                   wake_length=mainWakeLength,
                                   distribution=main_distribution, 
                                   contraction=contraction,
                                   output_dir=output_dir
                                   )
        
        # Small propellers
        self.core_size = core_size
        self.main_prop.airfoil = main_airfoil
        self.small_airfoil = small_airfoil
        self.small_props = []
        self.wind = wind
        self.type = type
        self.helicopter = helicopter
        self.reynolds = reynolds
        self.total_velocity_vectors = None
        self.total_collocation_points = None
        self.axial_velocity = None
        self.tangential_velocity = None
        self.vortexTABLE = self.main_prop.vortexTABLE
        addHSN = max(self.vortexTABLE[:, -2])
        main_NB = self.main_prop.NB
        main_R = self.main_prop.diameter/2
        if helicopter==False:
            for i in range(main_NB):
                sign = (-1)**i
                shift = i*(2*np.pi/main_NB)
                position = Point(main_R*np.cos(shift+np.pi/2), main_R*np.sin(shift+np.pi/2), 0.0)

                angles = np.array([0,-small_props_angle*np.pi/180, shift]) 
                small_prop = Propeller(position, 
                                    angles, 
                                    small_props_hub, 
                                    small_props_diameter, 
                                    small_props_NB,
                                    small_props_pitch, 
                                    small_props_RPM, 
                                    small_props_chord, 
                                    small_props_n,
                                    U=small_U,
                                    wake_length=smallWakeLength,
                                    distribution=small_distribution,
                                    bodyIndex=i+1,
                                    small=True, main_rotor=self.main_prop, contraction=contraction, blade1_angle =blade1_angle,
                                    downwash=downwash, wake_bend=wake_bend, output_dir=output_dir
                                    )
                
                table = small_prop.vortexTABLE
                table[:, -2] += addHSN+1
                addHSN = np.max(table[:, -2])


                self.vortexTABLE = np.concatenate((self.vortexTABLE, table), axis=0)

                self.small_props.append(small_prop)
        self.nPoints = np.max(self.vortexTABLE[:, -2])

    def translate(self, translation):
        self.main_prop.translate(translation)
        for small_prop in self.small_props:
            small_prop.translate(translation)
    
    def rotate(self, delta_x=0, delta_y=0, delta_z=0):
        self.main_prop.rotate(delta_x, delta_y, delta_z)
        for small_prop in self.small_props:
            small_prop.rotate(delta_x, delta_y, delta_z)

    def display(self, color_main='blue', color_small='green', extra_points=None, extra_lines=None):
        bodies = [self.main_prop] + self.small_props
        colors = [color_main] + [color_small]*len(self.small_props)
        table_final = self.vortexTABLE
        #np.savetxt('table_final_blender.txt', table_final)
        bu.scene(bodies, colors, collocation_points = self.total_collocation_points, 
              total_velocity_vectors=self.total_velocity_vectors,
              axial_velocity_vectors=self.axial_velocity, 
              tangential_velocity_vectors=self.tangential_velocity, 
              extra_points=extra_points, extra_lines=extra_lines, savefig=True)
def middle(array):
    return (array[:-1] + array[1:])*0.5      
  
def plot_geometry_only(drone):
    fig, ax = plt.subplots(1,2, figsize=(12,6))
    design = set_bw_design()

    # the left plot contains chord distributions for main and small propeller
    r_main = drone.main_prop.r
    R = drone.main_prop.diameter/2
    r_main = middle(r_main)
    n_main = drone.main_prop.n
    chord_main = drone.main_prop.chord[:(n_main)]
    chord_main = middle(chord_main) 
    pitch_main = drone.main_prop.pitch[:(n_main-1)]
    ax[0].plot(r_main/R, chord_main/R, label='Main propeller', color=design['colors'][0])
    ax[1].plot(r_main/R, pitch_main , label='Main propeller', color=design['colors'][0])
    if not drone.helicopter:
        R_small = drone.small_props[0].diameter/2
        r_small = drone.small_props[0].r
        r_small = middle(r_small)
        # map the small propeller chord to the main propeller radius for better visualization
        n_small = drone.small_props[0].n
        chord_small = drone.small_props[0].chord[:(n_small)]
        chord_small = middle(chord_small)
        pitch = drone.small_props[0].pitch[:(n_small)]
        ax[0].plot(r_small/R_small, chord_small/R_small, label='Small propeller', color=design['colors'][1])
        ax[1].plot(r_small/R_small, pitch , label='Small propeller', color=design['colors'][1])
    ax[0].set_xlabel('r/R')
    ax[0].set_ylabel('c/R')
    ax[0].legend()  
    ax[1].set_xlabel('r/R')
    ax[1].set_ylabel('Pitch (deg)')
    ax[1].legend()
    plt.show()

def parabolic_distribution(root, tip, r, a):

    a = a
    c = root
    locR = r[-1] - r[0]
    b = (tip -c - a*locR*locR)/ locR
    x = np.linspace(0, locR, len(r))
    return  a*x**2 + b*x + c

def synthetic_twist_curve(r, beta_root=30, beta_tip=12,
                          bump_pos=0.2, bump_height=6, noise=False,
                          seed=None):
    """
    Generate a synthetic blade twist curve β(r/R) with typical shape.

    Parameters
    ----------
    r : array-like
        Radial positions, normalized 0..1.
    beta_root : float
        Pitch at root (deg).
    beta_tip : float
        Pitch at tip (deg).
    bump_pos : float
        Position (r/R) of local bump near root.
    bump_height : float
        Height of that bump (deg).
    noise : float
        Random jitter amplitude (deg).
    seed : int
        Random seed for reproducibility.

    Returns
    -------
    beta : ndarray
        β distribution, deg.
    """
    rng = np.random.default_rng(seed)
    r = np.asarray(r)

    # base linear decay
    beta = beta_root + (beta_tip - beta_root) * r

    # add a Gaussian-like bump near the root
    beta += bump_height * np.exp(-((r - bump_pos) / 0.1)**2)

    # small random noise
    if noise > 0:
        beta += rng.normal(scale=noise, size=r.shape)

    return beta


    

# drone = defineDrone('configs/base.json', main_RPM=330, small_RPM=10000, plotGeom=True)
# plot_geometry_only(drone)

class QuadCopter:
    def __init__(self, R, prop_hub, prop_diameter, 
                 prop_NB, prop_pitch, prop_RPM, 
                 prop_chord, prop_n, wake_length, 
                 distribution, contraction, downwash, reynolds, main_airfoil, core_size=1e-5):
        self.R = R
        self.type= 'quadcopter'
        self.props = []
        self.airfoil = main_airfoil
        self.total_collocation_points = None
        self.total_velocity_vectors = None
        self.axial_velocity = None
        self.tangential_velocity = None
        self.reynolds = reynolds
        self.core_size = core_size
        self.airfoil = main_airfoil
        self.v_axial = None
        self.vortexTABLE = None
        addHSN = 0

        for i in range(4):
            # sign = (-1)**i
            shift = i*(2*np.pi/4)
            position = Point(self.R*np.cos(shift), self.R*np.sin(shift), 0.0)

            angles = np.array([0,0, 0])

            prop = Propeller(position, 
                                angles, 
                                prop_hub, 
                                prop_diameter, 
                                prop_NB,
                                prop_pitch, 
                                prop_RPM, 
                                prop_chord, 
                                prop_n,
                                wake_length=wake_length,
                                distribution=distribution,
                                bodyIndex=0,
                                contraction=contraction,
                                downwash=downwash, )
            table = prop.vortexTABLE
            
            
            if i == 0:
                self.vortexTABLE = prop.vortexTABLE
            else:
                addHSN = np.max(self.vortexTABLE[:, -2])
                table[:, -2] += addHSN+1
                self.vortexTABLE = np.concatenate((self.vortexTABLE, table), axis=0)
            self.props.append(prop)
        self.nPoints = np.max(self.vortexTABLE[:, -2])

    def display(self, color_main='blue', color_small='green', extra_points=None, extra_lines=None):
        bodies = self.props
        colors = [color_main]*len(self.props) 
        table_final = self.vortexTABLE
        #np.savetxt('table_final_blender.txt', table_final)
        bu.scene(bodies, colors, collocation_points = self.total_collocation_points, 
                total_velocity_vectors=self.total_velocity_vectors,
                axial_velocity_vectors=self.axial_velocity, 
                tangential_velocity_vectors=self.tangential_velocity, 
                extra_points=extra_points, extra_lines=extra_lines, savefig=True)
        
# def  defineQuad(filename):
#     with 
# # n = 15
# chord = np.ones(n)*0.02
# pitch = np.linspace(20, 10, n)
# quad = QuadCopter(R=0.4, prop_hub=0.1, prop_diameter=0.12,
#                     prop_NB=3, prop_pitch=pitch, prop_RPM=4000,
#                     prop_chord=chord, prop_n=n, wake_length=10,
#                     distribution='uniform', contraction=True, downwash=0.0)
# #quad.display()


def defineDrone(filename, main_U=None, small_U=None, main_RPM=None, small_RPM=None, plotGeom=False):
    helicopter, quadcopter = False, False
    # if plotGeom:
    #     filename = f'./{filename}'
    # else:
    #     filename = f'./configs/{filename}'
    with open(f'{filename}', 'r') as f:
    #with open(f'./{filename}', 'r') as f:

    #with open(filename, 'r') as f:
        config = json.load(f)

        LINEAR = config['settings']['linear']

        aircraft_type = config['main_propeller']['AIRCRAFT']

        if aircraft_type == 'helicopter':
            helicopter = True
        elif aircraft_type == 'quadcopter':
            helicopter = False
            quadcopter = True
        else:
            helicopter = False

        main_position = Point(*config['main_propeller']['position']) # general  
        main_angles = config['main_propeller']['angles']             # general
        main_hub = config['main_propeller']['hub']                   # general
        main_diameter = config['main_propeller']['diameter']
        main_NB = config['main_propeller']['NB']
        main_n = config['main_propeller']['n']
        main_chord_root = config['main_propeller']['chord_root']
        main_chord_tip = config['main_propeller']['chord_tip']
        main_pitch_root = config['main_propeller']['pitch_root']
        main_pitch_tip = config['main_propeller']['pitch_tip']
        main_airfoil = config['main_propeller']['airfoil']
        main_wake_length = config['main_propeller']['wake_length']
        main_r = np.linspace(main_hub, main_diameter/2, main_n)

        if LINEAR:
            main_pitch = np.linspace(main_pitch_root, main_pitch_tip, main_n-1) #+ main_optimal_AoA
            main_pitch = np.concatenate([main_pitch]*main_NB)
            main_chord = np.linspace(main_chord_root, main_chord_tip, main_n)
        else:
        # Pitch distribution
            pitch_incline = config['settings']['pitch_incline_main']
            main_pitch = parabolic_distribution(main_pitch_root, main_pitch_tip, main_r, pitch_incline)
            main_pitch = np.tile((main_pitch[0:-1] + main_pitch[1:])*0.5, main_NB)
            main_chord_a = config['settings']['main_chord_a']
            main_chord = parabolic_distribution(main_chord_root, main_chord_tip, main_r, main_chord_a)
            main_chord = middle(main_chord)


        #main_chord = np.linspace(main_chord_root, main_chord_tip, main_n)
        # Parabolic chord distribution
        
        main_chord = np.concatenate([main_chord]*main_NB)

        if main_U is None:
            main_U = config['main_propeller']['uWake']
        
        if main_RPM is None:
            main_RPM = config['main_propeller']['rpm']

        

        if aircraft_type == 'drone':
            if small_U is None:
                small_U = config['small_propellers']['uWake']
            if small_RPM is None:
                small_RPM = config['small_propellers']['rpm']

        
            small_props_angle = config['small_propellers']['angle']
            small_props_diameter = config['small_propellers']['diameter']
            small_props_NB = config['small_propellers']['NB']
            small_props_radius = small_props_diameter/2
            
            small_chord_root = config['small_propellers']['chord_root']
            small_chord_tip = config['small_propellers']['chord_tip']
            small_props_n = config['small_propellers']['n']
            small_props_hub = config['small_propellers']['hub']

            

            
            small_twist_root = config['small_propellers']['twist_root']
            small_twist_tip = config['small_propellers']['twist_tip']
            small_pitch = config['small_propellers']['pitch']
            small_AoA = config['small_propellers']['AoA']
            small_wake_length = config['small_propellers']['wake_length']

            small_r = np.linspace(small_props_hub, small_props_diameter/2, small_props_n)

            # Pitch distribution
            pitch_incline = config['settings']['pitch_incline_small']
            #small_props_twist = parabolic_distribution(small_twist_root, small_twist_tip, small_r, pitch_incline)
            small_props_twist = synthetic_twist_curve(small_r/small_r[-1], beta_root=small_twist_root, beta_tip=small_twist_tip,
                                                    bump_pos=0.2, bump_height=15, noise=0, seed=None)
            small_props_pitch = middle(small_props_twist)  + small_pitch
            # cmall chord distribution
            small_chord_a = config['settings']['small_chord_a']
            small_props_chord = parabolic_distribution(small_chord_root, small_chord_tip, small_r, small_chord_a)

            small_distribution = config['small_propellers']['distribution']
            blade1_angle = config['settings']['blade1_angle']
            downwash = config['settings']['downwash']
            small_airfoil = config['small_propellers']['airfoil']
        else:
            small_props_angle = None
            small_props_diameter = None
            small_props_NB = None
            small_props_hub = None
            small_props_n = None
            small_wake_length = None
            small_props_chord = None
            small_props_pitch = None
            small_distribution = 'cosine'
            blade1_angle = 0
            downwash = None
            small_airfoil = None


        contraction = config['settings']['contraction']
        wind_speed = config['settings']['wind_speed']
        wind_angle = config['settings']['wind_angle']
        wake_bend = config['settings']['wake_bend']
        output_dir = config['settings']['output_dir']
        #print("Output directory:", output_dir)

        wind = np.array([wind_speed*np.cos(wind_angle*np.pi/180),
                        0,
                        wind_speed*np.sin(wind_angle*np.pi/180)])
        
        reynolds = config['settings']['reynolds']

        core_size = config['settings']['core_size']

        main_distribution = config['main_propeller']['distribution']
        if quadcopter:
            diagonal = config['main_propeller']['diagonal']
            drone = QuadCopter(R=diagonal, prop_hub=main_hub, prop_diameter=main_diameter,
                                prop_NB=main_NB, prop_pitch=main_pitch, prop_RPM=main_RPM,
                                prop_chord=main_chord, prop_n=main_n, wake_length=main_wake_length,
                                distribution=main_distribution, contraction=contraction, downwash=downwash,
                                reynolds=reynolds, main_airfoil=main_airfoil, core_size=core_size)   
                               
        
        else:
            drone = Drone(main_position, main_angles, main_hub, main_diameter, 
                                    main_NB, main_pitch, main_RPM, main_chord, main_n, main_airfoil, small_airfoil,
                                    small_props_angle, small_props_diameter, small_props_NB, small_props_hub,
                                    small_RPM, small_props_chord, small_props_n, small_props_pitch,
                                    mainWakeLength=main_wake_length, smallWakeLength=small_wake_length, main_U=main_U, small_U = small_U, 
                                    main_distribution=main_distribution, small_distribution=small_distribution,
                                    contraction=contraction, wind=wind, reynolds=reynolds, core_size=core_size, helicopter=helicopter, blade1_angle=blade1_angle, 
                                    downwash=downwash, wake_bend=wake_bend, type= aircraft_type, output_dir=output_dir)

        return drone
    

#quad = defineDrone('base_quad.json')
# print(np.array(quad.props[0].collocationPoints))
# print(np.array(quad.props[1].collocationPoints))
# # print(quad.total_collocation_points)
# quad.display()