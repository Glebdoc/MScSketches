import numpy as np
import pyvista as pv
import bemUtils as bu
import json
import xfoilUtil as xfu
import matplotlib.pyplot as plt

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
                 contraction=True, blade1_angle=0, downwash=None, wake_bend = False):
        self.diameter = diameter
        self.position = position
        self.small = small
        self.angles = np.array(angles)
        self.NB = NB
        self.pitch = pitch
        self.RPM = RPM
        self.n = n
        self.U = U
        self.wake_bend = wake_bend
        self.downwash = downwash
        self.wake_length = wake_length
        self.chord = chord
        self.v_axial = None
        self.Ct = None
        self.distribution = distribution
        if self.distribution == 'cosine':
            print('using cosine')
            R = diameter*0.5
            theta = np.linspace(np.arccos(hub/R), np.pi/10,  n)
            spacing = np.cos(0.5*theta)
            spacing = (spacing - np.min(spacing)) / (np.max(spacing) - np.min(spacing)) 
            spacing = spacing *(R - hub) + hub
            self.r = spacing
        else:
            self.r = np.linspace(hub, diameter*0.5 , n)
        self.azimuth = np.array([0, 0, 1])
        self.origin = position
        self.collocationPoints = [np.vstack((np.zeros(n-1), (self.r[:-1]+self.r[1:])*0.5, np.zeros(n-1)))]
        self.vortexTABLE = []
        self.horseShoes = None
        self.sigma = np.average(self.NB*self.chord[:n]/(np.pi*self.r))
        self.ppr=20
        self.bodyIndex = bodyIndex
        self.assemble(main_rotor=main_rotor, contraction=contraction)
        if self.small:
            """
            We first need to rotate around Z to pic a desired blade position with respect to the main rotor.
            Then we rotate around Y - simulating the naccelle incline 
            Then we bend the wake modifying 
            """
            temp_angles = np.array([0, 0, blade1_angle*np.pi/180])
            self.rotate(extra=True, a=temp_angles[0], b=temp_angles[1], c=temp_angles[2])
            if self.wake_bend:
                self.bendSmallWake(main_rotor, downwash=True)
        self.rotate()
        self.translate(position, main_rotor=main_rotor)
        
    def downwash(self):
        points = self.vortexTABLE[:, :6]
        points = points.reshape(-1, 3)

    def bendSmallWake(self, main_rotor, downwash=False):

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
        if main_rotor is not None:
            n_main = main_rotor.n
            main_NB = main_rotor.NB

        ppr = self.ppr 
        omega = 2*np.pi*self.RPM/60
        length = self.diameter*self.wake_length
        total_time = length/self.U
        nrevs = total_time*omega/(2*np.pi)
        total_steps = int(nrevs*ppr)

        table = []

        dt = np.linspace(0, total_time, total_steps)
        zw = -self.U*dt 

        try:
            v_axial_orig = np.genfromtxt('./auxx/v_axial.txt')
            if self.bodyIndex == 0:
                v_axial = median_filter(v_axial_orig[:self.n-1].copy(), kernel_size=3, times=1)
                v_axial = np.tile(v_axial, (self.NB))
            else:
                start = main_NB*(n_main-1) + (self.n-1)*self.NB*(self.bodyIndex-1)
                end = main_NB*(n_main-1) + (self.n-1)*self.NB*self.bodyIndex
                v_axial = v_axial_orig[start:end]


        except:
            v_axial = None

        if v_axial is None:
            #my_U = np.linspace(-1, -3, self.n-1)
            omega = 2*np.pi*self.RPM/60
            solidity = self.NB*self.chord[:self.n-1].flatten()/(np.pi*self.r[:self.n-1])
            twist_local = np.radians(self.pitch[:self.n-1])
            cla = 0.5
            inflowratio = solidity*cla/16*(np.sqrt(1 + 32*twist_local/(cla*solidity)) - 1)
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

    def translate(self, translation, main_rotor=None):
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
            collocs[0,:] += translation.x
            collocs[1,:] += translation.y
            collocs[2,:] += translation.z
            self.collocationPoints[i] = collocs


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


        self.azimuth = R @ self.azimuth

#####################################################################

    # def rotate(self, delta_x=0, delta_y=0, delta_z=0, extra=False):
        
    #     if not extra:
    #         self.angles = np.array([delta_x, delta_y, delta_z])

    #     R = bu.rotation_matrix(*self.angles)

    #     table = self.vortexTABLE

    #     points = table[:, :6].reshape(-1, 3) 
    #     points = (R @ points.T).T 
    #     table[:, :6] = points.reshape(-1, 6)
    #     self.vortexTABLE = table


    #     for i in range(self.NB):
    #         self.collocationPoints[i] = R @ self.collocationPoints[i]


    #     self.azimuth = R @ self.azimuth

####################################################################



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
                 main_NB, main_pitch, main_RPM, main_chord, main_n, main_airfoil,
                 small_props_angle, small_props_diameter, small_props_NB, small_props_hub,
                 small_props_RPM, small_props_chord, small_props_n, small_props_pitch,
                 mainWakeLength, smallWakeLength, main_U, small_U, main_distribution, 
                 small_distribution, 
                 helicopter=False, contraction=True, wind=None, reynolds=False, core_size = 1e-5, 
                 blade1_angle=0, downwash=None, wake_bend=False):
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
                                   contraction=contraction
                                   )
        
        # Small propellers
        self.core_size = core_size
        self.main_prop.airfoil = main_airfoil
        self.small_props = []
        self.wind = wind
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
                                    downwash=downwash, wake_bend=wake_bend)
                
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
        bu.scene(bodies, colors, collocation_points = self.total_collocation_points, 
              total_velocity_vectors=self.total_velocity_vectors,
              axial_velocity_vectors=self.axial_velocity, 
              tangential_velocity_vectors=self.tangential_velocity, 
              extra_points=extra_points, extra_lines=extra_lines)
        

def defineDrone(filename, main_U=None, small_U=None, main_RPM=None, small_RPM=None):
    with open(f'./configs/{filename}', 'r') as f:
        config = json.load(f)

        aircraft_type = config['main_propeller']['AIRCRAFT']

        if aircraft_type == 'helicopter':
            helicopter = True
        else:
            helicopter = False

        main_position = Point(*config['main_propeller']['position'])
        main_angles = config['main_propeller']['angles']
        main_hub = config['main_propeller']['hub']
        main_diameter = config['main_propeller']['diameter']
        main_NB = config['main_propeller']['NB']
        main_n = config['main_propeller']['n']
        main_chord_root = config['main_propeller']['chord_root']
        main_chord_tip = config['main_propeller']['chord_tip']
        main_pitch_root = config['main_propeller']['pitch_root']
        main_pitch_tip = config['main_propeller']['pitch_tip']
        main_airfoil = config['main_propeller']['airfoil']
        main_wake_length = config['main_propeller']['wake_length']

        main_pitch = np.linspace(main_pitch_root, main_pitch_tip, main_n-1) #+ main_optimal_AoA
        main_pitch = np.concatenate([main_pitch]*main_NB)

        main_chord = np.linspace(main_chord_root, main_chord_tip, main_n)
        main_chord = np.concatenate([main_chord]*main_NB)

        if main_U is None:
            main_U = config['main_propeller']['uWake']
        
        if main_RPM is None:
            main_RPM = config['main_propeller']['rpm']

        

        if helicopter!= True:
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

            

            
            small_pitch_root = config['small_propellers']['pitch_root']
            small_pitch_tip = config['small_propellers']['pitch_tip']
            small_AoA = config['small_propellers']['AoA']
            small_wake_length = config['small_propellers']['wake_length']

            small_r = np.linspace(small_props_hub, small_props_diameter/2, small_props_n-1)

            # Pitch distribution
            A = [[small_props_hub**2, small_props_hub, 1],
                 [small_props_radius**2, small_props_radius, 1],
                 [small_props_radius*2, 1, 0]]
            Y = [small_pitch_root, small_pitch_tip, 0]
            a, b, c = np.linalg.solve(A, Y)
            x = np.linspace(small_props_hub, small_props_radius, small_props_n-1)
            small_props_pitch = a*x**2 + b*x + c

            #small_props_pitch = bu.twistGen(small_pitch_root, small_pitch_tip, small_r, small_AoA)
            #small_props_pitch = np.linspace(small_pitch_root, small_pitch_tip, small_props_n-1)
            small_props_chord = np.linspace(small_chord_root, small_chord_tip, small_props_n)
            small_distribution = config['small_propellers']['distribution']
            blade1_angle = config['settings']['blade1_angle']
            downwash = config['settings']['downwash']
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


        contraction = config['settings']['contraction']
        wind_speed = config['settings']['wind_speed']
        wind_angle = config['settings']['wind_angle']
        wake_bend = config['settings']['wake_bend']

        wind = np.array([wind_speed*np.cos(wind_angle*np.pi/180),
                        0,
                        wind_speed*np.sin(wind_angle*np.pi/180)])
        
        reynolds = config['settings']['reynolds']

        core_size = config['settings']['core_size']

        main_distribution = config['main_propeller']['distribution']
        
        drone = Drone(main_position, main_angles, main_hub, main_diameter, 
                                    main_NB, main_pitch, main_RPM, main_chord, main_n, main_airfoil,
                                    small_props_angle, small_props_diameter, small_props_NB, small_props_hub,
                                    small_RPM, small_props_chord, small_props_n, small_props_pitch,
                                    mainWakeLength=main_wake_length, smallWakeLength=small_wake_length, main_U=main_U, small_U = small_U, 
                                    main_distribution=main_distribution, small_distribution=small_distribution, 
                                    contraction=contraction, wind=wind, reynolds=reynolds, core_size=core_size, helicopter=helicopter, blade1_angle=blade1_angle, 
                                    downwash=downwash, wake_bend=wake_bend)

        return drone