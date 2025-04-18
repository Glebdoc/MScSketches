import numpy as np
import pyvista as pv
import bemUtils as bu
import timeit
import json
import xfoilUtil as xfu


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


class Propeller():
    def __init__(self, position, angles, hub, diameter, NB, pitch, RPM, chord, n, U=2, wake_length=5, distribution='cosine', bodyIndex=0, small=False, main_rotor=None, contraction=True):
        self.diameter = diameter
        self.position = position
        self.small = small
        self.angles = np.array(angles)
        self.NB = NB
        self.pitch = pitch
        self.RPM = RPM
        self.n = n
        self.U = U
        self.wake_length = wake_length
        self.chord = chord
        distribution = 'cosine'
        if distribution == 'cosine':
            R = diameter*0.5
            theta = np.linspace(np.arccos(hub/R), np.pi/8,  n)
            spacing = np.cos(0.5*theta)
            spacing = (spacing - np.min(spacing)) / (np.max(spacing) - np.min(spacing)) 
            spacing = spacing *(R - hub) + hub
            self.r = spacing
        else:
            self.r = np.linspace(hub, 1 , n)*diameter*0.5
        self.azimuth = np.array([0, 0, 1])
        self.origin = position
        self.collocationPoints = [np.vstack((np.zeros(n-1), (self.r[:-1]+self.r[1:])*0.5, np.zeros(n-1)))]
        #self.collocationPoints = [np.vstack((self.chord[:self.n-1]*0.5, (self.r[:-1]+self.r[1:])*0.5, np.zeros(n-1)))]
        self.vortexTABLE = []
        self.horseShoes = None
        self.bodyIndex = bodyIndex
        self.assemble(main_rotor=None, contraction=contraction)
        if self.small:
            self.bendSmallWake()
        self.rotate(*self.angles)
        self.translate(position)
        self.fillVotexTable()

    def fillVotexTable(self):
        for i in range(len(self.horseShoes)):
            horse = self.horseShoes[i]
            for j in range(len(horse.leftset)):
                local_vortex = horse.leftset[j] 
                self.vortexTABLE.append([local_vortex.x1, local_vortex.y1, local_vortex.z1, local_vortex.x2, local_vortex.y2, local_vortex.z2, i, local_vortex.Gamma])
                local_vortex = horse.rightset[j]
                self.vortexTABLE.append([local_vortex.x1, local_vortex.y1, local_vortex.z1, local_vortex.x2, local_vortex.y2, local_vortex.z2, i, local_vortex.Gamma])
            local_vortex = horse.centre
            self.vortexTABLE.append([local_vortex.x1, local_vortex.y1, local_vortex.z1, local_vortex.x2, local_vortex.y2, local_vortex.z2, i, local_vortex.Gamma])
        self.vortexTABLE = np.array(self.vortexTABLE)
        
    def bendSmallWake(self):

        zw = np.array(self.zw)
        tempR = np.sqrt(self.position.x**2 + self.position.y**2)

        for i in range(len(self.horseShoes)):
            horse = self.horseShoes[i]
            for j in range(len(horse.leftset)):
                if j==0:
                    continue
                #Update left vortex
                vortex = horse.leftset[j]

                vortex.p1.y += tempR
                vortex.p2.y += tempR

                r_to_point_1 = np.sqrt(vortex.p1.z**2 + vortex.p1.y**2)
                r_to_point_2 = np.sqrt(vortex.p2.z**2 + vortex.p2.y**2)

                theta= np.arctan(zw[j]/tempR)
                r_to_axis = np.sqrt(tempR**2 + zw[j]**2)

                deltaR1 = r_to_point_1 - r_to_axis
                deltaR2 = r_to_point_2 - r_to_axis

                r_new1 = tempR + deltaR1
                r_new2 = tempR + deltaR2

                vortex.p1.z = r_new1 * np.sin(theta) 
                vortex.p1.y = r_new1 * np.cos(theta) - tempR

                vortex.p2.z = r_new2 * np.sin(theta)
                vortex.p2.y = r_new2 * np.cos(theta) - tempR

                #Update right vortex
                vortex = horse.rightset[j]

                vortex.p1.y += tempR
                vortex.p2.y += tempR

                r_to_point_1 = np.sqrt(vortex.p1.z**2 + vortex.p1.y**2)
                r_to_point_2 = np.sqrt(vortex.p2.z**2 + vortex.p2.y**2)

                theta= np.arctan(zw[j]/tempR)
                r_to_axis = np.sqrt(tempR**2 + zw[j]**2)

                deltaR1 = r_to_point_1 - r_to_axis
                deltaR2 = r_to_point_2 - r_to_axis

                r_new1 = tempR + deltaR1
                r_new2 = tempR + deltaR2

                vortex.p1.z = r_new1 * np.sin(theta) 
                vortex.p1.y = r_new1 * np.cos(theta) - tempR

                vortex.p2.z = r_new2 * np.sin(theta)
                vortex.p2.y = r_new2 * np.cos(theta) - tempR



    def assemble(self, main_rotor=None, contraction=True):
        ppr = 30 
        omega = 2*np.pi*self.RPM/60
        length = self.diameter*self.wake_length
        total_time = length/self.U
        nrevs = total_time*omega/(2*np.pi)
        total_steps = int(nrevs*ppr)

        dt = np.linspace(0, total_time, total_steps)
        zw = -self.U*dt 
        self.zw = zw

        # left vortex
        horseShoes = []

        if contraction:
            mult = bu.contraction_sigmoid(zw, contraction=0.55)
        else:
            mult = np.ones(len(zw))
        

        for j in range(self.NB):
            shift = j*2*np.pi/self.NB
            if j != 0:
                R = bu.rotation_matrix(0, 0, shift)
                self.collocationPoints.extend(R @ self.collocationPoints[0:self.n-1])

            for i in range(self.n - 1):
                xwl = self.r[i]*np.sin(omega*dt)*mult + self.chord[i]
                xwr = self.r[i+1]*np.sin(omega*dt)*mult + self.chord[i+1]
                ywl = self.r[i]*np.cos(omega*dt)*mult # + angle etc
                ywr = self.r[i+1]*np.cos(omega*dt)*mult # + angle etc
                leftVortex = [(Vortex( Point(self.chord[i], self.r[i], 0), Point(0, self.r[i], 0), 1))] #removed 0 
                rightVortex = [(Vortex(Point(0, self.r[i+1], 0), Point(self.chord[i+1], self.r[i+1], 0),1))] # removed 0 
                wakeLeft = []
                wakeRight = []
                
                for k in range(len(xwl)-1):

                    # playing with contraction 
                    aLeft = Point(xwl[k+1], ywl[k+1], zw[k+1])
                    bLeft = Point(xwl[k], ywl[k], zw[k])
                    aRight = Point(xwr[k], ywr[k], zw[k])
                    bRight = Point(xwr[k+1], ywr[k+1], zw[k+1])

                    wakeLeft.append(Vortex(aLeft, bLeft, 1))
                    wakeRight.append(Vortex(aRight, bRight, 1))

                leftVortex = leftVortex + wakeLeft
                rightVortex = rightVortex + wakeRight

                centralVortex = (Vortex(Point(0, self.r[i], 0), Point(0, self.r[i+1], 0),1)) # removed 0 
                horseShoe = HorseShoe(leftVortex, centralVortex, rightVortex)
                if j != 0:
                    horseShoe.rotate(R)
                horseShoes.append(horseShoe)
        self.horseShoes = horseShoes

    def translate(self, translation):
        translation_array = np.array([translation.x, translation.y, translation.z]).reshape(3, 1)

        for horseShoe in self.horseShoes:
            horseShoe.translate(translation)

        for i in range(self.NB):
            self.collocationPoints[i] += translation_array
                

    def rotate(self, delta_x=0, delta_y=0, delta_z=0):
        

        self.angles = np.array([delta_x, delta_y, delta_z])

        R = bu.rotation_matrix(*self.angles)

        for horseShoe in self.horseShoes:
            horseShoe.rotate(R)

        for i in range(self.NB):
            self.collocationPoints[i] = R @ self.collocationPoints[i]

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
                 main_NB, main_pitch, main_RPM, main_chord, main_n, main_airfoil,
                 small_props_angle, small_props_diameter, small_props_NB, small_props_hub,
                 small_props_RPM, small_props_chord, small_props_n, small_props_pitch,
                 mainWakeLength, smallWakeLength, main_U, small_U, main_distribution='uniform', 
                 small_distribution='uniform', 
                 helicopter=False, contraction=True, wind=None, reynolds=False):
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
        self.main_prop.airfoil = main_airfoil
        self.small_props = []
        self.wind = wind
        self.reynolds = reynolds
        self.total_velocity_vectors = None
        self.total_collocation_points = None
        self.axial_velocity = None
        self.tangential_velocity = None
        self.vortexTABLE = self.main_prop.vortexTABLE

        main_NB = self.main_prop.NB
        main_R = self.main_prop.diameter/2
        if helicopter==False:
            for i in range(main_NB):
                shift = i*(2*np.pi/main_NB)
                position = Point(main_R*np.sin(shift), main_R*np.cos(shift), 0.05) # try 0.05

                angles = np.array([0,-small_props_angle*np.pi/180,-shift])
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
                                    small=True, main_rotor=self.main_prop, contraction=contraction)
                
                table = small_prop.vortexTABLE
                table[:,-1]+=(self.vortexTABLE[-1,-1] + 1) 

                self.vortexTABLE = np.concatenate((self.vortexTABLE, table), axis=0)

                self.small_props.append(small_prop)

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

        if main_U is None:
            main_U = config['main_propeller']['uWake']
        if small_U is None:
            small_U = config['small_propellers']['uWake']
        if main_RPM is None:
            main_RPM = config['main_propeller']['rpm']
        if small_RPM is None:
            small_RPM = config['small_propellers']['rpm']

        Re_avg = 1.225*main_RPM*0.7*0.5*main_diameter * 2*np.pi/60 * 0.5*(main_chord_tip+main_chord_root)/1.81e-5
        main_optimal_AoA = xfu.optimalAoA(main_airfoil, Re_avg)

        main_pitch = np.linspace(main_pitch_root, main_pitch_tip, main_n-1) + main_optimal_AoA
        main_pitch = np.concatenate([main_pitch]*main_NB)

        main_chord = np.linspace(main_chord_root, main_chord_tip, main_n-1)
        main_chord = np.concatenate([main_chord]*main_NB)



        small_props_angle = config['small_propellers']['angle']
        small_props_diameter = config['small_propellers']['diameter']
        small_props_NB = config['small_propellers']['NB']
        
        small_chord_root = config['small_propellers']['chord_root']
        small_chord_tip = config['small_propellers']['chord_tip']
        small_props_n = config['small_propellers']['n']
        small_props_hub = config['small_propellers']['hub']
        
        small_pitch_root = config['small_propellers']['pitch_root']
        small_pitch_tip = config['small_propellers']['pitch_tip']
        small_AoA = config['small_propellers']['AoA']
        small_wake_length = config['small_propellers']['wake_length']

        small_r = np.linspace(small_props_hub, small_props_diameter/2, small_props_n-1)

        small_props_pitch = bu.twistGen(small_pitch_root, small_pitch_tip, small_r, small_AoA)
        small_props_chord = np.linspace(small_chord_root, small_chord_tip, small_props_n)

        contraction = config['settings']['contraction']
        wind_speed = config['settings']['wind_speed']
        wind_angle = config['settings']['wind_angle']

        wind = np.array([wind_speed*np.cos(wind_angle*np.pi/180),
                        0,
                        wind_speed*np.sin(wind_angle*np.pi/180)])
        
        reynolds = config['settings']['reynolds']

        


        drone = Drone(main_position, main_angles, main_hub, main_diameter, 
                                    main_NB, main_pitch, main_RPM, main_chord, main_n, main_airfoil,
                                    small_props_angle, small_props_diameter, small_props_NB, small_props_hub,
                                    small_RPM, small_props_chord, small_props_n, small_props_pitch,
                                    mainWakeLength=main_wake_length, smallWakeLength=small_wake_length, main_U=main_U, small_U = small_U, 
                                    main_distribution='uniform', small_distribution='uniform', 
                                    contraction=contraction, wind=wind, reynolds=reynolds)

        # write optimal AoA to config file
        config['main_propeller']['optimal_AoA'] = main_optimal_AoA
        with open(f'./configs/{filename}', 'w') as f:
            json.dump(config, f, indent=4)

        return drone