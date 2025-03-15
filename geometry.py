import numpy as np
import pyvista as pv
from bemUtils import*
import timeit


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

    def crossX(self, x, y, z):
        return (y - self.y1)*(z - self.z2) - (z - self.z1)*(y - self.y2)
    
    def crossY(self, x, y, z):
        return (z - self.z1)*(x - self.x2) - (x - self.x1)*(z - self.z2)
    
    def crossZ(self, x, y, z):
        return (x - self.x1)*(y - self.y2) - (y - self.y1)*(x - self.x2)
    
    def crossMag(self, x, y, z):
        return np.sqrt(self.crossX(x, y, z)**2 + self.crossY(x, y, z)**2 + self.crossZ(x, y, z)**2)


    def dot1(self, x, y, z):
        return (self.x2 - self.x1)*(x - self.x1) + (self.y2 - self.y1)*(y - self.y1) + (self.z2 - self.z1)*(z - self.z1)
    
    def dot2(self, x, y, z):
        return (self.x2 - self.x1)*(x - self.x2) + (self.y2 - self.y1)*(y - self.y2) + (self.z2 - self.z1)*(z - self.z2)
    
    def r1(self, x, y, z):
        return np.sqrt((x - self.x1)**2 + (y - self.y1)**2 + (z - self.z1)**2)
    
    def r2(self, x, y, z): 
        return np.sqrt((x - self.x2)**2 + (y - self.y2)**2 + (z - self.z2)**2)
    

    def length(self):
        return np.sqrt((self.x2 - self.x1)**2 + (self.y2 - self.y1)**2)
    
    def velocity(self, point):
        """Optimized velocity function maintaining correctness."""
        x, y, z = point.x, point.y, point.z

        cross_vec = np.array([self.crossX(x, y, z), self.crossY(x, y, z), self.crossZ(x, y, z)])

        # Compute cross magnitude
        #crossMag = self.crossMag(x, y, z)
        crossMag =np.linalg.norm(cross_vec, axis=0)
        if crossMag < 1e-6:
            return np.zeros(3)

        # Compute r1 and r2
        r1, r2 = self.r1(x, y, z), self.r2(x, y, z)
        if r1 < 1e-6 or r2 < 1e-6:
            return np.zeros(3)

        # Compute dot products
        dot1, dot2 = self.dot1(x, y, z), self.dot2(x, y, z)

        # Compute K factor
        K = (self.Gamma / (4 * np.pi * crossMag**2)) * (dot1 / r1 - dot2 / r2)

        

        # Compute cross product result
        

        return K * cross_vec

    def velocity_vectorized(self, points):
        x, y, z = points[:, 0], points[:, 1], points[:, 2]

        cross_vec = np.array([self.crossX(x, y, z), self.crossY(x, y, z), self.crossZ(x, y, z)]).T

        # Compute cross magnitude
        #crossMag = self.crossMag(x, y, z)
        crossMag = np.linalg.norm(cross_vec, axis=1)

        mask = crossMag < 1e-6
        crossMag[mask] = 1
        # Compute r1 and r2
        r1, r2 = self.r1(x, y, z), self.r2(x, y, z)
        r1[r1 < 1e-6] = 1e-5
        r2[r2 < 1e-6] = 1e-5

        # Compute dot products
        dot1, dot2 = self.dot1(x, y, z), self.dot2(x, y, z)

        # Compute K factor
        K = (self.Gamma / (4 * np.pi * crossMag**2)) * (dot1 / r1 - dot2 / r2)

        K[mask] = 0

        # Compute cross product result
        

        return K[:, np.newaxis] * cross_vec

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
        

    def velocity(self, point, vectorized=False):
        vort1 = 0
        vort3 = 0
        for i in range(len(self.leftset)):
            if vectorized:
                vort1 += self.leftset[i].velocity_vectorized(point)
                vort3 += self.rightset[i].velocity_vectorized(point)
            else:
                vort1 += self.leftset[i].velocity(point)
                vort3 += self.rightset[i].velocity(point)
        if vectorized:
            vort2 = self.centre.velocity_vectorized(point)
        else:
            vort2 = self.centre.velocity(point)
        return vort1 + vort2 + vort3

    
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
    def __init__(self, position, angles, hub, diameter, NB, pitch, RPM, chord, n, U=2, wake_length=5, distribution='uniform', bodyIndex=0, small=False, main_rotor=None):
        self.diameter = diameter
        self.small = small
        self.angles = np.array(angles)
        self.NB = NB
        self.pitch = pitch
        self.RPM = RPM
        self.n = n
        self.U = U
        self.wake_length = wake_length
        self.chord = chord
        if distribution == 'cosine':
            self.r = np.linspace(hub, 1 , n)*diameter*0.5*np.cos(np.linspace(0, np.pi, n))
        else:
            self.r = np.linspace(hub, 1 , n)*diameter*0.5
        self.azimuth = np.array([0, 0, 1])
        self.origin = position
        self.collocationPoints = [np.vstack((np.zeros(n-1), (self.r[:-1]+self.r[1:])*0.5, np.zeros(n-1)))]
        self.vortexTABLE = []
        self.horseShoes = None
        self.bodyIndex = bodyIndex
        self.assemble(main_rotor=None)
        if self.small:
            self.bendSmallWake()
        self.rotate(*self.angles)
        self.translate(position)
        
    def bendSmallWake(self):
        ppr = 30 
        omega = 2*np.pi*self.RPM/60
        length = self.diameter*self.wake_length
        total_time = length/self.U
        nrevs = total_time*omega/(2*np.pi)
        total_steps = int(nrevs*ppr)
        dt = np.linspace(0, total_time, total_steps)
        zw = -self.U*dt

        tempR = 0.7
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



    def assemble(self, main_rotor=None):
        # wake
        # I want to have D number of points per revolution of the propeller
        # I call them points per rev (ppr)
        ppr = 30 
        omega = 2*np.pi*self.RPM/60
        length = self.diameter*self.wake_length
        total_time = length/self.U
        # how much time does 1 rev take?
        # or rather how many revs can I fit in the wake?
        nrevs = total_time*omega/(2*np.pi)
        total_steps = int(nrevs*ppr)


        dt = np.linspace(0, total_time, total_steps)
        zw = -self.U*dt # + angle etc
        self.zw = zw
        # left vortex
        horseShoes = []

        for j in range(self.NB):
            shift = j*2*np.pi/self.NB
            if j != 0:
                R = rotation_matrix(0, 0, shift)
                self.collocationPoints.extend(R @ self.collocationPoints[0:self.n-1])


            for i in range(self.n - 1):
                xwl = self.r[i]*np.sin(omega*dt) + self.chord[i]
                xwr = self.r[i+1]*np.sin(omega*dt) + self.chord[i+1]
                ywl = self.r[i]*np.cos(omega*dt) # + angle etc
                ywr = self.r[i+1]*np.cos(omega*dt) # + angle etc
                leftVortex = [(Vortex( Point(self.chord[i], self.r[i], 0), Point(0, self.r[i], 0), 0))]
                self.vortexTABLE.append([self.chord[i], self.r[i], 0, 0, self.r[i], 0, self.bodyIndex*(self.n*self.NB) + j*self.n + i])
                rightVortex = [(Vortex(Point(0, self.r[i+1], 0), Point(self.chord[i+1], self.r[i+1], 0), 0))]
                self.vortexTABLE.append([0, self.r[i+1], 0, self.chord[i+1], self.r[i+1], 0, self.bodyIndex*(self.n*self.NB) + j*self.n + i])
                wakeLeft = []
                wakeRight = []

                for k in range(len(xwl)-1):
                    
    
                    aLeft = Point(xwl[k+1], ywl[k+1], zw[k+1])
                    bLeft = Point(xwl[k], ywl[k], zw[k])

                    aRight = Point(xwr[k], ywr[k], zw[k])
                    bRight = Point(xwr[k+1], ywr[k+1], zw[k+1])

                    wakeLeft.append(Vortex(aLeft, bLeft, 1))
                    self.vortexTABLE.append([xwl[k+1], ywl[k+1], zw[k+1], xwl[k], ywl[k], zw[k], self.bodyIndex*(self.n*self.NB) + j*self.n + i])
                    wakeRight.append(Vortex(aRight, bRight, 1))
                    self.vortexTABLE.append([xwr[k], ywr[k], zw[k], xwr[k+1], ywr[k+1], zw[k+1], self.bodyIndex*(self.n*self.NB) + j*self.n + i])

                leftVortex = leftVortex + wakeLeft
                rightVortex = rightVortex + wakeRight

                centralVortex = (Vortex(Point(0, self.r[i], 0), Point(0, self.r[i+1], 0), 0))
                self.vortexTABLE.append([0, self.r[i], 0, 0, self.r[i+1], 0, self.bodyIndex*(self.n*self.NB) + j*self.n + i])
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
        R = rotation_matrix(*self.angles)

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

# drone = Drone(main_position, main_angles, main_hub, main_diameter, main_NB, main_pitch, main_RPM, main_chord, main_n,
#               small_props_angles, small_props_diameter, small_props_NB,
#               small_props_RPM, small_props_chord, small_props_n)

class Drone:
    def __init__(self, main_position, main_angles, main_hub, main_diameter,
                 main_NB, main_pitch, main_RPM, main_chord, main_n, 
                 small_props_angle, small_props_diameter, small_props_NB, small_props_hub,
                 small_props_RPM, small_props_chord, small_props_n, small_props_pitch,
                 mainWakeLength, smallWakeLength, main_U, small_U, main_distribution='uniform', small_distribution='uniform', helicopter=False):
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
                                   distribution=main_distribution)
        
        # Small propellers
        self.small_props = []
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
                position = Point(main_R*np.sin(shift), main_R*np.cos(shift), 0)

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
                                    small=True, main_rotor=self.main_prop)
                self.small_props.append(small_prop)
                self.vortexTABLE.extend(small_prop.vortexTABLE)
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
        scene(bodies, colors, collocation_points = self.total_collocation_points, 
              total_velocity_vectors=self.total_velocity_vectors,
              axial_velocity_vectors=self.axial_velocity, 
              tangential_velocity_vectors=self.tangential_velocity, 
              extra_points=extra_points, extra_lines=extra_lines)