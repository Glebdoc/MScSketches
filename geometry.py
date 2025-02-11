import numpy as np
import pyvista as pv
from bemUtils import*

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
    def __init__(self, point1, point2, Gamma):
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
        x = point.x
        y = point.y
        z = point.z

        crossMag = self.crossMag(x, y, z)
        if crossMag < 1e-6:
            return np.array([0, 0, 0])
        r1 = self.r1(x, y, z)
        if r1 < 1e-6:
            return np.array([0, 0, 0])
        r2 = self.r2(x, y, z)
        if r2 < 1e-6:
            return np.array([0, 0, 0])
        
        K = (self.Gamma/(4*np.pi*crossMag*crossMag)) * (self.dot1(x, y, z)/r1 - self.dot2(x, y, z)/r2)
        return K*np.array([self.crossX(x, y, z), self.crossY(x, y, z), self.crossZ(x, y, z)])
    
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
        

    def velocity(self, point):
        vort1 = 0
        vort3 = 0
        for i in range(len(self.leftset)):
            vort1 += self.leftset[i].velocity(point)
            vort3 += self.rightset[i].velocity(point)
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
    def __init__(self, position, angles, hub, diameter, NB, pitch, RPM, chord, n, U=2, wake_length=5):
        self.diameter = diameter
        self.angles = np.array(angles)
        self.NB = NB
        self.pitch = pitch
        self.RPM = RPM
        self.n = n
        self.U = U
        self.wake_length = wake_length
        self.chord = chord
        self.r = np.linspace(hub, 1 , n)*diameter/2
        self.assemble()
        self.rotate(*self.angles)
        self.translate(position)

        
    
    def assemble(self):
        # wake
        delta = self.diameter/self.U
        dt = np.arange(0, self.wake_length*self.diameter/self.U, delta/100)
        omega = 2*np.pi*self.RPM/60
        zw = -self.U*dt # + angle etc
        
        # left vortex
        horseShoes = []
        for j in range(self.NB):
            shift = j*2*np.pi/self.NB
            for i in range(self.n - 1):
                xwl = self.r[i]*np.sin(omega*dt) + self.chord 
                xwr = self.r[i+1]*np.sin(omega*dt) + self.chord
                ywl = self.r[i]*np.cos(omega*dt) # + angle etc
                ywr = self.r[i+1]*np.cos(omega*dt) # + angle etc
                leftVortex = [(Vortex( Point(self.chord, self.r[i], 0), Point(0, self.r[i], 0), 0))]
                rightVortex = [(Vortex(Point(0, self.r[i+1], 0), Point(self.chord, self.r[i+1], 0), 0))]
                wakeLeft = []
                wakeRight = []
                for k in range(len(xwl)-1):
                    aLeft = Point(xwl[k+1], ywl[k+1], zw[k+1])
                    bLeft = Point(xwl[k], ywl[k], zw[k])

                    aRight = Point(xwr[k], ywr[k], zw[k])
                    bRight = Point(xwr[k+1], ywr[k+1], zw[k+1])

                    wakeLeft.append(Vortex(aLeft, bLeft, 1))
                    wakeRight.append(Vortex(aRight, bRight, 1))

                leftVortex = leftVortex + wakeLeft
                rightVortex = rightVortex + wakeRight

                centralVortex = (Vortex(Point(0, self.r[i], 0), Point(0, self.r[i+1], 0), 0))
                horseShoe = HorseShoe(leftVortex, centralVortex, rightVortex)

                if j != 0:
                    R = rotation_matrix(0, 0, shift)
                    horseShoe.rotate(R)
                horseShoes.append(horseShoe)
                self.horseShoes = horseShoes
                
    def translate(self, translation):
        for horseShoe in self.horseShoes:
            horseShoe.translate(translation)
                

    def rotate(self, delta_x=0, delta_y=0, delta_z=0):
        self.angles = np.array([delta_x, delta_y, delta_z])
        R = rotation_matrix(*self.angles)
        for horseShoe in self.horseShoes:
            horseShoe.rotate(R)


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
    def __init__(self, main_position, main_angles, main_hub, main_diameter, main_NB, main_pitch, main_RPM, main_chord, main_n, 
                 small_props_angles, small_props_diameters, small_props_NB, 
                 small_props_RPM, small_props_chord, small_props_n):
        # Main propeller
        self.main_prop = Propeller(main_position, main_angles, main_hub, main_diameter, main_NB, main_pitch, main_RPM, 
                                   main_chord, main_n)
        
        # Small propellers
        self.small_props = []
        main_NB = self.main_prop.NB
        main_R = self.main_prop.diameter/2
        for i in range(main_NB):
            shift = i*2*np.pi/main_NB
            position = Point(main_R*np.sin(shift), main_R*np.cos(shift), 0)
            angles = (0,np.radians(90),shift)
            small_prop = Propeller(position, small_props_angles[i], 0, small_props_diameters[i], 
                                   small_props_NB[i], small_props_RPM[i], small_props_chord[i], 
                                   small_props_n[i])
            self.small_props.append(small_prop)

    def translate(self, translation):
        # Translate both main and small props
        self.main_prop.translate(translation)
        for small_prop in self.small_props:
            small_prop.translate(translation)
    
    def rotate(self, delta_x=0, delta_y=0, delta_z=0):
        # Rotate both main and small props
        self.main_prop.rotate(delta_x, delta_y, delta_z)
        for small_prop in self.small_props:
            small_prop.rotate(delta_x, delta_y, delta_z)

    def display(self, color_main='blue', color_small='green'):
        # Display main propeller and small propellers
        self.main_prop.display(color_main)
        
        for small_prop in self.small_props:
            small_prop.display(color_small)


