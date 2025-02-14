import numpy as np 

def computeVelocityField():

    #define a plane and discretize it 
    #I want to create a Z-Y plane
    #I will create a grid of points in this plane
    #I will calculate the velocity field at each point
    #I will plot the velocity field using pyvista 

    #define the plane
    discretization = 0.1 
    z = np.arange(-3, 3, discretization)
    y = np.arange(-3, 3, discretization)

    point_coordinates = np.zeros((len(z)*len(y), 3))

    #create the grid of points
    for i in range(len(z)):
        for j in range(len(y)):
            point_coordinates[i*len(y) + j] = np.array([0, y[j], z[i]])
    
    #calculate the velocity field at each point
    