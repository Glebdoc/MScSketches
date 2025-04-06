import ctypes
import numpy as np
from geometry import*

# Setting up the test data 


# Load the shared library
mylib = ctypes.CDLL("./mylib.dll")  # Use "mylib.dll" on Windows

collocationPoints = np.array([
    [0.1, 0.2, 0.3], 
    [0.4, 0.5, 0.6], 
    [0.7, 0.8, 0.9]
], dtype=np.float64)

N = len(collocationPoints)  # Number of collocation points
# Define the influence matrices
uInfluence = np.zeros((N,N), dtype=np.float64)
vInfluence = np.zeros((N,N), dtype=np.float64)
wInfluence = np.zeros((N,N), dtype=np.float64)

vortexTable = np.array([
    [1, 1, 1, 2, 2, 2, 0, 1],
    [3, 3, 3, 4, 4, 4, 0, 1],
    [5, 5, 5, 6, 6, 6, 1, 1]
], dtype=np.float64)

T = vortexTable.shape[0]  # Number of vortices
# Call the function


collocationPoints_ptr = collocationPoints.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
vortexTable_ptr = vortexTable.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
uInfluence_ptr = uInfluence.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
vInfluence_ptr = vInfluence.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
wInfluence_ptr = wInfluence.ctypes.data_as(ctypes.POINTER(ctypes.c_double))



res = mylib.computeInfluenceMatrices(N, T, collocationPoints_ptr, vortexTable_ptr, 
                                     uInfluence_ptr, vInfluence_ptr, wInfluence_ptr)
print("C function returned:", res)
print("uInfluence:", uInfluence)
print("vInfluence:", vInfluence)
print("wInfluence:", wInfluence)