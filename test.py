import ctypes
import numpy as np

# Load the shared library
vector_lib = ctypes.CDLL("./myVelocity.dll")


# Define function argument types and return type
vector_lib.compute_3D_vector.argtypes = (ctypes.c_double, ctypes.c_double, ctypes.c_double, 
                                         ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                         ctypes.POINTER(ctypes.c_double))

def compute_3D_vector(x1, y1, z1, x2, y2, z2):
    # Prepare an array for the result (3 elements)
    result = (ctypes.c_double * 3)()

    # Call the C function
    vector_lib.compute_3D_vector(x1, y1, z1, x2, y2, z2, result)

    # Convert to NumPy array
    return np.array([result[0], result[1], result[2]])

# Example usage
point1 = np.array([1.0, 2.0, 3.0])
point2 = np.array([4.0, 5.0, 6.0])

result = compute_3D_vector(*point1, *point2)
print("Result from C function:", result)  # Expected: [3. 3. 3.]