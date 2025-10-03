import numpy as np 
import ctypes, os
import matplotlib.pyplot as plt
from bemUtils import set_bw_design


# read c library 
if os.name == 'nt':
    mylib = ctypes.CDLL("./mylib.dll")  
else:
    mylib = ctypes.CDLL("./mylib.so")

core = 1e-5
N = 1 
T = 1

# table 
table = np.array([[0.0, -0.3, 0.0, 0.0, 0.3, 0.0, 0, 1.0 ]], dtype=np.float64) # start(x,y,z), end(x,y,z)

n = 20
Z = np.linspace(0, 1.0, n)
vtan = np.zeros(n)
design = set_bw_design()
print(design)
count=0
Gamma = np.linspace(0.1, 10.0, 4)
for core in [1e-7, 1e-7, 1e-7, 1e-7]:
   
    for i in range(len(Z)):
        point = np.array([[0.0, 0.0, Z[i]]], dtype=np.float64) # x,y,z

        table = np.array([[0.0, -0.3, 0.0, 0.0, 0.3, 0.0, 0, Gamma[count] ]], dtype=np.float64) # start(x,y,z), end(x,y,z)
        core_size = ctypes.c_float(core)
        u_influences = np.zeros((N,T), dtype=np.float64)
        v_influences = np.zeros((N,T), dtype=np.float64)
        w_influences = np.zeros((N,T), dtype=np.float64)

        collocationPoints_ptr = point.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        vortexTable_ptr = table.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        uInfluence_ptr = u_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        vInfluence_ptr = v_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        wInfluence_ptr = w_influences.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

        res = mylib.computeInfluenceMatrices(N, T, collocationPoints_ptr, vortexTable_ptr, uInfluence_ptr, vInfluence_ptr, wInfluence_ptr, core_size)
        vtan[i] = u_influences[0,0]

    count+=1
    plt.plot(Z, vtan, 
             label=r'$\Gamma=$'+ f'{Gamma[count-1]:.1f} m$^2$/s', 
             marker=design['markers'][count%len(design)],
             linestyle=design['line_styles'][count%len(design)])
plt.xlabel('r (m)')
plt.ylabel(r'$V_{\theta} (m/s)$')
plt.legend()
plt.show()
