import sys
sys.path.append('../build/')

import numpy as np
import pybind_example as pe

print
print("print_numpy")
print
v = np.ones((2,))
print(v)
pe.print_numpy( v )

print
print("print_stl_numpy")
print
vs = []
for i in xrange(5):
    vs += [np.random.uniform(low=0,high=5,size=(2,))]
print(vs)
pe.print_stl_numpy( vs )