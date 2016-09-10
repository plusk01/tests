import ctypes

import numpy as np
from numpy.ctypeslib import ndpointer

# Load the shared library
lib = ctypes.cdll.LoadLibrary('./mult_by_two.so')

# Get the actual reference to the function in the C library
mult_by_two = lib.mult_by_two

# the C function is a void
mult_by_two.restype = None

# Define the arguments of the C function
mult_by_two.argtypes =  [
                            # const double *indatav
                            ndpointer(ctypes.c_double, flags="C_CONTIGUOUS"),

                            # size_t size
                            ctypes.c_size_t,

                            # double *outdatav
                            ndpointer(ctypes.c_double, flags="C_CONTIGUOUS")
                        ]

def mult_by_two_wrapper(indata, outdata):
    assert indata.size == outdata.size
    mult_by_two(indata, indata.size, outdata)


if __name__ == '__main__':
    size = (5,6)
    indata = np.random.random(size=size)
    outdata = np.zeros(size)

    print(indata)
    print

    mult_by_two_wrapper(indata, outdata)

    print(outdata)
