import numpy as np
import ctypes

from numpy.ctypeslib import ndpointer

lib = ctypes.cdll.LoadLibrary('./cvmat_python_lib.so')

lib.mult_by_three.restype = None
lib.mult_by_three.argtypes = [ctypes.c_int, ctypes.c_int, ndpointer(ctypes.c_double)]

# http://stackoverflow.com/questions/19031836/get-background-model-from-backgroundsubtractormog2-in-python

def mult_by_three(A):
    rows, cols = A.shape
    response = A.copy()
    lib.mult_by_three(rows, cols, response)

    return response


if __name__ == '__main__':

    size = (5,6)
    A = np.random.random(size=size)

    print A
    print
    print
    print mult_by_three(A)
    print
    print A