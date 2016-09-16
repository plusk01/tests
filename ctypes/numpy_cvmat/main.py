import numpy as np
import ctypes

from numpy.ctypeslib import ndpointer

lib = ctypes.cdll.LoadLibrary('./lib.so')

lib.mult_by_three.restype = None
lib.mult_by_three.argtypes = [ctypes.c_int, ctypes.c_int, ndpointer(ctypes.c_double)]

lib.vectorize.restype = None
lib.vectorize.argtypes = [ctypes.c_int, ctypes.c_int, ndpointer(ctypes.c_double)]

lib.create_vector_point2f.restype = None
lib.create_vector_point2f.argtypes = [ctypes.c_int, ctypes.c_int, ndpointer(ctypes.c_double)]

lib.fill_with_randn.restype = None
lib.fill_with_randn.argtypes = [ctypes.c_int, ctypes.c_int, ndpointer(ctypes.c_double)]

# http://stackoverflow.com/questions/19031836/get-background-model-from-backgroundsubtractormog2-in-python

def mult_by_three(A):
    rows, cols = A.shape
    response = A.copy()
    lib.mult_by_three(rows, cols, response)

    return response


if __name__ == '__main__':


    A = np.random.randn(5,2)
    print A

    rows, cols = A.shape
    lib.vectorize(rows, cols, A)

    print
    print
    print
    print

    lib.create_vector_point2f(rows, cols, A)

    print
    print
    print
    print

    size = (5,6)
    A = np.random.random(size=size)

    print A
    print
    print
    print mult_by_three(A)
    print
    print A

    print
    print
    print

    print "Randu"

    size = (5, 5)
    B = np.zeros(size)

    # After this, B should be full of random data
    rows, cols = B.shape
    lib.fill_with_randn(rows, cols, B)

    print B