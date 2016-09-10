from ctypes import *

class Vector(object):
    lib = cdll.LoadLibrary('./vector_python_lib.so') # class level loading lib
    lib.new_vector.restype = c_void_p
    lib.new_vector.argtypes = []
    lib.delete_vector.restype = None
    lib.delete_vector.argtypes = [c_void_p]
    lib.vector_size.restype = c_int
    lib.vector_size.argtypes = [c_void_p]
    lib.vector_get.restype = c_int
    lib.vector_get.argtypes = [c_void_p, c_int]
    lib.vector_push_back.restype = None
    lib.vector_push_back.argtypes = [c_void_p, c_int]
    lib.mult_by_three.restype = c_void_p
    lib.mult_by_three.argtypes = [c_void_p]

    def __init__(self, vector_ptr=None):
        if vector_ptr:
            self.vector = vector_ptr
        else:
            self.vector = Vector.lib.new_vector()  # pointer to new vector

    def __del__(self):  # when reference count hits 0 in Python,
        Vector.lib.delete_vector(self.vector)  # call C++ vector destructor

    def __len__(self):
        return Vector.lib.vector_size(self.vector)

    def __getitem__(self, i):  # access elements in vector at index
        if 0 <= i < len(self):
            return Vector.lib.vector_get(self.vector, c_int(i))
        raise IndexError('Vector index out of range')

    def __repr__(self):
        return '[{}]'.format(', '.join(str(self[i]) for i in range(len(self))))

    def push(self, i):  # push calls vector's push_back
        Vector.lib.vector_push_back(self.vector, c_int(i))

    def mult_by_three(self):  # mult_by_three in Python calls mult_by_three in C++
        return Vector(Vector.lib.mult_by_three(self.vector))