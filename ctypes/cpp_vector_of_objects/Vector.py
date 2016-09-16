from ctypes import *

class Vector(object):
    lib = cdll.LoadLibrary('./lib.so') # class level loading lib
    lib.new_vector.restype = c_void_p
    lib.new_vector.argtypes = []
    lib.delete_vector.restype = None
    lib.delete_vector.argtypes = [c_void_p]
    lib.vector_size.restype = c_int
    lib.vector_size.argtypes = [c_void_p]
    lib.vector_get.restype = c_void_p
    lib.vector_get.argtypes = [c_void_p, c_int]
    lib.vector_push_back.restype = None
    lib.vector_push_back.argtypes = [c_void_p, c_void_p]

    def __init__(self, vector_ptr=None):
        if vector_ptr:
            self.vector = vector_ptr
        else:
            self.vector = Vector.lib.new_vector()  # pointer to new vector

    def free(self):  # when reference count hits 0 in Python,
        if self.vector:
            # Vector.lib.delete_vector(self.vector)  # call C++ vector destructor
            self.vector = None

    def __len__(self):
        return Vector.lib.vector_size(self.vector)

    def __getitem__(self, i):  # access elements in vector at index
        if 0 <= i < len(self):
            m_ptr = Vector.lib.vector_get(self.vector, c_int(i))
            from Model import Model
            return Model(m_ptr)
        raise IndexError('Vector index out of range')

    def __repr__(self):
        return '[{}]'.format(', '.join(str(self[i]) for i in range(len(self))))

    def push(self, m):  # push calls vector's push_back

        m_ptr = m.model

        Vector.lib.vector_push_back(self.vector, c_void_p(m_ptr))

class IntVector(object):
    lib = cdll.LoadLibrary('./lib.so') # class level loading lib
    lib.new_int_vector.restype = c_void_p
    lib.new_int_vector.argtypes = []
    lib.delete_int_vector.restype = None
    lib.delete_int_vector.argtypes = [c_void_p]
    lib.int_vector_size.restype = c_int
    lib.int_vector_size.argtypes = [c_void_p]
    lib.int_vector_get.restype = c_int
    lib.int_vector_get.argtypes = [c_void_p, c_int]
    lib.int_vector_push_back.restype = None
    lib.int_vector_push_back.argtypes = [c_void_p, c_int]

    def __init__(self, vector_ptr=None):
        if vector_ptr:
            self.vector = vector_ptr
        else:
            self.vector = Vector.lib.new_int_vector()  # pointer to new vector

    def __del__(self):  # when reference count hits 0 in Python,
        if self.vector:
            # Vector.lib.delete_vector(self.vector)  # call C++ vector destructor
            self.vector = None

    def __len__(self):
        return Vector.lib.int_vector_size(self.vector)

    def __getitem__(self, i):  # access elements in vector at index
        if 0 <= i < len(self):
            return Vector.lib.int_vector_get(self.vector, c_int(i))
        raise IndexError('Vector index out of range')

    def __repr__(self):
        return '[{}]'.format(', '.join(str(self[i]) for i in range(len(self))))

    def push(self, i):  # push calls vector's push_back
        IntVector.lib.int_vector_push_back(self.vector, c_int(i))