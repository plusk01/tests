from ctypes import *

from Vector import Vector

class Model(object):
    lib = cdll.LoadLibrary('./model_python_lib.so') # class level loading lib
    lib.new_model.restype = c_void_p
    lib.new_model.argtypes = []
    lib.delete_model.restype = None
    lib.delete_model.argtypes = [c_void_p]

    lib.model_get_height.restype = c_float
    lib.model_get_height.argtypes = [c_void_p]
    lib.model_set_height.restype = None
    lib.model_set_height.argtypes = [c_void_p, c_float]

    lib.model_get_width.restype = c_float
    lib.model_get_width.argtypes = [c_void_p]
    lib.model_set_width.restype = None
    lib.model_set_width.argtypes = [c_void_p, c_float]

    lib.model_get_rho.restype = c_double
    lib.model_get_rho.argtypes = [c_void_p]
    lib.model_set_rho.restype = None
    lib.model_set_rho.argtypes = [c_void_p, c_double]

    lib.model_get_T.restype = c_int
    lib.model_get_T.argtypes = [c_void_p]
    lib.model_set_T.restype = None
    lib.model_set_T.argtypes = [c_void_p, c_int]

    lib.model_get_CS.restype = c_void_p
    lib.model_get_CS.argtypes = [c_void_p]

    # lib.mult_by_three.restype = c_void_p
    # lib.mult_by_three.argtypes = [c_void_p]

    def __init__(self, model_ptr=None):
        if model_ptr:
            self.model = model_ptr
        else:
            self.model = Model.lib.new_model()  # pointer to new model

        # self.CS_vector = Vector(Model.lib.model_get_CS(self.model))

    def __del__(self):  # when reference count hits 0 in Python,
        if self.model:
            Model.lib.delete_model(self.model)  # call C++ vector destructor
            self.model = None

    # def __len__(self):
    #     return Model.lib.vector_size(self.vector)

    # def __getitem__(self, i):  # access elements in vector at index
    #     if 0 <= i < len(self):
    #         return Model.lib.vector_get(self.vector, c_int(i))
    #     raise IndexError('Vector index out of range')

    # def __repr__(self):
    #     return '[{}]'.format(', '.join(str(self[i]) for i in range(len(self))))

    # def push(self, i):  # push calls vector's push_back
    #     Vector.lib.vector_push_back(self.vector, c_int(i))

    # def mult_by_three(self):  # mult_by_three in Python calls mult_by_three in C++
    #     return Vector(Vector.lib.mult_by_three(self.vector))

    @property
    def height(self):
        return Model.lib.model_get_height(self.model)

    @height.setter
    def height(self, value):
        Model.lib.model_set_height(self.model, value)

    @property
    def width(self):
        return Model.lib.model_get_width(self.model)

    @width.setter
    def width(self, value):
        Model.lib.model_set_width(self.model, value)

    @property
    def rho(self):
        return Model.lib.model_get_rho(self.model)

    @rho.setter
    def rho(self, value):
        Model.lib.model_set_rho(self.model, value)

    @property
    def T(self):
        return Model.lib.model_get_T(self.model)

    @T.setter
    def T(self, value):
        Model.lib.model_set_T(self.model, value)

    @property
    def CS(self):
        return Vector(Model.lib.model_get_CS(self.model)) #self.CS_vector