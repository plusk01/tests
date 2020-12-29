#!/usr/bin/env python3

import sys
sys.path.append('../build/')

import numpy as np
import pybind_example as pe

class MyDerived(pe.smartptrs.Base):
  def __init__(self):
    # use direct, not super()
    # https://pybind11.readthedocs.io/en/stable/advanced/classes.html#overriding-virtual-functions-in-python
    pe.smartptrs.Base.__init__(self, 10)
    self.z = 2 * self.v

  def __repr__(self):
    return "<My with v = {} and z = {}>".format(self.v, self.z)

  @staticmethod
  def GetInitFunction():
    def init():
      return MyDerived()
    return init

  @staticmethod
  def GetUpdateFunction(w):
    def update(base):
      if isinstance(base, MyDerived):
        base.z += w
        print(base)
    return update


base = pe.smartptrs.Base(7)
derived = pe.smartptrs.Derived()
my = MyDerived()

example = pe.smartptrs.Example()

print("\nHere comes the test:\n")
example.print(base)
example.print(derived)
example.print(my)

print("\nProcessing derived:")
example.process(derived)
example.print(derived)

print("\nProcessing my:")
example.process(my, MyDerived.GetUpdateFunction(1))
print(my)

print("\nCreate my:")
obj = example.create(MyDerived.GetInitFunction())
print(type(obj))
print(obj) # how to get a MyDerived?
