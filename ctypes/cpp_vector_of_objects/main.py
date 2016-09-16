from Vector import Vector, IntVector
from Model import Model
import numpy as np

v = Vector()


m = Model()
for i in xrange(5):
    m.T += 1
    print m.T

v = m.CS
for i in xrange(5):
    v.push(i)
    # print m.CS

print m.CS

v.push(m)

print v

# see http://stackoverflow.com/a/16887455