from Vector import Vector
from Model import Model
import numpy as np

m = Model()

for i in xrange(5):
    m.T += 1
    print m.T

v = m.CS
for i in xrange(5):
    v.push(i)
    # print m.CS

print m.CS

# see http://stackoverflow.com/a/16887455