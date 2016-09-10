from Vector import Vector
import numpy as np

v = Vector()

for i in xrange(30):
    v.push(np.random.randint(0, 30))

print v

print v.mult_by_three()

print v

# see http://stackoverflow.com/a/16887455