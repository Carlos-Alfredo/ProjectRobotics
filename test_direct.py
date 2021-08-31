from direct_kin import directf
import numpy as np

Q = [-89.99999941612928, 23.318919284425856, -90]
theta = [0.1552, 0.1368, 0.1917]
P = directf(Q,theta)
print(P)