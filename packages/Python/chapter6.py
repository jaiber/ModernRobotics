#!/usr/bin/python3

import math
from modern_robotics import *


def cross_mul(w, q):
    w_so3mat = VecToso3(w)
    return np.dot(-w_so3mat, q)


print("Question 2")
res = cross_mul(np.array([0, 0, 1]), np.array([1, 0, 0]))
print("result 1 : ", res)

res = cross_mul(np.array([0, 0, 1]), np.array([2, 0, 0]))
print("result 2 : ", res)

res = cross_mul(np.array([0, 0, 1]), np.array([3, 0, 0]))
print("result 3 : ", res)

# res = cross_mul(np.array([0,0,1]), np.array([3.732,0,2.732]))
# print ("result 5 : ", res)

np.set_printoptions(formatter={"float": lambda x: "{0:0.4f}".format(x)})

# np.set_printoptions(precision=3)

print("Q2")
M = np.array([[1, 0, 0, 3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

Slist = np.array(
    [
        [0, 0, 0],
        [0, 0, 0],
        [1, 1, 1],
        [0, 0, 0],
        [0, -1, -2],
        [0, 0, 0],
    ]
)

thetalist0 = np.array([math.pi / 4, math.pi / 4, math.pi / 4])

Tsd = np.array([[-0.585, -0.811, 0, 0.076], [0.811, -0.585, 0, 2.608], [0, 0, 1, 0], [0, 0, 0, 1]])

eomg = 0.001
ev = 0.0001

[thetalist, success] = IKinSpace(Slist, M, Tsd, thetalist0, eomg, ev)

print("thetalist: ", thetalist)
print("success: ", success)
