#!/usr/bin/python3

import math
from modern_robotics import *


def cross_mul(w, q):
    w_so3mat = VecToso3(w)
    return np.dot(-w_so3mat, q)


print("Question 2")
res = cross_mul(np.array([0, 0, 1]), np.array([1, 0, 0]))
print("result 1 : ", res)

res = cross_mul(np.array([0, 1, 0]), np.array([1, 0, 0]))
print("result 2 : ", res)

res = cross_mul(np.array([0, 1, 0]), np.array([2.732, 0, -1]))
print("result 3 : ", res)

res = cross_mul(np.array([0, 1, 0]), np.array([3.732, 0, 0.732]))
print("result 4 : ", res)

# res = cross_mul(np.array([0,0,1]), np.array([3.732,0,2.732]))
# print ("result 5 : ", res)

res = cross_mul(np.array([0, 0, 1]), np.array([3.732, 0, 2.732]))
print("result 6 : ", res)

print("Question 3")
res = cross_mul(np.array([0, 0, 1]), np.array([-2.732, 0, -2.732]))
print("result 1 : ", res)

res = cross_mul(np.array([0, 1, 0]), np.array([-2.732, 0, -2.732]))
print("result 2 : ", res)

res = cross_mul(np.array([0, 1, 0]), np.array([-1, 0, -3.732]))
print("result 3 : ", res)

res = cross_mul(np.array([0, 1, 0]), np.array([0, 0, -2]))
print("result 4 : ", res)

res = cross_mul(np.array([0, 0, 0]), np.array([0, 0, -1]))
print("result 5 : ", res)

res = cross_mul(np.array([0, 0, 1]), np.array([0, 0, 0]))
print("result 6 : ", res)

np.set_printoptions(formatter={"float": lambda x: "{0:0.4f}".format(x)})

# np.set_printoptions(precision=3)

sq_rt3 = math.sqrt(3)

print("Q4")
M = np.array([[1, 0, 0, 2 + sq_rt3], [0, 1, 0, 0], [0, 0, 1, 1 + sq_rt3], [0, 0, 0, 1]])

Slist = np.array(
    [
        [0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0],
        [1, 0, 0, 0, 0, 1],
        [0, 0, 1, -(sq_rt3 - 1), 0, 0],
        [-1, 0, 0, 0, 0, -(2 + sq_rt3)],
        [0, 1, 1 + sq_rt3, 2 + sq_rt3, 1, 0],
    ]
)
Blist = np.array(
    [
        [0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0],
        [1, 0, 0, 0, 0, 1],
        [0, 1 + sq_rt3, 2 + sq_rt3, 2, 0, 0],
        [1 + sq_rt3, 0, 0, 0, 0, 0],
        [0, -(1 + sq_rt3), -1, 0, 1, 0],
    ]
)
thetalist = np.array([-math.pi / 2, math.pi / 2, math.pi / 3, -math.pi / 4, 1, math.pi / 6])

T = FKinSpace(M, Slist, thetalist)
print("T array")
print(T)

print("Q5")
T = FKinBody(M, Blist, thetalist)
print("T array")
print(T)
