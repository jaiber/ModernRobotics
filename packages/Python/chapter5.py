#!/usr/bin/python3

import math
import sys
from modern_robotics import *


def cross_mul(w, q):
    w_so3mat = VecToso3(w)
    return np.dot(-w_so3mat, q)


print("Question 2")
res = cross_mul(np.array([0, 0, 1]), np.array([1, 0, 0]))
print("result 1 : ", res)

res = cross_mul(np.array([0, 0, 1]), np.array([2, 0, 0]))
print("result 2 : ", res)

np.set_printoptions(formatter={"float": lambda x: "{0:0.4f}".format(x)})

# np.set_printoptions(precision=3)

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
Jb = np.array(
    [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [1, 1, 1, 1],
        [-1, -1, -1, 0],
        [3, 2, 1, 1],
        [0, 0, 0, 1],
    ]
)
thetalist = np.array([0, math.pi / 4, 0])

Js = JacobianSpace(Slist, thetalist)
print("Js shape: ", np.shape(Js))
print("Js: ")
print(Js)

F = np.array([0, 0, 0, 2, 0, 0])
print("F shape: ", np.shape(F))

RES = np.dot(Js.T, F)

print("RES: ")
print(RES)

print("Q2")
F = np.array([0, 0, 10, 10, 10, 0])
RES = np.dot(Jb.T, F)

print("RES: ")
print(RES)
# sys.exit(0)

print("Q3:")
Slist = np.array([[0, 1, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 2, 1], [0, 0, 0]])
thetalist = np.array([math.pi / 2, math.pi / 2, 1])

res = JacobianSpace(Slist, thetalist)
print("res:")
print(res)

print("Q4")
Blist = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0], [3, 0, 0], [0, 3, 0], [0, 0, 1]])

res = JacobianBody(Blist, thetalist)
print("res:")
print(res)

print("Q5")
Jv = np.array(
    [
        [-0.105, 0, 0.006, -0.045, 0, 0.006, 0],
        [-0.889, 0.006, 0, -0.844, 0.006, 0, 0],
        [0, -0.105, 0.889, 0, 0, 0, 0],
    ]
)

res = np.dot(Jv, Jv.T)
print("res")
print(res)
eig = np.linalg.eig(res)
print("eig")
print(eig)

sqrt_eig = np.sqrt(eig.eigenvalues)
print("sqrt_eig")
print(sqrt_eig)
print("Q6 ans: ", np.max(sqrt_eig))

ev = eig.eigenvectors
max1 = np.max(ev, axis=0)
print("max1: ", max1)
# print("max1: ", np.max(ev, axis=0))
# print("max1: ", np.max(ev, axis=1))
print("norm: ", np.linalg.norm(max1))


# print("norm")
# print(np.linalg.norm(sqrt_eig))
norm = max1 / np.linalg.norm(max1)
# print("sqrt_eig")
print(norm)
print("its norm: ", np.linalg.norm(norm))
