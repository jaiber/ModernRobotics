#!/usr/bin/python3

from modern_robotics import *

vec = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])

print("vec: ", vec)

# omgtheta = so3ToVec(vec)
# print("omgtheta: ", omgtheta)

# Calculate the matrix logarithm of a rotation matrix
logR = MatrixLog3(vec)
print("logR: ", logR)

# Get the rotation angle and axis from the matrix logarithm
theta = so3ToVec(logR)
print("theta: ", theta)

# Print theta in radians
print("theta in radians: ", np.linalg.norm(theta))

exp_coord_rot = np.array([1, 2, 0])
print("exp_coord_rot: ", exp_coord_rot)

vec = VecToso3(exp_coord_rot)

# Q9
mexp = MatrixExp3(vec)
print("mexp: ", mexp)