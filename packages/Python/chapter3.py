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


T_SA = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 1], [0, 0, 0, 1]])
T_SB = np.array([[1, 0, 0, 0], [0, 0, 1, 2], [0, -1, 0, 0], [0, 0, 0, 1]])

T_AS = TransInv(T_SA)

T_AB = np.dot(T_AS, T_SB)
print("TAB: ", T_AB)

p_b = np.array([1, 2, 3])

# Q5
p_s = np.dot(T_SB, np.append(p_b, 1))
print("p_s: ", p_s)

adj_T_AS = Adjoint(T_AS)
print("adj_T_AS: ", adj_T_AS)
V_s = np.array([3, 2, 1, -1, -2, -3])

# Q6
V_a = np.dot(adj_T_AS, V_s)
print("V_a: ", V_a)

# Q8
# Calculate matrix logarithm of T_SA
logT_SA = MatrixLog6(T_SA)
print("logT_SA: ", logT_SA)
# Get the rotation angle and axis from the matrix logarithm
theta = so3ToVec(logT_SA[0:3, 0:3])
# print theta in radians
print("theta in radians: ", np.linalg.norm(theta))

# Q9

S_theta = np.array([0, 1, 2, 3, 0, 0])
# Calculate the matrix exponential of the twist
mexp = MatrixExp6(VecTose3(S_theta))
print("mexp: ", mexp)

# Q10
F_b = np.array([1, 0, 0, 2, 1, 0])
adj_T_SB = Adjoint(T_SB)
# Calculate the representation of wrench in Frame B
F_a = np.dot(adj_T_SB, F_b)
print("F_a: ", F_a)

# Q11
T = np.array([[0, -1, 0, 3], [1, 0, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])
# Print transInv
print("TransInv: ", TransInv(T))

# Q14
S_theta = np.array(
    [[0, -1.5708, 0, 2.3562], [1.5708, 0, 0, -2.3562], [0, 0, 0, 1], [0, 0, 0, 0]]
)
# Print MatrixExp6
print("MatExp6", MatrixExp6(S_theta))

# Q15
T = np.array([[0, -1, 0, 3], [1, 0, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])
# Print MatrixLog6
print("MatrixLog6: ", MatrixLog6(T))
