#!/usr/bin/env python3

import sys
import math
import csv
from modern_robotics import *


def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot
        amd prints intermediate results, required for the assignment

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20

    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev

    joint_angles = []

    while err and i < maxiterations:
        thetalist = thetalist + np.dot(np.linalg.pinv(JacobianBody(Blist, thetalist)), Vb)
        i = i + 1
        print("Iteration " + str(i) + ":")
        print("\tjoint vector : ", end="")

        np.savetxt(sys.stdout, thetalist, newline="", fmt="%0.4f ")
        print()
        joint_angles.append(thetalist)

        se3Vec = MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, thetalist)), T))
        print("\tSE(3) end−effector config : ", end="")
        np.savetxt(sys.stdout, se3Vec, newline="", fmt="%0.3f ")
        print()

        Vb = se3ToVec(se3Vec)
        print("\terror twist V_b: ", end="")
        np.savetxt(sys.stdout, Vb, newline="", fmt="%0.3f ")
        print()

        # Calculate angular error magnitued
        eomg_err = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        print("\tangular error magnitude ∣∣omega_b∣∣ : %0.3f" % eomg_err)

        ev_err = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        print("\tlinear error magnitude ∣∣v_b∣∣ : %0.3f" % ev_err)

        err = eomg_err > eomg or ev_err > ev
        print("------------------------------------------")

    return (thetalist, joint_angles, not err)


# Main code

# UR5 constants
L1 = 0.425
L2 = 0.392
H1 = 0.089
H2 = 0.095
W1 = 0.109
W2 = 0.082

# The Body screw axes list
Blist = np.array(
    [
        [0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, -1, 0],
        [0, 1, 1, 1, 0, 1],
        [W1 + W2, H2, H2, H2, -W2, 0],
        [0, -L1 - L2, -L2, 0, 0, 0],
        [L1 + L2, 0, 0, 0, 0, 0],
    ]
)

# The desired end-effector configuration
Tsd = np.array([[0, 1, 0, -0.5], [0, 0, -1, 0.1], [-1, 0, 0, 0.1], [0, 0, 0, 1]])

# Error limits - given in assignment
eomg = 0.001
ev = 0.0001

# The {0} configuration as given in chapter 4 (page 145)
M = np.array([[-1, 0, 0, L1 + L2], [0, 0, 1, W1 + W2], [0, 1, 0, H1 - H2], [0, 0, 0, 1]])

# Initial theta_0 assumed values
thetalist0 = np.array([2, -1, 1, -0.5, -0.5, 0])

# Set numpy print only 3 decimals
np.set_printoptions(formatter={"float": lambda x: "{0:0.4f}".format(x)})

[thetalist, joint_angles, success] = IKinBodyIterates(Blist, M, Tsd, thetalist0, eomg, ev)

csv_file = "iterates.csv"

if success is True:
    print("SUCCESS")
    # print("thetalist: ", thetalist)
    # print("joint_angles: ", joint_angles)

    with open(csv_file, "w", newline="") as csvw:
        csvwriter = csv.writer(csvw)
        csvwriter.writerows(joint_angles)
        print("Wrote", csv_file)
else:
    print("No IK solutions found!")
