#!/usr/bin/python3

import csv
from modern_robotics import *

# Project
print("Project C3 W2")

g = np.array([0, 0, -9.81])

# Copied from the UR5_parameter file
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]
Slist = [
    [0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, -1, 0],
    [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
    [0, 0, 0, 0, 0.81725, 0],
    [0, 0, 0.425, 0.81725, 0, 0.81725],
]

# Initialize the values for {0} position, torques and velocities
thetalist = np.array([0, 0, 0, 0, 0, 0])
dthetalist = np.array([0, 0, 0, 0, 0, 0])
Ftip = np.array([0, 0, 0, 0, 0, 0])
taulist = np.array([0, 0, 0, 0, 0, 0])

thetalistArr = []
dt = 0.01

# First simulation
print("Running 1st simulation")
for i in range(0, 300):
    # Compute the acceleration
    ddthetalist = ForwardDynamics(
        thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist
    )
    thetalistArr.append(thetalist)
    # Compute angles and velocities for 1 timestep
    [thetalistNext, dthetalistNext] = EulerStep(thetalist, dthetalist, ddthetalist, dt)

    # Update the new angles and velocities
    thetalist = thetalistNext
    dthetalist = dthetalistNext

# Write out the simulation1 values
with open("simulation1.csv", "w", newline="") as csvw:
    csvwriter = csv.writer(csvw)
    csvwriter.writerows(thetalistArr)
    print("Wrote simulation1.csv")

# Second simulation
thetalist = np.array([0, -1, 0, 0, 0, 0])
dthetalist = np.array([0, 0, 0, 0, 0, 0])

thetalistArr.clear()
print("Running 2nd simulation")
for i in range(0, 500):
    # Compute the acceleration
    ddthetalist = ForwardDynamics(
        thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist
    )
    thetalistArr.append(thetalist)
    # Compute angles and velocities for 1 timestep
    [thetalistNext, dthetalistNext] = EulerStep(thetalist, dthetalist, ddthetalist, dt)

    # Update the new angles and velocities
    thetalist = thetalistNext
    dthetalist = dthetalistNext

# Write out the simulation1 values
with open("simulation2.csv", "w", newline="") as csvw:
    csvwriter = csv.writer(csvw)
    csvwriter.writerows(thetalistArr)
    print("Wrote simulation2.csv")