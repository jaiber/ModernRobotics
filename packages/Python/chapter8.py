#!/usr/bin/python3

import sys
import math
from modern_robotics import *

thetalist = np.array([
    0, math.pi/6, math.pi/4, math.pi/3, math.pi/2, 2*math.pi/3
])

dthetalist = np.array([
    0.2, 0.2, 0.2, 0.2, 0.2, 0.2
])

ddthetalist = np.array([
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1
])

g = np.array([
    0, 0, -9.81
])

Ftip = np.array([
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1
])

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

taulist = InverseDynamics(thetalist=thetalist, dthetalist=dthetalist,
                          ddthetalist=ddthetalist, g=g, Ftip=Ftip,
                          Mlist=Mlist, Glist=Glist, Slist=Slist)

np.set_printoptions(formatter={"float": lambda x: "{0:0.4f}".format(x)})
print("taulist: ", taulist)

M = MassMatrix(thetalist,Mlist,Glist,Slist)
print("MassMatrix M: ", repr(M))

c = VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist)
print("VelQuadraticForces c: ", repr(c))

grav = GravityForces(thetalist,g,Mlist,Glist,Slist)
print("GravityForces grav: ", repr(grav))

JTFtip = EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist)
print("EndEffectorForces JFtip: ", repr(JTFtip))

ddthetalist = ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)
print("ForwardDynamics ddthetalist: ", repr(ddthetalist))

sys.exit()
# Project
print("Project C3 W2")
thetalist = np.array([0, 0, 0, 0, 0, 0])
dthetalist = np.array([0, 0, 0, 0, 0, 0])
Ftip = np.array([0, 0, 0, 0, 0, 0])
#ddthetalist = np.array([0, 0, 0, 0, 0, 0])

#taulist = InverseDynamics(thetalist=thetalist, dthetalist=dthetalist,
#                          ddthetalist=ddthetalist, g=g, Ftip=Ftip,
#                          Mlist=Mlist, Glist=Glist, Slist=Slist)
taulist = np.array([0, 0, 0, 0, 0, 0])
print("taulist: ", taulist)

ddthetalist = ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)
print("ForwardDynamics ddthetalist: ", repr(ddthetalist))

dt = 0.01
for i in range(0, 300):
    [thetalistNext,dthetalistNext] = EulerStep(thetalist,dthetalist,ddthetalist,dt)
    #print("thetalistNext: ", thetalistNext)
    np.savetxt(sys.stdout, thetalist, newline="", delimiter=", ", fmt="%0.4f ")
    print()

    #print("dthetalistNext: ", dthetalistNext)
    thetalist = thetalistNext
    dthetalist = dthetalistNext
    ddthetalist = ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)
    #print("ForwardDynamics ddthetalist: ", repr(ddthetalist))


#Ftipmat = np.zeros([6, 6])
#taumat = InverseDynamicsTrajectory(thetalist,dthetalist,ddthetalist, g,Ftipmat,Mlist,Glist,Slist)
#print("Taumat: ", taumat)


#[thetamat,dthetamat] = ForwardDynamicsTrajectory(thetalist,dthetalist,taumat, g,Ftipmat,Mlist,Glist,Slist,dt,intRes)