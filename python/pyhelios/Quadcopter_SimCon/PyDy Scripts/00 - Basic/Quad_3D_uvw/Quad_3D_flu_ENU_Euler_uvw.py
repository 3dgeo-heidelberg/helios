# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

"""
Using PyDy and Sympy, this script generates the equations for the state derivatives 
of a quadcopter in a 3-dimensional space. The states in this particular script are:

x, y, z           : Position of the drone's center of mass in the inertial frame, 
                    expressed in the inertial frame
u, v, w           : Velocity of the drone's center of mass in the inertial frame, 
                    expressed in the drone's frame
phi, theta, psi   : Orientation (roll, pitch, yaw angles) of the drone in the 
                    inertial frame, following the order ZYX (yaw, pitch, roll)
p, q, r           : Angular velocity of the drone in the inertial frame,
                    expressed in the drone's frame

Important note    : This script uses a flu body orientation (front-left-up) and 
                    a ENU world orientation (East-North-Up)

Other note        : In the resulting state derivatives, there are still simplifications
                    that can be made that SymPy cannot simplify (factoring).

author: John Bass
email: 
"""

from sympy import symbols
from sympy.physics.mechanics import *

# Reference frames and Points
# ---------------------------
N = ReferenceFrame('N')  # Inertial Frame
B = ReferenceFrame('B')  # Drone after X (roll) rotation (Final rotation)
C = ReferenceFrame('C')  # Drone after Y (pitch) rotation
D = ReferenceFrame('D')  # Drone after Z (yaw) rotation (First rotation)

No = Point('No')
Bcm = Point('Bcm')  # Drone's center of mass
M1 = Point('M1')    # Motor 1 is front left, then the rest increments CW (Drone is in X configuration, not +)
M2 = Point('M2')
M3 = Point('M3')
M4 = Point('M4')

# Variables
# ---------------------------
# x, y and z are the drone's coordinates in the inertial frame, expressed with the inertial frame
# u, v and w are the drone's velocities in the inertial frame, expressed with the drone's frame
# phi, theta and psi represents the drone's orientation in the inertial frame, expressed with a ZYX Body rotation
# p, q and r are the drone's angular velocities in the inertial frame, expressed with the drone's frame

x, y, z, u, v, w = dynamicsymbols('x y z u v w')
phi, theta, psi, p, q, r = dynamicsymbols('phi theta psi p q r')

# First derivatives of the variables
xd, yd, zd, ud, vd, wd = dynamicsymbols('x y z u v w', 1)
phid, thetad, psid, pd, qd, rd = dynamicsymbols('phi theta psi p q r', 1)

# Constants
# ---------------------------
mB, g, dxm, dym, dzm, IBxx, IByy, IBzz = symbols('mB g dxm dym dzm IBxx IByy IBzz')
ThrM1, ThrM2, ThrM3, ThrM4, TorM1, TorM2, TorM3, TorM4 = symbols('ThrM1 ThrM2 ThrM3 ThrM4 TorM1 TorM2 TorM3 TorM4')

# Rotation ZYX Body
# ---------------------------
D.orient(N, 'Axis', [psi, N.z])
C.orient(D, 'Axis', [theta, D.y])
B.orient(C, 'Axis', [phi, C.x])

# Origin
# ---------------------------
No.set_vel(N, 0)

# Translation
# ---------------------------
Bcm.set_pos(No, x*N.x + y*N.y + z*N.z)
Bcm.set_vel(N, u*B.x + v*B.y + w*B.z) 

# Motor placement
# M1 is front left, then clockwise numbering
# dzm is positive for motors above center of mass
# ---------------------------
M1.set_pos(Bcm,  dxm*B.x + dym*B.y + dzm*B.z)
M2.set_pos(Bcm,  dxm*B.x - dym*B.y + dzm*B.z)
M3.set_pos(Bcm, -dxm*B.x - dym*B.y + dzm*B.z)
M4.set_pos(Bcm, -dxm*B.x + dym*B.y + dzm*B.z)
M1.v2pt_theory(Bcm, N, B)
M2.v2pt_theory(Bcm, N, B)
M3.v2pt_theory(Bcm, N, B)
M4.v2pt_theory(Bcm, N, B)

# Inertia Dyadic
# ---------------------------
IB = inertia(B, IBxx, IByy, IBzz)

# Create Bodies
# ---------------------------
BodyB = RigidBody('BodyB', Bcm, B, mB, (IB, Bcm))
BodyList = [BodyB]

# Forces and Torques
# ---------------------------
Grav_Force = (Bcm, -mB*g*N.z)
FM1 = (M1, ThrM1*B.z)
FM2 = (M2, ThrM2*B.z)
FM3 = (M3, ThrM3*B.z)
FM4 = (M4, ThrM4*B.z)

TM1 = (B,  TorM1*B.z)
TM2 = (B, -TorM2*B.z)
TM3 = (B,  TorM3*B.z)
TM4 = (B, -TorM4*B.z)
ForceList = [Grav_Force, FM1, FM2, FM3, FM4, TM1, TM2, TM3, TM4]

# Kinematic Differential Equations
# ---------------------------
kd = [xd - dot(Bcm.vel(N), N.x), yd - dot(Bcm.vel(N), N.y), zd - dot(Bcm.vel(N), N.z), p - dot(B.ang_vel_in(N), B.x), q - dot(B.ang_vel_in(N), B.y), r - dot(B.ang_vel_in(N), B.z)]

# Kane's Method
# ---------------------------
KM = KanesMethod(N, q_ind=[x, y, z, phi, theta, psi], u_ind=[u, v, w, p, q, r], kd_eqs=kd)
(fr, frstar) = KM.kanes_equations(BodyList, ForceList)

# Equations of Motion
# ---------------------------
MM = KM.mass_matrix_full
kdd = KM.kindiffdict()
rhs = KM.forcing_full
rhs = rhs.subs(kdd)

MM.simplify()
print('Mass Matrix')
print('-----------')
mprint(MM)
print()

rhs.simplify()
print('Right Hand Side')
print('---------------')
mprint(rhs)
print()

# So, MM*x = rhs, where x is the State Derivatives
# Solve for x
stateDot = MM.inv()*rhs
print('State Derivatives')
print('-----------------------------------')
mprint(stateDot)
print()

# POST-PROCESSING
# ---------------------------

# Useful Numbers
# ---------------------------
# p, q, r are the drone's angular velocities in the inertial frame, expressed in the drone's frame.
# These calculations are not relevant to the ODE, but might be used for control.
print('P, Q, R (Angular velocities in drone frame)')
print('-------------------------------------------')
mprint(dot(B.ang_vel_in(N), B.x))
print()
mprint(dot(B.ang_vel_in(N), B.y))
print()
mprint(dot(B.ang_vel_in(N), B.z))
print()

# xdot, ydot, zdot are the drone's velocities in the inertial frame, expressed in the inertial frame.
# These calculations are not relevant to the ODE, but might be used for control
xdot = dot(Bcm.vel(N).subs(kdd), N.x)
ydot = dot(Bcm.vel(N).subs(kdd), N.y)
zdot = dot(Bcm.vel(N).subs(kdd), N.z)
print('xdot, ydot, zdot (Velocities in inertial frame)')
print('-----------------------------------')
mprint(xdot)
print()
mprint(ydot)
print()
mprint(zdot)
print()

# uFlat, vFlat, wFlat are the drone's velocities in the inertial frame, expressed in a frame
# parallel to the ground, but with the drone's heading (so only after the YAW rotation). 
# These calculations are not relevant to the ODE, but might be used for control.
uFlat = dot(Bcm.vel(N).subs(kdd), D.x)
vFlat = dot(Bcm.vel(N).subs(kdd), D.y)
wFlat = dot(Bcm.vel(N).subs(kdd), D.z)
print('uFlat, vFlat, wFlat (Velocities in drone frame (after Yaw))')
print('-----------------------------------------------------------')
mprint(uFlat)
print()
mprint(vFlat)
print()
mprint(wFlat)
print()
