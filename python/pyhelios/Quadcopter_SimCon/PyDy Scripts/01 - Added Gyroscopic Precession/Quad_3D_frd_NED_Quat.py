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
xdot, ydot, zdot  : Velocity of the drone's center of mass in the inertial frame, 
                    expressed in the inertial frame
q0, q1, q2, q3    : Orientation of the drone in the inertial frame using quaternions
p, q, r           : Angular velocity of the drone in the inertial frame,
                    expressed in the drone's frame

Important note    : This script uses a frd body orientation (front-right-down) and 
                    a NED world orientation (North-East-Down). The drone's altitude is -z.

Other note        : In the resulting state derivatives, there are still simplifications
                    that can be made that SymPy cannot simplify (factoring).

author: John Bass
email: 
"""

from sympy import symbols, Matrix
from sympy.physics.mechanics import *

# Reference frames and Points
# ---------------------------
N = ReferenceFrame('N')  # Inertial Frame
B = ReferenceFrame('B')  # Drone after X (roll) rotation (Final rotation)

No = Point('No')
Bcm = Point('Bcm')  # Drone's center of mass
M1 = Point('M1')    # Motor 1 is front left, then the rest increments CW (Drone is in X configuration, not +)
M2 = Point('M2')
M3 = Point('M3')
M4 = Point('M4')

# Variables
# ---------------------------
# x, y and z are the drone's coordinates in the inertial frame, expressed with the inertial frame
# xdot, ydot and zdot are the drone's speeds in the inertial frame, expressed with the inertial frame
# q0, q1, q2, q3 represents the drone's orientation in the inertial frame, using quaternions
# p, q and r are the drone's angular velocities in the inertial frame, expressed with the drone's frame

x, y, z, xdot, ydot, zdot = dynamicsymbols('x y z xdot ydot zdot')
q0, q1, q2, q3, p, q, r = dynamicsymbols('q0 q1 q2 q3 p q r')

# First derivatives of the variables
xd, yd, zd, xdotd, ydotd, zdotd = dynamicsymbols('x y z xdot ydot zdot', 1)
q0d, q1d, q2d, q3d, pd, qd, rd = dynamicsymbols('q0 q1 q2 q3 p q r', 1)

# Constants
# ---------------------------
mB, g, dxm, dym, dzm, IBxx, IByy, IBzz, IRzz, wM1, wM2, wM3, wM4 = symbols('mB g dxm dym dzm IBxx IByy IBzz IRzz wM1 wM2 wM3 wM4')
ThrM1, ThrM2, ThrM3, ThrM4, TorM1, TorM2, TorM3, TorM4 = symbols('ThrM1 ThrM2 ThrM3 ThrM4 TorM1 TorM2 TorM3 TorM4')

# Rotation Quaternion
# ---------------------------
B.orient(N, 'Quaternion', [q0, q1, q2, q3])
B.set_ang_vel(N, p*B.x + q*B.y + r*B.z)

# Origin
# ---------------------------
No.set_vel(N, 0)

# Translation
# ---------------------------
Bcm.set_pos(No, x*N.x + y*N.y + z*N.z)
Bcm.set_vel(N, Bcm.pos_from(No).dt(N)) 

# Motor placement
# M1 is front left, then clockwise numbering
# dzm is positive for motors above center of mass
# ---------------------------
M1.set_pos(Bcm,  dxm*B.x - dym*B.y - dzm*B.z)
M2.set_pos(Bcm,  dxm*B.x + dym*B.y - dzm*B.z)
M3.set_pos(Bcm, -dxm*B.x + dym*B.y - dzm*B.z)
M4.set_pos(Bcm, -dxm*B.x - dym*B.y - dzm*B.z)
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
Grav_Force = (Bcm, mB*g*N.z)
FM1 = (M1, -ThrM1*B.z)
FM2 = (M2, -ThrM2*B.z)
FM3 = (M3, -ThrM3*B.z)
FM4 = (M4, -ThrM4*B.z)

TM1 = (B, -TorM1*B.z)
TM2 = (B,  TorM2*B.z)
TM3 = (B, -TorM3*B.z)
TM4 = (B,  TorM4*B.z)

gyro = (B, -IRzz*cross(B.ang_vel_in(N), (wM1 - wM2 + wM3 - wM4)*B.z))

ForceList = [Grav_Force, FM1, FM2, FM3, FM4, TM1, TM2, TM3, TM4, gyro]

# Calculate Quaternion Derivative
# ---------------------------
Gquat = Matrix([[-q1,  q0,  q3, -q2],
                [-q2, -q3,  q0,  q1],
                [-q3,  q2, -q1,  q0]])

angVel = Matrix([[dot(B.ang_vel_in(N), B.x)],[dot(B.ang_vel_in(N), B.y)],[dot(B.ang_vel_in(N), B.z)]]) # Basically the same as: angVel = Matrix([[p],[q],[r]])
quat_dot = 1.0/2*Gquat.T*angVel

# Kinematic Differential Equations
# ---------------------------
kd = [xdot - xd, ydot - yd, zdot - zd, q0d - quat_dot[0], q1d - quat_dot[1], q2d - quat_dot[2], q3d - quat_dot[3]]

# Kane's Method
# ---------------------------
KM = KanesMethod(N, q_ind=[x, y, z, q0, q1, q2, q3], u_ind=[xdot, ydot, zdot, p, q, r], kd_eqs=kd)
(fr, frstar) = KM.kanes_equations(BodyList, ForceList)

# Equations of Motion
# ---------------------------
MM = KM.mass_matrix_full
kdd = KM.kindiffdict()
rhs = KM.forcing_full
MM = MM.subs(kdd)
rhs = rhs.subs(kdd)

MM.simplify()
# MM = MM.subs(q0**2 + q1**2 + q2**2 + q3**2, 1)
print()
print('Mass Matrix')
print('-----------')
mprint(MM)

rhs.simplify()
# rhs = rhs.subs(q0**2 + q1**2 + q2**2 + q3**2, 1)
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
quat_dot_2 = Matrix([[q0d],[q1d],[q2d],[q3d]])
angVel_2 = 2.0*Gquat*quat_dot_2
print('P, Q, R (Angular velocities in drone frame)')
print('-------------------------------------------')
mprint(angVel_2[0])
print()
mprint(angVel_2[1])
print()
mprint(angVel_2[2])
print()

# u, v, w are the drone's velocities in the inertial frame, expressed in the drone's frame.
# These calculations are not relevant to the ODE, but might be used for control.
u = dot(Bcm.vel(N).subs(kdd), B.x).simplify()
v = dot(Bcm.vel(N).subs(kdd), B.y).simplify()
w = dot(Bcm.vel(N).subs(kdd), B.z).simplify()
print('u, v, w (Velocities in drone frame)')
print('-----------------------------------')
mprint(u)
print()
mprint(v)
print()
mprint(w)
print()

# uFlat, vFlat, wFlat are the drone's velocities in the inertial frame, expressed in a frame
# parallel to the ground, but with the drone's heading.
# These calculations are not relevant to the ODE, but might be used for control.
uFlat = dot(Bcm.vel(N).subs(kdd),  cross(B.y, N.z)).simplify()
vFlat = dot(Bcm.vel(N).subs(kdd), -cross(B.x, N.z)).simplify()
wFlat = dot(Bcm.vel(N).subs(kdd), N.z).simplify()
print('uFlat, vFlat, wFlat (Velocities in drone frame (after Yaw))')
print('-----------------------------------------------------------')
mprint(uFlat)
print()
mprint(vFlat)
print()
mprint(wFlat)
print()
