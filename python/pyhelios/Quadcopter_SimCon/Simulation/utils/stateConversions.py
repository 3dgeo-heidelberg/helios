# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import sin, cos, tan

def phiThetaPsiDotToPQR(phi, theta, psi, phidot, thetadot, psidot):
    
    p = -sin(theta)*psidot + phidot
    
    q = sin(phi)*cos(theta)*psidot + cos(phi)*thetadot
    
    r = -sin(phi)*thetadot + cos(phi)*cos(theta)*psidot
    
    return np.array([p, q, r])


def xyzDotToUVW_euler(phi, theta, psi, xdot, ydot, zdot):
    u = xdot*cos(psi)*cos(theta) + ydot*sin(psi)*cos(theta) - zdot*sin(theta)
    
    v = (sin(phi)*sin(psi)*sin(theta) + cos(phi)*cos(psi))*ydot + (sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi))*xdot + zdot*sin(phi)*cos(theta)
    
    w = (sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))*xdot + (-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*ydot + zdot*cos(phi)*cos(theta)
    
    return np.array([u, v, w])


def xyzDotToUVW_Flat_euler(phi, theta, psi, xdot, ydot, zdot):
    uFlat = xdot * cos(psi) + ydot * sin(psi)

    vFlat = -xdot * sin(psi) + ydot * cos(psi)

    wFlat = zdot

    return np.array([uFlat, vFlat, wFlat])

def xyzDotToUVW_Flat_quat(q, xdot, ydot, zdot):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    uFlat = 2*(q0*q3 - q1*q2)*ydot + (q0**2 - q1**2 + q2**2 - q3**2)*xdot

    vFlat = -2*(q0*q3 + q1*q2)*xdot + (q0**2 + q1**2 - q2**2 - q3**2)*ydot

    wFlat = zdot

    return np.array([uFlat, vFlat, wFlat])
