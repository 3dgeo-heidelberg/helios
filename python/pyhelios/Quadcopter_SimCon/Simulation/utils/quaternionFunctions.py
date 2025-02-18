# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import sin, cos
from numpy.linalg import norm


# Normalize quaternion, or any vector
def vectNormalize(q):
    return q/norm(q)


# Quaternion multiplication
def quatMultiply(q, p):
    Q = np.array([[q[0], -q[1], -q[2], -q[3]],
                  [q[1],  q[0], -q[3],  q[2]],
                  [q[2],  q[3],  q[0], -q[1]],
                  [q[3], -q[2],  q[1],  q[0]]])
    return Q@p


# Inverse quaternion
def inverse(q):
    qinv = np.array([q[0], -q[1], -q[2], -q[3]])/norm(q)
    return qinv