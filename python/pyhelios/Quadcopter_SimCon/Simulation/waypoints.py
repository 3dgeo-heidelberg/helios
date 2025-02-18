# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
from Quadcopter_SimCon.Simulation import config

deg2rad = pi/180.0

def makeWaypoints():
    
    v_average = 1.6
    wp=np.loadtxt("3dpoints.txt")
    wp=wp-wp[0,:]
    yaw=np.random.rand(np.shape(wp)[0])*360
    t=np.linspace(0,50,np.shape(wp)[0])
    yaw =yaw.astype(float)*deg2rad
    return t, wp, yaw, v_average
