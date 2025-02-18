# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

# Select Orientation of Quadcopter and Reference Frame
# ---------------------------
# "NED" for front-right-down (frd) and North-East-Down
# "ENU" for front-left-up (flu) and East-North-Up
orient = "NED"

# Select whether to use gyroscopic precession of the rotors in the quadcopter dynamics
# ---------------------------
# Set to False if rotor inertia isn't known (gyro precession has negigeable effect on drone dynamics)
usePrecession = bool(False)