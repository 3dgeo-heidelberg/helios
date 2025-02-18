# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

# Position and Velocity Control based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/PositionControl.cpp
# Desired Thrust to Desired Attitude based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/Utility/ControlMath.cpp
# Attitude Control based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
# and https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
# Rate Control based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp

import numpy as np
from numpy import pi
from numpy import sin, cos, tan, sqrt
from numpy.linalg import norm
from Quadcopter_SimCon.Simulation import utils
from Quadcopter_SimCon.Simulation import config

rad2deg = 180.0/pi
deg2rad = pi/180.0

# Set PID Gains and Max Values
# ---------------------------

# Position P gains
Py    = 1.0
Px    = Py
Pz    = 1.0

pos_P_gain = np.array([Px, Py, Pz])

# Velocity P-D gains
Pxdot = 5.0
Dxdot = 0.5
Ixdot = 5.0

Pydot = Pxdot
Dydot = Dxdot
Iydot = Ixdot

Pzdot = 4.0
Dzdot = 0.5
Izdot = 5.0

vel_P_gain = np.array([Pxdot, Pydot, Pzdot])
vel_D_gain = np.array([Dxdot, Dydot, Dzdot])
vel_I_gain = np.array([Ixdot, Iydot, Izdot])

# Attitude P gains
Pphi = 8.0
Ptheta = Pphi
Ppsi = 1.5
PpsiStrong = 8

att_P_gain = np.array([Pphi, Ptheta, Ppsi])

# Rate P-D gains
Pp = 1.5
Dp = 0.04

Pq = Pp
Dq = Dp 

Pr = 1.0
Dr = 0.1

rate_P_gain = np.array([Pp, Pq, Pr])
rate_D_gain = np.array([Dp, Dq, Dr])

# Max Velocities
uMax = 5.0
vMax = 5.0
wMax = 5.0

velMax = np.array([uMax, vMax, wMax])
velMaxAll = 5.0

saturateVel_separetely = False

# Max tilt
tiltMax = 50.0*deg2rad

# Max Rate
pMax = 200.0*deg2rad
qMax = 200.0*deg2rad
rMax = 150.0*deg2rad

rateMax = np.array([pMax, qMax, rMax])


class Control:
    
    def __init__(self, quad, yawType):
        self.sDesCalc = np.zeros(16)
        self.w_cmd = np.ones(4)*quad.params["w_hover"]
        self.thr_int = np.zeros(3)
        if (yawType == 0):
            att_P_gain[2] = 0
        self.setYawWeight()
        self.pos_sp    = np.zeros(3)
        self.vel_sp    = np.zeros(3)
        self.acc_sp    = np.zeros(3)
        self.thrust_sp = np.zeros(3)
        self.eul_sp    = np.zeros(3)
        self.pqr_sp    = np.zeros(3)
        self.yawFF     = np.zeros(3)

    
    def controller(self, traj, quad, sDes, Ts):

        # Desired State (Create a copy, hence the [:])
        # ---------------------------
        self.pos_sp[:]    = traj.sDes[0:3]
        self.vel_sp[:]    = traj.sDes[3:6]
        self.acc_sp[:]    = traj.sDes[6:9]
        self.thrust_sp[:] = traj.sDes[9:12]
        self.eul_sp[:]    = traj.sDes[12:15]
        self.pqr_sp[:]    = traj.sDes[15:18]
        self.yawFF[:]     = traj.sDes[18]
        
        # Select Controller
        # ---------------------------
        if (traj.ctrlType == "xyz_vel"):
            self.saturateVel()
            self.z_vel_control(quad, Ts)
            self.xy_vel_control(quad, Ts)
            self.thrustToAttitude(quad, Ts)
            self.attitude_control(quad, Ts)
            self.rate_control(quad, Ts)
        elif (traj.ctrlType == "xy_vel_z_pos"):
            self.z_pos_control(quad, Ts)
            self.saturateVel()
            self.z_vel_control(quad, Ts)
            self.xy_vel_control(quad, Ts)
            self.thrustToAttitude(quad, Ts)
            self.attitude_control(quad, Ts)
            self.rate_control(quad, Ts)
        elif (traj.ctrlType == "xyz_pos"):
            self.z_pos_control(quad, Ts)
            self.xy_pos_control(quad, Ts)
            self.saturateVel()
            self.z_vel_control(quad, Ts)
            self.xy_vel_control(quad, Ts)
            self.thrustToAttitude(quad, Ts)
            self.attitude_control(quad, Ts)
            self.rate_control(quad, Ts)

        # Mixer
        # --------------------------- 
        self.w_cmd = utils.mixerFM(quad, norm(self.thrust_sp), self.rateCtrl)
        
        # Add calculated Desired States
        # ---------------------------         
        self.sDesCalc[0:3] = self.pos_sp
        self.sDesCalc[3:6] = self.vel_sp
        self.sDesCalc[6:9] = self.thrust_sp
        self.sDesCalc[9:13] = self.qd
        self.sDesCalc[13:16] = self.rate_sp


    def z_pos_control(self, quad, Ts):
       
        # Z Position Control
        # --------------------------- 
        pos_z_error = self.pos_sp[2] - quad.pos[2]
        self.vel_sp[2] += pos_P_gain[2]*pos_z_error
        
    
    def xy_pos_control(self, quad, Ts):

        # XY Position Control
        # --------------------------- 
        pos_xy_error = (self.pos_sp[0:2] - quad.pos[0:2])
        self.vel_sp[0:2] += pos_P_gain[0:2]*pos_xy_error
        
        
    def saturateVel(self):

        # Saturate Velocity Setpoint
        # --------------------------- 
        # Either saturate each velocity axis separately, or total velocity (prefered)
        if (saturateVel_separetely):
            self.vel_sp = np.clip(self.vel_sp, -velMax, velMax)
        else:
            totalVel_sp = norm(self.vel_sp)
            if (totalVel_sp > velMaxAll):
                self.vel_sp = self.vel_sp/totalVel_sp*velMaxAll


    def z_vel_control(self, quad, Ts):
        
        # Z Velocity Control (Thrust in D-direction)
        # ---------------------------
        # Hover thrust (m*g) is sent as a Feed-Forward term, in order to 
        # allow hover when the position and velocity error are nul
        vel_z_error = self.vel_sp[2] - quad.vel[2]
        if (config.orient == "NED"):
            thrust_z_sp = vel_P_gain[2]*vel_z_error - vel_D_gain[2]*quad.vel_dot[2] + quad.params["mB"]*(self.acc_sp[2] - quad.params["g"]) + self.thr_int[2]
        elif (config.orient == "ENU"):
            thrust_z_sp = vel_P_gain[2]*vel_z_error - vel_D_gain[2]*quad.vel_dot[2] + quad.params["mB"]*(self.acc_sp[2] + quad.params["g"]) + self.thr_int[2]
        
        # Get thrust limits
        if (config.orient == "NED"):
            # The Thrust limits are negated and swapped due to NED-frame
            uMax = -quad.params["minThr"]
            uMin = -quad.params["maxThr"]
        elif (config.orient == "ENU"):
            uMax = quad.params["maxThr"]
            uMin = quad.params["minThr"]

        # Apply Anti-Windup in D-direction
        stop_int_D = (thrust_z_sp >= uMax and vel_z_error >= 0.0) or (thrust_z_sp <= uMin and vel_z_error <= 0.0)

        # Calculate integral part
        if not (stop_int_D):
            self.thr_int[2] += vel_I_gain[2]*vel_z_error*Ts * quad.params["useIntergral"]
            # Limit thrust integral
            self.thr_int[2] = min(abs(self.thr_int[2]), quad.params["maxThr"])*np.sign(self.thr_int[2])

        # Saturate thrust setpoint in D-direction
        self.thrust_sp[2] = np.clip(thrust_z_sp, uMin, uMax)

    
    def xy_vel_control(self, quad, Ts):
        
        # XY Velocity Control (Thrust in NE-direction)
        # ---------------------------
        vel_xy_error = self.vel_sp[0:2] - quad.vel[0:2]
        thrust_xy_sp = vel_P_gain[0:2]*vel_xy_error - vel_D_gain[0:2]*quad.vel_dot[0:2] + quad.params["mB"]*(self.acc_sp[0:2]) + self.thr_int[0:2]

        # Max allowed thrust in NE based on tilt and excess thrust
        thrust_max_xy_tilt = abs(self.thrust_sp[2])*np.tan(tiltMax)
        thrust_max_xy = sqrt(quad.params["maxThr"]**2 - self.thrust_sp[2]**2)
        thrust_max_xy = min(thrust_max_xy, thrust_max_xy_tilt)

        # Saturate thrust in NE-direction
        self.thrust_sp[0:2] = thrust_xy_sp
        if (np.dot(self.thrust_sp[0:2].T, self.thrust_sp[0:2]) > thrust_max_xy**2):
            mag = norm(self.thrust_sp[0:2])
            self.thrust_sp[0:2] = thrust_xy_sp/mag*thrust_max_xy
        
        # Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
        # see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
        arw_gain = 2.0/vel_P_gain[0:2]
        vel_err_lim = vel_xy_error - (thrust_xy_sp - self.thrust_sp[0:2])*arw_gain
        self.thr_int[0:2] += vel_I_gain[0:2]*vel_err_lim*Ts * quad.params["useIntergral"]
    
    def thrustToAttitude(self, quad, Ts):
        
        # Create Full Desired Quaternion Based on Thrust Setpoint and Desired Yaw Angle
        # ---------------------------
        yaw_sp = self.eul_sp[2]

        # Desired body_z axis direction
        body_z = -utils.vectNormalize(self.thrust_sp)
        if (config.orient == "ENU"):
            body_z = -body_z
        
        # Vector of desired Yaw direction in XY plane, rotated by pi/2 (fake body_y axis)
        y_C = np.array([-sin(yaw_sp), cos(yaw_sp), 0.0])
        
        # Desired body_x axis direction
        body_x = np.cross(y_C, body_z)
        body_x = utils.vectNormalize(body_x)
        
        # Desired body_y axis direction
        body_y = np.cross(body_z, body_x)

        # Desired rotation matrix
        R_sp = np.array([body_x, body_y, body_z]).T

        # Full desired quaternion (full because it considers the desired Yaw angle)
        self.qd_full = utils.RotToQuat(R_sp)
        
        
    def attitude_control(self, quad, Ts):

        # Current thrust orientation e_z and desired thrust orientation e_z_d
        e_z = quad.dcm[:,2]
        e_z_d = -utils.vectNormalize(self.thrust_sp)
        if (config.orient == "ENU"):
            e_z_d = -e_z_d

        # Quaternion error between the 2 vectors
        qe_red = np.zeros(4)
        qe_red[0] = np.dot(e_z, e_z_d) + sqrt(norm(e_z)**2 * norm(e_z_d)**2)
        qe_red[1:4] = np.cross(e_z, e_z_d)
        qe_red = utils.vectNormalize(qe_red)
        
        # Reduced desired quaternion (reduced because it doesn't consider the desired Yaw angle)
        self.qd_red = utils.quatMultiply(qe_red, quad.quat)

        # Mixed desired quaternion (between reduced and full) and resulting desired quaternion qd
        q_mix = utils.quatMultiply(utils.inverse(self.qd_red), self.qd_full)
        q_mix = q_mix*np.sign(q_mix[0])
        q_mix[0] = np.clip(q_mix[0], -1.0, 1.0)
        q_mix[3] = np.clip(q_mix[3], -1.0, 1.0)
        self.qd = utils.quatMultiply(self.qd_red, np.array([cos(self.yaw_w*np.arccos(q_mix[0])), 0, 0, sin(self.yaw_w*np.arcsin(q_mix[3]))]))

        # Resulting error quaternion
        self.qe = utils.quatMultiply(utils.inverse(quad.quat), self.qd)

        # Create rate setpoint from quaternion error
        self.rate_sp = (2.0*np.sign(self.qe[0])*self.qe[1:4])*att_P_gain
        
        # Limit yawFF
        self.yawFF = np.clip(self.yawFF, -rateMax[2], rateMax[2])

        # Add Yaw rate feed-forward
        self.rate_sp += utils.quat2Dcm(utils.inverse(quad.quat))[:,2]*self.yawFF

        # Limit rate setpoint
        self.rate_sp = np.clip(self.rate_sp, -rateMax, rateMax)


    def rate_control(self, quad, Ts):
        
        # Rate Control
        # ---------------------------
        rate_error = self.rate_sp - quad.omega
        self.rateCtrl = rate_P_gain*rate_error - rate_D_gain*quad.omega_dot     # Be sure it is right sign for the D part
        

    def setYawWeight(self):
        
        # Calculate weight of the Yaw control gain
        roll_pitch_gain = 0.5*(att_P_gain[0] + att_P_gain[1])
        self.yaw_w = np.clip(att_P_gain[2]/roll_pitch_gain, 0.0, 1.0)

        att_P_gain[2] = roll_pitch_gain

    