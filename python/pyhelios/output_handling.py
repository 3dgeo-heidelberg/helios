import numpy as np


def outputToList(output=None):
    """Obtain the output as two different lists, the first one containing
    measurements and the seconds one containing trajectories.

    Arguments:
        output --- The output returned by a simulation

    Return:
        list of measurements, list of trajectories
    """
    # Prepare
    measurements = output.measurements
    nMeasurements = measurements.length()
    lMeasurements = []
    trajectories = output.trajectories
    nTrajectories = trajectories.length()
    lTrajectories = []

    # Fill
    for i in range(nMeasurements):
        meas = measurements[i]
        pos = meas.getPosition()
        ori = meas.getBeamOrigin()
        dir = meas.getBeamDirection()
        lMeasurements.append([
            pos.x, pos.y, pos.z,
            ori.x, ori.y, ori.z,
            dir.x, dir.y, dir.z,
            meas.intensity,  # 9
            meas.echoWidth,  # 10
            meas.returnNumber,  # 11
            meas.pulseReturnNumber,  # 12
            meas.fullwaveIndex,  # 13
            int(meas.hitObjectId),  # 14
            meas.classification,  # 15
            meas.gpsTime  # 16
        ])
    for i in range(nTrajectories):
        traj = trajectories[i]
        pos = traj.getPosition()
        lTrajectories.append([
            pos.x, pos.y, pos.z,
            traj.gpsTime,
            traj.roll,
            traj.pitch,
            traj.yaw
        ])

    # Return
    return lMeasurements, lTrajectories


def outputToNumpy(output=None):
    """Obtain the output as two different numpy arrays, the first one
    containing measurements and the second one containing trajectories.

    Arguments:
        output -- The output returned by a simulation

    Return:
        numpy array of measurements, numpy array of trajectories
    """
    # Obtain lists
    lMeasurements, lTrajectories = outputToList(output)

    # Return
    return np.array(lMeasurements), np.array(lTrajectories)
