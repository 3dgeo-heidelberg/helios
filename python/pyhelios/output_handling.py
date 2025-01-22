import numpy as np


def outputToList(output=None):
    """Obtain the output as two different lists, the first one containing
    measurements and the second one containing trajectories.

    Arguments:
        output --- The output returned by a simulation

    Return:
        list of measurements, list of trajectories
    """
    # Prepare
    measurements, trajectories, _, _, _ = output
    lMeasurements = []  # Initialize as empty list
    lTrajectories = []  # Initialize as empty list

    # Fill measurements
    for meas in measurements:
        pos = meas.position
        ori = meas.beam_origin
        dir = meas.beam_direction
        lMeasurements.append([
            pos[0], pos[1], pos[2],
            ori[0], ori[1], ori[2],
            dir[0], dir[1], dir[2],
            meas.intensity,  # 9
            meas.echo_width,  # 10
            meas.return_number,  # 11
            meas.pulse_return_number,  # 12
            meas.fullwave_index,  # 13
            int(meas.hit_object_id),  # 14
            meas.classification,  # 15
            meas.gps_time  # 16
        ])

    # Fill trajectories
    for traj in trajectories:
        pos = traj.position
        lTrajectories.append([
            pos[0], pos[1], pos[2],
            traj.gps_time,
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

    # Convert lists to numpy arrays
    return np.array(lMeasurements, dtype=np.float64), np.array(lTrajectories, dtype=np.float64)