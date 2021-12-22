# Light multirun demo shows how to launch multiple simulations when they are
# small sized
# As small simulations do not require too much memory, they can be all keeped
# in memory at the same time

import pyhelios
from threading import Condition as CondVar

cv = CondVar()


# ---  C A L L B A C K  --- #
# ------------------------- #
def callback(output=None):
    with cv:
        global cycleMeasurementsCount
        global cp1
        global cpn
        measurements = output.measurements

        # Set 1st cycle point
        if cycleMeasurementsCount == 0 and len(measurements) > 0:
            pos = measurements[0].getPosition()
            cp1.append(pos.x)
            cp1.append(pos.y)
            cp1.append(pos.z)

        # Update cycle measurement count
        cycleMeasurementsCount += len(measurements)

        # Update last cycle point
        if len(measurements) > 0:
            pos = measurements[len(measurements)-1].getPosition()
            cpn[0] = pos.x
            cpn[1] = pos.y
            cpn[2] = pos.z

        # Notify for conditional variable
        cv.notify()


# ---  M A I N  --- #
# ----------------- #
if __name__ == '__main__':
    # Configure simulation context
    # pyhelios.loggingVerbose2()
    pyhelios.loggingQuiet()
    pyhelios.setDefaultRandomnessGeneratorSeed("123")

    # Build multiple simulations
    nSimulations = 3
    sims = []
    print('>> Creating {n} simulations\n'.format(n=nSimulations))

    sim0 = pyhelios.Simulation(  # First simulation
        'data/surveys/voxels/als_detailedVoxels_mode_comparison.xml',
        'assets/',
        'output/',
        0,          # Num Threads
        False,      # LAS output FLAG
        False       # Zip output FLAG
    )
    sim0.simFrequency = 10
    # Sim frequency has to be setted
    # It is 0 by default and with 0 sim frequency it is not possible
    # to pause nor have callbacks
    # Sim frequency 0 means the simulation will start and run until it is
    # finished with no interleaved work
    sim0.finalOutput = True
    sim0.loadSurvey(
        True,       # Leg Noise Disabled FLAG
        True,       # Rebuild Scene FLAG
        True,       # Write Waveform FLAG
        True,       # Calc Echowidth FLAG
        False,      # Full Wave Noise FLAG
        True        # Platform Noise Disabled FLAG
    )
    sim0.setCallback(callback)
    sims.append(sim0)

    for i in range(1, nSimulations):  # Copies of first simulation
        sim = sim0.copy()
        sim.simFrequency += i+1
        sims.append(sim)

    print('>> Simulation frequencies = [ ', end='')
    for sim in sims:
        print('{f} '.format(f=sim.simFrequency), end='')
    print(']')

    print('>> Running {n} simulations\n'.format(n=len(sims)))

    # Run multiple simulations
    for i, sim in enumerate(sims):
        # Run the simulation
        print('>> Running simulation {i}'.format(i=i+1))
        cycleMeasurementsCount = 0
        cp1 = []
        cpn = [0, 0, 0]
        sim.start()

        # Join simulation thread
        with cv:  # Conditional variable necessary for callback mode only
            output = sim.join()
            while not output.finished:  # Loop necessary for callback mode only
                cv.wait()
                output = sim.join()

        # Digest output
        measurements = output.measurements
        trajectories = output.trajectories
        print('\tSimulation {i}:'.format(i=i+1))
        print('\t\tnumber of measurements : {n}'.format(n=len(measurements)))
        print('number of trajectories: {n}'.format(n=len(trajectories)))
        p1Pos = measurements[0].getPosition()
        pnPos = measurements[len(measurements)-1].getPosition()
        print('\t\tp1 position  : ({x}, {y}, {z})'.format(
            x=p1Pos.x, y=p1Pos.y, z=p1Pos.z))
        print('\t\tcp1 position : ({x}, {y}, {z})'.format(
            x=cp1[0], y=cp1[1], z=cp1[2]))
        print('\t\tpn position  : ({x}, {y}, {z})'.format(
            x=pnPos.x, y=pnPos.y, z=pnPos.z))
        print('\t\tcpn position : ({x}, {y}, {z})'.format(
            x=cpn[0], y=cpn[1], z=cpn[2]))
