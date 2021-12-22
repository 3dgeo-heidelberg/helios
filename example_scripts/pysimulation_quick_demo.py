import pyhelios
import time
import numpy as np

cycleMeasurementsCount = 0
cp1 = []
cpn = [0, 0, 0]


# ---  C A L L B A C K  --- #
# ------------------------- #
def callback(output=None):
    with pyhelios.PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE:
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
        pht.PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE.notify()


# ---  M A I N  --- #
# ----------------- #
if __name__ == '__main__':
    # Configure simulation context
    pyhelios.loggingVerbose()
    #pyhelios.loggingVerbose2()
	#pyhelios.loggingQuiet()
    pyhelios.setDefaultRandomnessGeneratorSeed("123")

    # Build a simulation
    simBuilder = pyhelios.SimulationBuilder(
        'data/surveys/toyblocks/als_toyblocks.xml',
        'assets/',
        'output/'
    )
    simBuilder.setNumThreads(0)
    simBuilder.setLasOutput(False)
    simBuilder.setZipOutput(False)
    # Sim frequency 0 means the simulation will start and run until it is
    # finished with no interleaved work (no callbacks)
    simBuilder.setSimFrequency(10)
    simBuilder.setFinalOutput(True)
    simBuilder.setLegNoiseDisabled(True)
    simBuilder.setRebuildScene(True)
    simBuilder.setWriteWaveform(True)
    simBuilder.setCalcEchowidth(True)
    simBuilder.setFullwaveNoise(False)
    simBuilder.setPlatformNoiseDisabled(True)
    simBuilder.setCallback(callback)
    sim = simBuilder.build()

    # Run the simulation
    sim.start()

    # Pause and resume simulation
    sim.pause()
    print('Simulation paused!')
    time.sleep(2.5)
    print('Resuming simulation ...')
    time.sleep(0.5)
    sim.resume()
    print('Simulation resumed!')

    # Join simulation thread
    output = sim.join()

    # Digest output
    measurements = output.measurements
    trajectories = output.trajectories
    print('number of cycle measurements: {n}'.format(
        n=cycleMeasurementsCount))
    print('number of measurements : {n}'.format(n=len(measurements)))
    print('number of trajectories: {n}'.format(n=len(trajectories)))
    p1Pos = measurements[0].getPosition()
    pnPos = measurements[len(measurements)-1].getPosition()
    print('p1 position : ({x}, {y}, {z})'.format(
        x=p1Pos.x, y=p1Pos.y, z=p1Pos.z))
    print('cp1 position : ({x}, {y}, {z})'.format(
        x=cp1[0], y=cp1[1], z=cp1[2]))
    print('pn position : ({x}, {y}, {z})'.format(
        x=pnPos.x, y=pnPos.y, z=pnPos.z))
    print('cpn position : ({x}, {y}, {z})'.format(
        x=cpn[0], y=cpn[1], z=cpn[2]))

    # PyHelios Tools
    npMeasurements, npTrajectories = pyhelios.outputToNumpy(output)
    print('Numpy measurements shape:', np.shape(npMeasurements))
    print('Numpy trajectories shape:', np.shape(npTrajectories))
    coords = npMeasurements[:, 0:3]
    spher = pyhelios.cartesianToSpherical(coords)
    cart = pyhelios.sphericalToCartesian(spher)
    print(
        'Max precision loss due to coordinate translation:',
        np.max(coords-cart)
    )
