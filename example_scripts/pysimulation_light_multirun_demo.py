# Light multirun demo shows how to launch multiple simulations when they are
# small sized
# As small simulations do not require too much memory, they can be all kept
# in memory at the same time

from pathlib import Path
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

    simBuilder = pyhelios.SimulationBuilder(
        'data/surveys/voxels/als_detailedVoxels_mode_comparison.xml',
        'assets/',
        'output/'
    )
    simBuilder.setCallbackFrequency(10)
    # Callback frequency has to be set
    # It is 0 by default and with 0 sim frequency it is not possible
    # to pause nor have callbacks
    # Sim frequency 0 means the simulation will start and run until it is
    # finished with no interleaved work
    simBuilder.setFinalOutput(True)
    simBuilder.setCallback(callback)
    sim0 = simBuilder.build()
    sims.append(sim0.sim)

    for i in range(1, nSimulations):  # Copies of first simulation
        sim_curr = sim0.sim.copy()
        for j in range(sim_curr.getNumLegs()):
            leg = sim_curr.getLeg(j)
            leg.getScannerSettings().pulseFreq += i * 50000
        sims.append(sim_curr)

    print('>> Running {n} simulations\n'.format(n=len(sims)))

    # Run multiple simulations
    for i, simulation in enumerate(sims):
        # Run the simulation
        print('>> Running simulation {i}'.format(i=i+1))
        cycleMeasurementsCount = 0
        cp1 = []
        cpn = [0, 0, 0]
        print('Pulse frequency: {f} '.format(f=simulation.getLeg(0).getScannerSettings().pulseFreq))
        simulation.start()

        # Join simulation thread
        with cv:  # Conditional variable necessary for callback mode only
            output = simulation.join()
            while not output.finished:  # Loop necessary for callback mode only
                cv.wait()
                output = simulation.join()

        # Digest output
        measurements = output.measurements
        trajectories = output.trajectories
        print('\tSimulation {i}:'.format(i=i+1))
        print('\t\tnumber of measurements : {n}'.format(n=len(measurements)))
        print('\t\tnumber of trajectories: {n}'.format(n=len(trajectories)))
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

    print("Simulated point clouds saved to {folder}".format(folder=str(Path(output.filepath).parent)))