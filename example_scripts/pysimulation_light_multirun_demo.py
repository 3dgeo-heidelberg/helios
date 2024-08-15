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
        measurements = output[0]

        # Set 1st cycle point
        if cycleMeasurementsCount == 0 and len(measurements) > 0:
            pos = measurements[0].position
            cp1.append(pos[0])
            cp1.append(pos[1])
            cp1.append(pos[2])

        # Update cycle measurement count
        cycleMeasurementsCount += len(measurements)

        # Update last cycle point
        if len(measurements) > 0:
            pos = measurements[len(measurements)-1].position
            cpn[0] = pos[0]
            cpn[1] = pos[1]
            cpn[2] = pos[2]

        # Notify for conditional variable
        cv.notify()


# ---  M A I N  --- #
# ----------------- #
if __name__ == '__main__':
    # Configure simulation context
    # pyhelios.loggingVerbose2()
    pyhelios.logging_quiet()
    pyhelios.default_rand_generator_seed("123")

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
        for j in range(sim_curr.num_legs):
            leg = sim_curr.get_leg(j)
            leg.scanner_settings.pulse_frequency += i * 50000
        sims.append(sim_curr)

    print('>> Running {n} simulations\n'.format(n=len(sims)))

    # Run multiple simulations
    for i, simulation in enumerate(sims):
        # Run the simulation
        print('>> Running simulation {i}'.format(i=i+1))
        cycleMeasurementsCount = 0
        cp1 = []
        cpn = [0, 0, 0]
        print('Pulse frequency: {f} '.format(f=simulation.get_leg(0).scanner_settings.pulse_frequency))
        simulation.start()

        # Join simulation thread
        with cv:  # Conditional variable necessary for callback mode only
            output = simulation.join()
            while not output[4]:  # Loop necessary for callback mode only
                cv.wait()
                output = simulation.join()

        # Digest output
        measurements = output[0]
        trajectories = output[1]
        print('\tSimulation {i}:'.format(i=i+1))
        print('\t\tnumber of measurements : {n}'.format(n=len(measurements)))
        print('\t\tnumber of trajectories: {n}'.format(n=len(trajectories)))
        p1Pos = measurements[0].position
        pnPos = measurements[len(measurements)-1].position
        print('\t\tp1 position  : ({x}, {y}, {z})'.format(
            x=p1Pos[0], y=p1Pos[1], z=p1Pos[2]))
        print('\t\tcp1 position : ({x}, {y}, {z})'.format(
            x=cp1[0], y=cp1[1], z=cp1[2]))
        print('\t\tpn position  : ({x}, {y}, {z})'.format(
            x=pnPos[0], y=pnPos[1], z=pnPos[2]))
        print('\t\tcpn position : ({x}, {y}, {z})'.format(
            x=cpn[0], y=cpn[1], z=cpn[2]))

    print("Simulated point clouds saved to {folder}".format(folder=str(Path(output[2]).parent)))