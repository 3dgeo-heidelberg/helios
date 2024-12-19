# Heavy multirun demo shows how to launch multiple simulations when they are
# big sized.
# The approach consists in having the reference simulation, copying it and
# modifying one time per execution.
# This way, once the iteration has finished, the simulation is released and
# memory becomes available again

import sys
import os
from pathlib import Path
from threading import Condition as CondVar
import pyhelios

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

    # Build reference simulation
    print('>> Creating base/reference simulation\n')
    simB = pyhelios.SimulationBuilder(
        surveyPath='data/surveys/voxels/als_detailedVoxels_mode_comparison.xml',
        assetsDir='assets/',
        outputDir='output/',
    )
    simB.setCallbackFrequency(10)
    # Callback frequency has to be setted
    # It is 0 by default and with 0 sim frequency it is not possible
    # to pause nor have callbacks
    # Sim frequency 0 means the simulation will start and run until it is
    # finished with no interleaved work
    simB.setFinalOutput(True)
    simB.setNumThreads(0)
    simB.setCallback(callback)

    sim0 = simB.build()  # First simulation

    # Run multiple simulations
    nSimulations = 3
    print('>> Running {n} simulations\n'.format(n=nSimulations))
    for i in range(nSimulations):
        # Run the simulation
        print('>> Running simulation {i}'.format(i=i+1))
        sim_curr = sim0.sim.copy()
        for j in range(sim_curr.num_legs):
            leg = sim_curr.get_leg(j)
            leg.scanner_settings.pulse_frequency += i * 50000
        print('Pulse frequency: {f} '.format(f=leg.scanner_settings.pulse_frequency))
        sim_curr.callback_frequency += i
        cycleMeasurementsCount = 0
        cp1 = []
        cpn = [0, 0, 0]
        sim_curr.start()

        # Join simulation thread
        with cv:  # Conditional variable necessary for callback mode only
            output = sim_curr.join()
            while not output[4]:  # Loop necessary for callback mode only
                cv.wait()
                output = sim_curr.join()

        # Digest output
        measurements = output[0]
        trajectories = output[1]
        print('\tSimulation {i}:'.format(i=i+1))
        print('\t\tnumber of measurements : {n}'.format(n=len(measurements)))
        print('number of trajectories: {n}'.format(n=len(trajectories)))
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
