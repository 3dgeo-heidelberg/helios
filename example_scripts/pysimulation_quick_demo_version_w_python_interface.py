import os
import pyhelios
import time
import numpy as np
from pyhelios.survey import Survey
from pyhelios.primitives import SimulationCycleCallback
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
        pyhelios.PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE.notify()


# ---  M A I N  --- #
# ----------------- #
if __name__ == '__main__':
    # Configure simulation context
    pyhelios.logging_verbose()
    pyhelios.default_rand_generator_seed("123")

    # Build a simulation
    survey = Survey.from_xml('data/surveys/toyblocks/als_toyblocks.xml')
    survey.las_output = False
    survey.zip_output = False
    survey.final_output = True
    survey.write_waveform = True
    survey.calc_echowidth = True
    survey.fullwavenoise = False
    survey.is_platform_noise_disabled = True

    
    simulation_callback = SimulationCycleCallback() 
    survey.callback = simulation_callback
    survey.output_path = 'output/'
    survey.run(num_threads=0, callback_frequency=10, export_to_file=False)
   # survey.resume()
    output = survey.join_output()

    measurements = output[0]
    trajectories = output[1]

    p1Pos = measurements[0].position
    pnPos = measurements[len(measurements)-1].position

    # PyHelios Tools
    npMeasurements, npTrajectories = pyhelios.outputToNumpy(output)

    coords = npMeasurements[:, 0:3]
    spher = pyhelios.cartesianToSpherical(coords)
    cart = pyhelios.sphericalToCartesian(spher)

