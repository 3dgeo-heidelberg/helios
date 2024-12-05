import os
import pyhelios
import time
import numpy as np
from pyhelios.survey import Survey
from pyhelios.primitives import SimulationCycleCallback

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
    print('number of measurements : {n}'.format(n=len(measurements)))
    print('number of trajectories: {n}'.format(n=len(trajectories)))
    p1Pos = measurements[0].position
    pnPos = measurements[len(measurements)-1].position
    print('p1 position : ({x}, {y}, {z})'.format(
        x=p1Pos[0], y=p1Pos[1], z=p1Pos[2]))
    print('pn position : ({x}, {y}, {z})'.format(
        x=pnPos[0], y=pnPos[1], z=pnPos[2]))
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