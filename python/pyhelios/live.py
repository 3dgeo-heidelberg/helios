from pyhelios import pyhelios_argparser
from pyhelios.pyh_obj import *

import pyhelios
import sys

# Empty list for trajectory values and/or measurement values
tpoints = []
mpoints = []
callback_counter = 0


# Define callback function to continuously extract measurement (and trajectory)
# values from simulation in case of live plotting.
def callback(output=None):
    global mpoints
    global tpoints
    global callback_counter

    callback_counter += 1

    n = 10

    # Executes in every nth iteration of callback function.
    if callback_counter >= n:

        # Extract trajectory points.
        trajectories = output.trajectories

        if len(trajectories) != 0:
            tpoints.append([trajectories[len(trajectories) - 1].getPosition().x,
                        trajectories[len(trajectories) - 1].getPosition().y,
                        trajectories[len(trajectories) - 1].getPosition().z])

            callback_counter = 0

    # Extract measurement points.
    measurements = output.measurements

    if len(measurements) == 0:
        return

    # Add current values to list.
    try:
        mpoints.append([measurements[len(measurements) - 1].getPosition().x,
                        measurements[len(measurements) - 1].getPosition().y,
                        measurements[len(measurements) - 1].getPosition().z,
                        int(measurements[len(measurements) - 1].hitObjectId)])

    except Exception as err:
        print(err)
        pass


# ---  M A I N  --- #
# ----------------- #
def helios_live():
    # Parse arguments.
    args = pyhelios_argparser.args

    # Set logging style.
    if args.loggingv:
        pyhelios.loggingVerbose()

    elif args.loggingv2:
        pyhelios.loggingVerbose2()

    elif args.loggingquiet:
        pyhelios.loggingQuiet()

    elif args.loggingsilent:
        pyhelios.loggingSilent()

    else:
        pyhelios.loggingDefault()

    # Set random generator seed if value has been supplied.
    if args.randomness_seed:
        pyhelios.setDefaultRandomnessGeneratorSeed(args.randomness_seed)

    # Build a simulation
    simBuilder = pyhelios.SimulationBuilder(
        args.survey_file,
        args.assets_path,
        args.output_path
    )

    # Configure sim
    simBuilder.setNumThreads(args.number_of_threads)
    simBuilder.setLasOutput(args.las_output_flag)
    simBuilder.setZipOutput(args.zip_output_flag)
    simBuilder.setCallbackFrequency(1)
    simBuilder.setLegNoiseDisabled(args.leg_noise_disabled_flag)
    simBuilder.setRebuildScene(args.rebuild_scene_flag)
    simBuilder.setWriteWaveform(args.write_waveform_flag)
    simBuilder.setCalcEchowidth(args.calc_echowidth_flag)
    simBuilder.setFullwaveNoise(args.fullwave_noise_flag)
    simBuilder.setPlatformNoiseDisabled(args.platform_noise_disabled_flag)
    simBuilder.setCallback(callback)

    # Build sim
    sim = simBuilder.build()

    if not args.open3d:
        sim.start()
        output = sim.join()

    elif args.open3d:
        try:
            import open3d as o3d
        except ImportError:
            print('Open3D is not installed. Please install Open3D with "pip install open3d".')
            sys.exit()
        
        import numpy as np
        import time

        # Create instance of Scene class, generate scene, print scene (if logging v2), and visualize.
        scene = Scene(args.survey_file, args.loggingv2)
        scene.gen_from_xml()

        if args.loggingv2:
            scene.print_scene()

        scene.visualize()

        # Start pyhelios simulation.
        sim.start()

        while sim.isRunning():
            if len(mpoints) > 0:
                # Convert x, y, z lists with points from pyhelios callback to arrays.
                # Update o3d point clouds with points from callback.
                a = np.array(mpoints)
                scene.measurement.points = o3d.utility.Vector3dVector(a[:, :-1])

                if len(tpoints) > 0:
                    b = np.array(tpoints)
                    scene.trajectory.points = o3d.utility.Vector3dVector(b)

                # Apply colours to trajectory and measurement geometries.
                # Measurement array needed for dimensions.
                scene.colourise(a)

                # Update geometries for visualizer.
                scene.visualizer.update_geometry(scene.measurement)
                scene.visualizer.update_geometry(scene.trajectory)

                # Refresh GUI.
                scene.visualizer.poll_events()
                scene.visualizer.update_renderer()

            time.sleep(0.1)

        # Save final output in variable.
        output = sim.join()

        # Keep window open after sim.
        scene.visualizer.run()


if __name__ == '__main__':
    helios_live()
