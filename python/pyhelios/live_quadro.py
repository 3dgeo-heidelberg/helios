from pyhelios import pyhelios_argparser
from pyhelios.pyh_obj import *
import cv2
import pyhelios
import sys
#from Quadcopter_SimCon.Simulation import *
sys.path.append(os.path.join(os.path.dirname('Quadcopter_SimCon.Simulation.py'), '..'))
sys.path.append(os.path.dirname('Quadcopter_SimCon.Simulation.py'))
sys.path.append(os.path.join(os.path.dirname('Quadcopter_SimCon.Simulation.py'), 'Simulation'))
from Quadcopter_SimCon.Simulation import run_3D_simulation
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

def click_mouse_pos(event,x,y,flags,param):
    global x_clicked,y_clicked, clicked
    if event == cv2.EVENT_LBUTTONDBLCLK:
        x_clicked,y_clicked = x,y
        clicked=1

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

    scene.visualizer.capture_screen_image("quadro_image.png")
    depth = np.array(scene.visualizer.capture_depth_float_buffer())
    scene.visualizer.close()
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('image', click_mouse_pos)
    global clicked
    clicked=0
    k=ord('b')
    col_old=cv2.imread('quadro_image.png')
    col=np.copy(col_old)
    print('double click to insert object, after that, press "a" to continue')
    #wait for click in image, then insert person, ends when pressing 'a'
    array_clickpos=np.ones((0,3))
    array_clickpoints=np.zeros((0,3))
    k = cv2.waitKey(20)
    intrinsic_matrix = np.array(scene.visualizer.get_view_control().convert_to_pinhole_camera_parameters().intrinsic.intrinsic_matrix)
    extrinsic_matrix = scene.visualizer.get_view_control().convert_to_pinhole_camera_parameters().extrinsic
    while(k!=ord('a')):
        while(clicked==0):
            cv2.imshow('image',col[:,:,2::-1])
            k=cv2.waitKey(20)
            if k==ord('a'):
                break
        if k==ord('a'):
            break
        col=cv2.circle(col,(x_clicked,y_clicked),3,(255,0,0),3)
        array_clickpos=np.vstack([array_clickpos,np.array([x_clicked,y_clicked,1])])
        point3d=np.linalg.inv(intrinsic_matrix) @ np.array([[x_clicked],[y_clicked],[1]])
        point3d=point3d/point3d[2,0]*depth[y_clicked,x_clicked]
        point3d_4=np.ones((4,1))
        point3d_4[:3,:]=point3d
        point3d_4 = np.linalg.inv(extrinsic_matrix) @ point3d_4
        array_clickpoints=np.vstack([array_clickpoints,point3d_4[:3,0]])
        print('3D Point:')
        print(point3d_4[:3,0])
        clicked=0
        k=cv2.waitKey(20)
    np.savetxt('3dpoints.txt',array_clickpoints)

    run_3D_simulation.main()
    # Start pyhelios simulation.
    scene.visualize()
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
    scene.visualizer.run()
if __name__ == '__main__':
    helios_live()
