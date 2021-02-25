#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import pyhelios_argparser

# Configure runpath from within pyhelios.
helios_run_path = 'run/'

# Add run path to python path.
sys.path.append(helios_run_path)

# PyHelios import only now possible.
import pyhelios

# Empty list for trajectory values and/or measurement values
tpoints = []
mpoints = []
callback_counter = 0

# Define callback function for jupyter live trajectory plot.
# Extracts trajectory values from simulation.
def live_plotting_callback(output=None):
    global tpoints

    # Extract trajectory points.
    trajectories = output.trajectories

    '''if len(trajectories) == 0:
        return'''

    tpoints.append([trajectories[-1].getPosition().x,
                    trajectories[-1].getPosition().y,
                    trajectories[-1].getPosition().z])

# Define callback function to continuously extract measurement (and trajectory)
# values from simulation in case of live plotting.
def live_plotting_callback_full(output=None):
    global mpoints
    global tpoints
    global callback_counter

    callback_counter += 1

    n = 10

    # Executes in every nth iteration of callback function.
    if callback_counter % n == 0:

        # Extract trajectory points.
        trajectories = output.trajectories

        if len(trajectories) == 0:
            return

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


def update_3dline(dynamic_list, line_obj, refresh_rate=0.5):
    '''Continuously updates matplotlib line with values from a steadily growing list.
     The function terminates when no new values are added to the list within the specified refresh rate.

    Parameters:
    dynamic_list (list): A continuously growing list of xyz points. Format: [[x, y, z], [x, y, z], ...]).
    line_obj (matplotlib.axes.Axes.plot): An empty line with 3 dimensions. E.g.: l, = ax.plot([], [], []).
    figure (matplotlib.pyplot.figure): The figure which contains the axis used to the create line object.

    Optional:
    refresh_rate (float, default=0.5): Determines time interval in seconds between each iteration of function.
    '''
    loop_stopper = 0
    while True:
        time.sleep(float(refresh_rate))

        # Check whether list contains more new values.
        if loop_stopper != len(dynamic_list):

            # Convert x, y, z list to array.
            a = np.array(dynamic_list)

            # Update values of line with data from array.
            line_obj.set_xdata(a[:, 0])
            line_obj.set_ydata(a[:, 1])
            line_obj.set_3d_properties(zs=a[:, 2])

            # Draw results onto figure.
            plt.draw()
            plt.pause(1e-17)
        else:
            break
        # Variable used to check for updates to list.
        loop_stopper = len(dynamic_list)


# ---  M A I N  --- #
# ----------------- #
if __name__ == '__main__':

    # Parse arguments.
    args = pyhelios_argparser.args

    # Perform test to check whether PyHelios is properly configured and installed.
    if args.test_is_desired:
        print('PyHelios is installed correctly.')
        print('PyHelios version: ' + pyhelios.getVersion())
        exit()

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

    # Initiate a simulation. Parameters: (surveyPath, assetsPath, outputPath, ...).
    sim = pyhelios.Simulation(
        args.survey_file,
        args.assets_path,
        args.output_path,
        args.number_of_threads,  # Num Threads
        args.las_output_flag,  # LAS output
        args.zip_output_flag,  # ZIP output
    )

    # Load the survey file.
    sim.loadSurvey(
        args.leg_noise_disabled_flag,  # Leg Noise Disabled FLAG
        args.rebuild_scene_flag,  # Rebuild Scene FLAG
        args.write_waveform_flag,  # Write Waveform FLAG
        args.calc_echowidth_flag,  # Calculate Echowidth FLAG
        args.fullwave_noise_flag,  # Full Wave Noise FLAG
        args.platform_noise_disabled_flag  # Platform Noise Disabled FLAG
    )

    if not args.live_trajectory_plot and not args.open3d:
        sim.start()

        output = sim.join()

    if args.live_trajectory_plot:
        import time
        import matplotlib.pyplot as plt
        import matplotlib
        from mpl_toolkits.mplot3d import Axes3D, art3d
        import numpy as np

        # Set callback and callback frequency.
        sim.simFrequency = 10000
        sim.setCallback(live_plotting_callback)

        # Empty list for groundplane vals.
        groundplane_list = []

        # List filled every 25th value from (-75, -75, 0) to (75, 75, 0).
        for i in range(-75, 76, 25):
            for j in range(-75, 76, 25):
                groundplane_list.append([i, j, 0])

        # List converted to numpy Array.
        groundplane = np.array(groundplane_list)

        # Matplotlib figure.
        fig = plt.figure()

        # Axes3d axis onto mpl figure.
        ax = fig.add_subplot(111, projection='3d')

        # Set axis extent.
        ax.set_xlim([-75, 75])
        ax.set_ylim([-75, 75])
        ax.set_zlim([0, 100])

        # Add axis labels.
        ax.set_xlabel('$X$')
        ax.set_ylabel('$Y$')
        ax.set_zlabel('$Z$')

        # Set title.
        ax.text2D(0.185, 1, "LIVE:", fontsize='11', transform=ax.transAxes, c='r')
        ax.text2D(0.27, 1, 'PyHelios LiDAR Simulation Trajectory Plot', fontsize='11', transform=ax.transAxes)

        # Plot groundplane onto figure.
        ax.plot_trisurf(groundplane[:, 0], groundplane[:, 1], groundplane[:, 2], color='darkgoldenrod',
                        label='groundplane', alpha=0.5)

        # Update canvas.
        plt.draw()
        plt.pause(1e-17)

        sim.start()

        time.sleep(2)

        # Create empty line object.
        line1, = ax.plot([], [], [])

        # Run function to plot trajectory while simulation is running.
        update_3dline(tpoints, line1, refresh_rate=1)

        output = sim.join()

        plt.show()

    if args.open3d:
        import open3d as o3d
        import numpy as np
        import time
        import matplotlib.pyplot as plt
        import xml.etree.ElementTree as ET

        # Set callback function which retrieves measurement values.
        sim.setCallback(live_plotting_callback_full)
        sim.simFrequency = 10

        # Parse survey file to extract scene file and then parse the scene file.
        scene = ET.parse(ET.parse(args.survey_file).find('survey').attrib['scene'].split('#')[0])
        # Get scene name from survey file.
        scene_name = ET.parse(args.survey_file).find('survey').attrib['scene'].split('#')[1]
        
        # Access XML root of scene file.
        root = scene.getroot()

        # Extract scene data from XML.
        # Create a list and fill it with a series of dicts with data on each individual scene part.
        scene_parts = []
        for scene_part in root.find('scene'):

            scene_part_data = {}
            # Default value of efilepath is False. Flag is necessary because efilepath scenedef requires different set of steps than regular filepath def.
            efile_flag = False

            for filter in scene_part.findall('filter'):
                
                # --------------------------------------------------------------------------
                # GeoTIFF scenepart.
                # --------------------------------------------------------------------------
                # Get file path.
                if filter.attrib['type'] == 'geotiffloader':
                    scene_part_data['extension'] = '.tif'
                    scene_part_data['filepath'] = filter.find('param').attrib['value']

                # --------------------------------------------------------------------------
                # Wavefront scenepart.
                # --------------------------------------------------------------------------
                # Get file path.
                if filter.attrib['type'] == 'objloader':
                
                    # In case of efilepath, create new scenepart dictionary for every obj file in said directory.
                    # Also get tranformation data and add it to each scenepart.
                    if filter.find('param').attrib['key'] == 'efilepath':
                        import os
                        path = os.path.dirname(filter.find('param').attrib['value'])
                        
                        # Set ignore transformation to true to stop appending of tranformation after efilepath process.
                        efile_flag = True
                        
                        translation = False
                        scale = False
                        rotate = False
                        
                        # Get transformation. 
                        for filter in scene_part.findall('filter'):
                        
                            # Translate filter for point clouds and wavefront objects.
                            if filter.attrib['type'] == 'translate':
                                    translation = filter.find('param').attrib['value'].split(';')

                            # Scale filter, is a parameter for each respective scene part datatype.
                            if filter.attrib['type'] == 'scale':
                                    scale = float(filter.find('param').attrib['value'])

                            # Rotate filter, is a parameter for each respective scene part datatype.
                            if filter.attrib['type'] == 'rotate':
                                    rotate = []
                                    rotate = filter.attrib['rotations'] if 'rotations' in filter.attrib else 'global'
                                    for rotation in filter.find('param').findall('rot'):
                                        rotate.append([rotation.attrib['axis'], rotation.attrib['angle_deg']])
                                    
                        for file in os.listdir(path):
                            scene_part_data = {}
                            
                            # Add extension and filepath if file is 
                            if file.endswith(".obj"):
                                scene_part_data['extension'] = '.obj'
                                scene_part_data['filepath'] = os.path.join(path, file)
                                
                                # If transformation values exist, add them to scenepart definition of each scenepart file.
                                if rotate:
                                    scene_part_data['rotate'] = rotate
                                if translation:
                                    scene_part_data['translation'] = translation
                                if scale:
                                    scene_part_data['scale'] = scale
                                scene_parts.append(scene_part_data)
                    else:
                        scene_part_data['extension'] = '.obj'
                        scene_part_data['filepath'] = filter.find('param').attrib['value']

                # --------------------------------------------------------------------------
                # Point cloud scenepart.
                # --------------------------------------------------------------------------
                if filter.attrib['type'] == 'xyzloader':
                    
                    # In case of efilepath, create new scenepart dictionary for every obj file in said directory.
                    # Also get tranformation data and add it to each scenepart.
                    if filter.find('param').attrib['key'] == 'efilepath':
                        import os
                        path = os.path.dirname(filter.find('param').attrib['value'])
                        
                        # Set ignore transformation to true to stop appending of tranformation after efilepath process.
                        efile_flag = True
                        
                        translation = False
                        scale = False
                        rotate = False
                        voxelsize = 1
                        
                        # Get transformation. 
                        for filter in scene_part.findall('filter'):
                        
                            # Translate filter for point clouds and wavefront objects.
                            if filter.attrib['type'] == 'translate':
                                    translation = filter.find('param').attrib['value'].split(';')

                            # Scale filter, is a parameter for each respective scene part datatype.
                            if filter.attrib['type'] == 'scale':
                                    scale = float(filter.find('param').attrib['value'])

                            # Rotate filter, is a parameter for each respective scene part datatype.
                            if filter.attrib['type'] == 'rotate':
                                    rotate = []
                                    rotate = filter.attrib['rotations'] if 'rotations' in filter.attrib else 'global'
                                    for rotation in filter.find('param').findall('rot'):
                                        rotate.append([rotation.attrib['axis'], rotation.attrib['angle_deg']])
                            if filter.attrib['key'] == 'voxelSize':
                                voxelsize = float(filter.attrib['value'])
                                    
                        for file in os.listdir(path):
                            scene_part_data = {}
                            
                            # Add extension and filepath if file is 
                            if file.endswith(".xyz"):
                                scene_part_data['extension'] = '.xyz'
                                scene_part_data['filepath'] = os.path.join(path, file)
                                scene_part_data['voxelsize'] = voxelsize
                                
                                # If transformation values exist, add them to scenepart definition of each scenepart file.
                                if rotate:
                                    scene_part_data['rotate'] = rotate
                                if translation:
                                    scene_part_data['translation'] = translation
                                if scale:
                                    scene_part_data['scale'] = scale
                                scene_parts.append(scene_part_data)
                    else:
                        
                        scene_part_data['extension'] = '.xyz'
                        # Iterate through (large number of) different params of point cloud.
                        for param in filter.findall('param'):
                            if param.attrib['key'] == 'filepath':
                                scene_part_data['filepath'] = param.attrib['value']

                            if param.attrib['key'] == 'voxelSize':
                                scene_part_data['voxelsize'] = float(param.attrib['value'])
                
                # If efilepath used, transformation has already been added to each individual scenepart.
                if not efile_flag:
                    # Translate filter for point clouds and wavefront objects.
                    if filter.attrib['type'] == 'translate':
                        scene_part_data['translation'] = filter.find('param').attrib['value'].split(';')

                    # Scale filter, is a parameter for each respective scene part datatype.
                    if filter.attrib['type'] == 'scale':
                        scene_part_data['scale'] = float(filter.find('param').attrib['value'])

                    # Rotate filter, is a parameter for each respective scene part datatype.
                    if filter.attrib['type'] == 'rotate':
                        scene_part_data['rotate'] = []
                        scene_part_data['rotate_method'] = filter.attrib['rotations'] if 'rotations' in filter.attrib else 'global'
                        for rotation in filter.find('param').findall('rot'):
                            scene_part_data['rotate'].append([rotation.attrib['axis'], rotation.attrib['angle_deg']])

            # If efilepath used, sceneparts have already been added to list.
            if not efile_flag:
                scene_parts.append(scene_part_data)
        
        # Create instance of visualizer class. Needed for creation of GUI.
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="PyHelios Simulation")
        #vis.register_key_callback(80, lambda *x: sim.pause)  # 80 = p
        #vis.register_key_callback(67, lambda *x: sim.resume)  # 67 = c

        # Iterate over sceneparts and create individual open3d geometries. Also add the geometries to window.
        # .obj-dict = {extension : '.obj', file_path : str, scale : float, geometry : (o3d_triangle_mesh)}
        # .xyz-dict = {extension : '.xyz', file_path : str, translation : [x, y ,z], voxelsize : float,
        # geometry : (o3d_point_cloud/VoxelGrid)}
        for scene_part in scene_parts:

            if scene_part['extension'] == '.obj':
                # Create open3d object.
                scene_part['geometry'] = o3d.io.read_triangle_mesh(scene_part['filepath'])

                # Change colour of first scene part. This should be the ground plane.
                if scene_parts.index(scene_part) == 0:
                    scene_part['geometry'].paint_uniform_color([0.866, 0.858, 0.792])

                # Apply rotation
                if 'rotate' in scene_part:
                    if scene_part['rotate_method'].lower() == 'local':
                        print('WARNING: Local rotation mode not supported yet! Applying global rotations.')
                    for axis, angle in scene_part['rotate']:
                        print('Rotating geometry: ', scene_part['filepath'])
                        print('Axis, Angle: ', axis, ",", angle)
                        if axis.lower() == 'x':
                            rot = [1, 0, 0]
                        elif axis.lower() == 'y':
                            rot = [0, 1, 0]
                        elif axis.lower() == 'z':
                            rot = [0, 0, 1]
                        else:
                            raise Exception(f"Unknown rotation axis: {axis.lower()}")
                        R = scene_part['geometry'].get_rotation_matrix_from_axis_angle(np.array(rot) * float(angle) / 180. * np.pi)
                        scene_part['geometry'].rotate(R, center=[0, 0, 0])

                # Apply scaling.
                if 'scale' in scene_part:
                    print('Scaling geometry: ', scene_part['filepath'])
                    scene_part['geometry'].scale(scene_part['scale'], [0, 0, 0])

                # Apply translation.
                if 'translation' in scene_part:
                    scene_part['translation'] = [float(i) for i in scene_part['translation']]
                    print('Translating geometry: ', scene_part['filepath'])
                    print('Translation: ', scene_part['translation'])
                    scene_part['geometry'].translate(np.array(scene_part['translation'], dtype=float), relative=True)


                # Add geometry to visualizer.
                scene_part['geometry'].compute_vertex_normals()
                vis.add_geometry(scene_part['geometry'])

            elif scene_part['extension'] == '.xyz':
                # Create o3d point cloud.
                scene_part['geometry'] = (o3d.io.read_point_cloud(scene_part['filepath']))

                if 'translation' in scene_part:
                    print('Translating geometry: ', scene_part['filepath'])
                    # Translate point cloud.
                    # Get points as np array
                    point_cloud = np.asarray(scene_part['geometry'].points)

                    # Apply translation.
                    point_cloud[:, 0] += float(scene_part['translation'][0])
                    point_cloud[:, 1] += float(scene_part['translation'][1])
                    point_cloud[:, 2] += float(scene_part['translation'][2])

                    # Update geometry points with new values.
                    scene_part['geometry'].points = o3d.utility.Vector3dVector(point_cloud)
                    
                '''
                if 'scale' in scene_part:
                    print('Scaling geometry: ', scene_part['filepath'])
                    
                    # Scale point cloud.
                    # Get points as np array
                    point_cloud = np.asarray(scene_part['geometry'].points)
                    point_cloud = point_cloud*scene_part['scale']
                    
                    # Update geometry points with new values.
                    scene_part['geometry'].points = o3d.utility.Vector3dVector(point_cloud)
                '''
                
                # Set random colours.
                scene_part['geometry'].colors = o3d.utility.Vector3dVector(
                    np.random.uniform(0, 1, size=(point_cloud.shape[0], 3)))

                # Voxelise.
                scene_part['geometry'] = o3d.geometry.VoxelGrid.create_from_point_cloud(
                    scene_part['geometry'], voxel_size=scene_part['voxelsize'])

                # Add voxels to visualizer.
                vis.add_geometry(scene_part['geometry'])

            elif scene_part['extension'] == '.tif':
                # Imports.
                from osgeo import gdal

                file = scene_part['filepath']
                #scale = scene_part['scale']

                ds = gdal.Open(file)

                numBands = ds.RasterCount

                width = ds.RasterXSize
                height = ds.RasterYSize
                gt = ds.GetGeoTransform()
                band = ds.GetRasterBand(1)

                point_count = width * height

                ulx, xres, xskew, uly, yskew, yres = ds.GetGeoTransform()
                lrx = ulx + (ds.RasterXSize * xres)
                lry = uly + (ds.RasterYSize * yres)
                data = band.ReadAsArray(0, 0, width, height).astype(np.float)
                xs = np.linspace(ulx, lrx, ds.RasterXSize)
                ys = np.linspace(uly, lry, ds.RasterYSize)
                xm, ym = np.meshgrid(xs, ys)
                points = np.vstack([xm.flatten(), ym.flatten(), data.flatten()]).T
                where_nodata = points[:, 2] == band.GetNoDataValue()
                where_data = np.logical_not(where_nodata)
                points[where_nodata, 2] = np.mean(points[where_data, 2], axis=0)

                # Create triangles based on indices of points in 3d array.
                tri1_1 = np.arange(0, points.shape[0] - width - 1)  # the last row has no triangles
                tri1_2 = tri1_1 + width  # up
                tri1_3 = tri1_1 + 1  # right
                tri1 = np.vstack([tri1_1, tri1_2, tri1_3]).T
                # second type of triangle
                tri2_2 = tri1_1 + width + 1 # up & right
                tri2 = np.vstack([tri1_2, tri2_2, tri1_3]).T

                # remove edge triangles
                to_del = np.arange(1, height-1) * width - 1

                # remove nodata triangles
                nodata_del = np.nonzero(data[:-1, :].flatten() == band.GetNoDataValue())[0]
                to_del = np.unique(np.concatenate([to_del, nodata_del, nodata_del-1,
                                                   nodata_del-width, nodata_del-width-1]))

                tri1 = np.delete(tri1, to_del, axis=0)
                tri2 = np.delete(tri2, to_del, axis=0)
                triangles = np.concatenate([tri1, tri2])

                scene_part['geometry'] = o3d.geometry.TriangleMesh()
                scene_part['geometry'].vertices = o3d.utility.Vector3dVector(points)
                scene_part['geometry'].triangles = o3d.utility.Vector3iVector(triangles)
                scene_part['geometry'].compute_vertex_normals()
                vis.add_geometry(scene_part['geometry'])

        # Create point cloud geometry for survey data and scanner trajectory.
        measurement = o3d.geometry.PointCloud()
        trajectory = o3d.geometry.PointCloud()

        # Add center point to visualization.
        center_point = np.array([[0, 0, 0]])
        center = o3d.geometry.PointCloud()
        center.points = o3d.utility.Vector3dVector(center_point)
        vis.add_geometry(center)

        # Add measurement point cloud to window.
        vis.add_geometry(measurement)
        vis.add_geometry(trajectory)
        # Refresh GUI.
        vis.poll_events()
        vis.update_renderer()

        # Start pyhelios simulation.
        sim.start()

        while sim.isRunning():
            if len(mpoints) > 0:
                # Convert x, y, z lists with points from pyhelios callback to arrays.
                a = np.array(mpoints)
                b = np.array(tpoints)

                '''line_indices = np.zeros([0, 2])
                for i in range(np.shape(b)[0]):
                    line_indices = np.vstack((line_indices, [i, i+1]))
                print(line_indices)'''

                # Update o3d point clouds with points from callback.
                measurement.points = o3d.utility.Vector3dVector(a[:, :-1])
                try:
                    trajectory.points = o3d.utility.Vector3dVector(b)
                    #trajectory.lines = o3d.utility.Vector3dVector(line_indices)
                except Exception as err:
                    print(err)

                trajectory.paint_uniform_color([1, 0.706, 0])

                colours = np.zeros((a.shape[0], 3))

                # Set unique colour for each object based on object id.
                obj_ids = a[:, -1]

                uid = np.unique(obj_ids)

                cm = plt.get_cmap('tab10')

                for oid in uid:
                    colours[obj_ids == oid, :] = cm(((oid+1)/15)%1)[:3]  # 15 colors in this colormap, then repeat

                measurement.colors = o3d.utility.Vector3dVector(colours)

                # Update geometries for visualizer.
                vis.update_geometry(measurement)
                vis.update_geometry(trajectory)

                # Refresh GUI.
                vis.poll_events()
                vis.update_renderer()

            time.sleep(0.1)

        # Save final output in variable.
        output = sim.join()

        # Keep window open after sim.
        vis.run()

    if args.plot_result:
        # PyHeliostools.
        from pyheliostools import outputToNumpy
        # Polyscope.
        import polyscope as ps

        print('Preparing data for polyscope plot...')

        # Create numpy Array with points from trajectory.
        measurement_points, trajectory_points = outputToNumpy(output)

        # Points to be plotted:
        # First three cols are x, y and z vals.
        t_points = trajectory_points[:, 0:3]
        m_points = measurement_points[:, :3]

        # Initialize polyscope.
        ps.init()

        # Set correct direction for visualization of point clouds.
        ps.set_up_dir("z_up")

        # Register both the trajectory and measurement point clouds seperately.
        measurement_cloud = ps.register_point_cloud("Measurements", m_points)
        trajectory_cloud = ps.register_point_cloud("Scanner Trajectory", t_points)

        # Set more visually appealing point radiuses.
        measurement_cloud.set_radius(0.00091, relative=True)
        trajectory_cloud.set_radius(0.00191, relative=True)

        ps.show()
