import open3d as o3d
import numpy as np
import time
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import os


class Scene:
    """
    Class that represents the scene of a pyhelios simulation.
    ...
    Attributes
    ----------
    survey : str
        relative or absolute survey file as provided on init
    name : str
        name of scene as parsed from survey file
    parts : list
        list that contains each scene part as individual objects
    dir : str
        directory of xml scene file
    visualizer : o3d visualization
        open3d visualization that is used for visualization of scene
    xml_root : xml.etree root
        root of parsed xml scene file
    logging : bool
        flag to enable or disable logging
    trajectory : o3d ptcloud geometry
        open3d geometry that contains the survey trajectory points
    measurement : o3d ptcloud geometry
        open3d geometry that contains the survey measurement points
    Methods
    -------
    gen_from_xml():
        Parses the scene file and generates scenepart objects for the scene referenced from survey file.
        Then parses and applies transformation to scene parts and adds them to parts list.
    visualize():
        Creates open3d window and adds sceneparts + measurement/trajectory point clouds to visualizer.
    print_scene():
        Prints information on sceneparts.
    colourise(measurement_array):
        Adds colours to open3d geometries of survey.
    """
    def __init__(self, survey_file, logging_flag=False):
        """
        Init constructs all necessary attributes from survey file.
        """
        self.survey = survey_file
        self.name = ET.parse(survey_file).find('survey').attrib['scene'].split('#')[1]
        self.parts = []
        self.dir = ET.parse(survey_file).find('survey').attrib['scene'].split('#')[0]
        self.visualizer = o3d.visualization.VisualizerWithKeyCallback()
        self.xml_root = ET.parse(self.dir).getroot()
        self.logging = logging_flag
        self.trajectory = o3d.geometry.PointCloud()
        self.measurement = o3d.geometry.PointCloud()

    def gen_from_xml(self):
        """Parses the scene file and generates scenepart objects for the scene referenced from survey file.
        Then parses and applies transformation to scene parts and adds them to parts attribute."""

        if self.logging:
            print("\nBuilding scene from xml file...")

        for scene in self.xml_root.findall('scene'):
            if scene.attrib['id'] == self.name:
                for scene_part in scene.findall('part'):
                    loader = scene_part.find('filter').attrib['type']
                    if loader == 'geotiffloader':
                        part = TiffScenepart(scene_part, self.logging)

                    elif loader == 'objloader':
                        part = ObjScenepart(scene_part, self.logging)

                    elif loader == 'xyzloader':
                        part = VoxelScenepart(scene_part, self.logging)

                    elif loader == 'detailedvoxels':
                        part = PointCloudScenepart(scene_part, self.logging)

                    part.gen_from_xml()
                    part.parse_tranformation()
                    part.apply_tf()
                    self.parts.append(part)

        if self.logging:
            print("\n------------------------------------------------------------------------")
            print("Scene fully loaded! Information below:")
            print("------------------------------------------------------------------------")

    def visualize(self):
        """Creates open3d window and adds sceneparts + measurement/trajectory point clouds to visualizer."""
        # Create o3d window.
        if self.logging:
            print("Creating open3d visualisation...")

        self.visualizer.create_window(window_name="PyHelios Simulation, Scene: {}".format(self.name))

        # Add center point to visualization.
        center_point = np.array([[0, 0, 0]])
        center = o3d.geometry.PointCloud()
        center.points = o3d.utility.Vector3dVector(center_point)
        self.visualizer.add_geometry(center)

        # Add measurement and trajectory geoms to visualizer.
        self.visualizer.add_geometry(self.measurement)
        self.visualizer.add_geometry(self.trajectory)
        self.visualizer.get_render_option().mesh_show_back_face = True

        if self.logging:
            print("Adding sceneparts to visualisation...\n")

        for part in self.parts:

            # Change colour of ground plane scenepart.
            if os.path.basename(part.path).split('.')[0] == 'groundplane':
                part.o3dGeometry.paint_uniform_color([0.866, 0.858, 0.792])

            # Add geometry to visualizer.
            self.visualizer.add_geometry(part.o3dGeometry)

        # Refresh GUI.
        self.visualizer.poll_events()
        self.visualizer.update_renderer()

    def print_scene(self):
        """Prints formatted string with information on scene parts in scene."""
        print('\nScene generated from survey: ' + self.survey)
        print('Scene id: ' + self.name)
        print('Located at: ' + self.dir)
        print('\n----------------------------------------------------')
        print('Consists of sceneparts:')
        i = 0
        for scene_part in self.parts:
            i = i+1
            print('----------------------------------------------------')
            print('Scene part number {}:'.format(i))
            print('Path: ' + scene_part.path)
            print('Type: ' + scene_part.type)
            if scene_part.type == 'voxel':
                print('Voxel size - {}'.format(scene_part.voxel_size))
            print('Transformation: rot - {}, {}; scale - {}; translate - {}.'.format(scene_part.rotate.rotate,
                                                                                     scene_part.rotate.method,
                                                                                     scene_part.scale.scale,
                                                                                     scene_part.translation.translation))
        print("\n")

    def colourise(self, measurement_array):
        """
        Adds colours to open3d geometries of survey.
        Parameters
        ----------
        measurement_array : np.array
        Array with survey measurements that is used to extract shape and size of measurements.
        """

        # Paint trajectory points with uniform colour.
        self.trajectory.paint_uniform_color([1, 0.706, 0])

        # Set unique colour for each object in measurements based on object id.
        # Get shape of measurement array.
        colours = np.zeros((measurement_array.shape[0], 3))

        # Get unique array with object ids and unique object ids.
        obj_ids = measurement_array[:, -1]
        uid = np.unique(obj_ids)

        # plt colormap.
        cm = plt.get_cmap('tab10')

        # Generate colours.
        for oid in uid:
            colours[obj_ids == oid, :] = cm(((oid + 1) / 15) % 1)[:3]  # 15 colors in this colormap, then repeat

        # Apply colours to measurement geometry.
        self.measurement.colors = o3d.utility.Vector3dVector(colours)




class Scenepart:
    """
    Class used to represent individual scene parts of different data types.
    ...
    Attributes
    ----------
    o3dGeometry : None
        open3d geometry of scene part object
    translation : object of Translation() class
        translation object contains the transformation to be applied to scene part
    scale : object of Scale() class
        scale object contains the scale value to be applied to scene part
    rotate : object of Rotation() class
        rotate object contains the rotation to be applied to scene part
    type : str
        string with information on datatype of scene part
    path : str
        path of scene part source file
    xml_loc : xml.etree location
        the location of the scene part within the parsed xml scene file
    logging : bool
        flag to enable or disable logging
    Methods
    -------
    gen_from_xml():
        Generates open3d objects from source files.
    parse_transformation():
        Extracts general transformation from scene part xml and sets the respective attributes of the object.
        (self.translation, self.rotate, self.scale)
    apply_tf():
        Applies the general transformation (translate, rotate, scale) to the open3d geometry.
    """

    def __init__(self, xml_loc, logging_flag):
        """
        Uses location of scene file in xml file to get path of scene part. Generates all other necessary attributes.
        """
        self.o3dGeometry = None
        self.translation = Translation()  # Translation class.
        self.scale = Scale()  # Scale class.
        self.rotate = Rotation()  # Rotation class.
        self.type = None
        self.path = xml_loc.find('filter').find('param').attrib['value']
        self.xml_loc = xml_loc
        self.logging = logging_flag
        self.up = None
        for param in xml_loc.find('filter').findall('param'):
            if param.attrib['key'] == 'up':
                self.up = param.attrib['value']
                        
    def apply_tf(self):
        """
        Apply transformation from attributes rotate, scale, translation to open3d geometry.
        """
        if self.logging:
            print("Applying transformation!")
            
        if self.up == 'y' or self.up == 'Y':
            print("Y-axis set to 'up'.")
            rot = [1, 0, 0]
            R = self.o3dGeometry.get_rotation_matrix_from_axis_angle(
                    np.array(rot) * float(90) / 180. * np.pi)
            self.o3dGeometry.rotate(R, center=[0, 0, 0])
            
        if self.up == 'x' or self.up == 'X':
            print("X-axis set to 'up'.")
            rot = [0, 1, 0]
            R = self.o3dGeometry.get_rotation_matrix_from_axis_angle(
                    np.array(rot) * float(90) / 180. * np.pi)
            self.o3dGeometry.rotate(R, center=[0, 0, 0])
            
        # Apply rotation
        if self.rotate.rotate:
            if self.rotate.method.lower() == 'local':
                print('WARNING: Local rotation mode not supported yet! Applying global rotations.')
            for axis, angle in self.rotate.rotate:
                if self.logging:
                    print('Rotating geometry...')
                if axis.lower() == 'x':
                    rot = [1, 0, 0]
                elif axis.lower() == 'y':
                    rot = [0, 1, 0]
                elif axis.lower() == 'z':
                    rot = [0, 0, 1]
                else:
                    raise Exception(f"Unknown rotation axis: {axis.lower()}")
                R = self.o3dGeometry.get_rotation_matrix_from_axis_angle(
                    np.array(rot) * float(angle) / 180. * np.pi)
                self.o3dGeometry.rotate(R, center=[0, 0, 0])

        # Apply scaling.
        if self.scale.scale != 1.0:
            if self.logging:
                print('Scaling geometry...')
            self.o3dGeometry.scale(self.scale.scale, [0, 0, 0])

        # Apply translation.
        self.translation.translation = [float(i) for i in self.translation.translation]
        if not all([item == 0 for item in self.translation.translation]):
            if self.logging:
                print('Translating geometry...')
            self.o3dGeometry.translate(np.array(self.translation.translation, dtype=float), relative=True)

    def parse_tranformation(self):
        """
        Parse xml of scene and call method gen_from_xml to generate individual parts of general transformation.
        (self.translation, self.rotate, self.scale)
        """

        if self.logging:
            print("Loading scenepart transformation... (translate, scale, rotate)")

        # Get transformation.
        for filter in self.xml_loc.findall('filter'):

            # Translate filter for point clouds and wavefront objects.
            if filter.attrib['type'] == 'translate':
                self.translation.gen_from_xml(filter)
                if self.logging:
                    print("Found translation.")
                    print("Translation: {}".format(self.translation.translation))

            # Scale filter, is a parameter for each respective scene part datatype.
            if filter.attrib['type'] == 'scale':
                self.scale.gen_from_xml(filter)
                if self.logging and self.scale.scale != 1.0:
                    print("Found scale value.")
                    print("Scale value: {}".format(self.scale.scale))

            # Rotate filter, is a parameter for each respective scene part datatype.
            if filter.attrib['type'] == 'rotate':
                self.rotate.gen_from_xml(filter)
                if self.logging:
                    print("Found rotation.")
                    print("Rotation: {} - Method: {}".format(self.rotate.rotate, self.rotate.method))


class PointCloudScenepart(Scenepart):
    """
    Detailed voxel scene part generated from .vox file.
    For more information see parent class (Scenepart).
    """
    def __init__(self, xml_loc, logging_flag):
        super().__init__(xml_loc, logging_flag)
        self.type = 'pt_cloud'
        self.points = o3d.geometry.PointCloud()
        self.voxel_size = 0.05
        self.mode = 'transmittive'

        # Iterate through xml parameters.
        for param in xml_loc.find('filter').findall('param'):
            # Path needs to be extracted seperately, as path is not always first param in xml.
            if param.attrib['key'] == 'filepath':
                self.path = param.attrib['value']

            # Extract intersection mode, to be used in voxel building.
            if param.attrib['key'] == 'intersectionMode':
                self.mode = param.attrib['value']

        if self.logging:
            print("------------------------------------------------------------------------")
            print("Detailed voxel scenepart:")
            print("Source file: {}".format(self.path))

    def gen_from_xml(self):
        """
        Creates open3d geometry for detailed voxel scene part from scenepart path.
        """

        if self.logging:
            print("Loading detailed voxel scenepart!")
            print("Mode: {}".format(self.mode))

        # Get header of .vox file, including important positional data.
        header = []
        with open(self.path, "r") as src:
            for i in range(6):
                header.append(src.readline().rstrip("\n"))
        header.pop(5)
        header[0] = "Format: [1] - min_corner; " \
                    "[2] - max_corner; " \
                    "[3] - split; " \
                    "[4] - res val"

        for i in range(1, 4):
            header[i] = [float(i) for i in header[i].split(': ')[1].split(' ')]

        # Parsing res no longer necessary for current implementation.
        '''try:
            header[4] = float(header[4].split('#res: ')[1].split(' ')[0])
        except Exception:
            header[4] = float(header[4].split('#res:')[1].split(' ')[0])'''

        # Get vertices of voxels.
        vertices = np.loadtxt(self.path, usecols=range(0, 3), skiprows=6, dtype=np.float64)

        # Apply transformation to vertices using header data. (min corner, max corner, split)
        for i in vertices:
            for j in range(3):
                i[j] = i[j] / header[3][j] * (header[2][j] - header[1][j]) + header[1][j]

        # In case of transmittive or fixed apply fixed mode, using only voxels with transmittance < 1.
        if self.mode == 'transmittive' or self.mode == 'fixed':
            transmittance = np.loadtxt(self.path, usecols=13, skiprows=6, dtype=np.float64)
            vertices = np.delete(vertices, np.where(transmittance >= 1), axis=0)

        # In case of scaled mode apply rudimentary scaled mode, using only voxels with pad (PADVBTotal) != 0.
        if self.mode == 'scaled':
            padvb = np.loadtxt(self.path, usecols=3, skiprows=6, dtype=np.float64)
            vertices = np.delete(vertices, np.where(padvb == 0), axis=0)

        # Vertices to o3d point cloud and o3d point cloud to o3d voxelgrid.
        self.points.points = o3d.utility.Vector3dVector(vertices)
        self.o3dGeometry = o3d.geometry.VoxelGrid.create_from_point_cloud(self.points, voxel_size=self.voxel_size)

    def apply_tf(self):
        """
        Apply transformation from attributes rotate, scale, translation to open3d geometry.
        Overloaded to translate first the voxel point cloud and then set new points to final voxelgrid geometry.
        """
        if self.logging:
            print("Applying transformation!")
        # Apply rotation
        if self.rotate.rotate:
            if self.rotate.method.lower() == 'local':
                print('WARNING: Local rotation mode not supported yet! Applying global rotations.')
            for axis, angle in self.rotate.rotate:
                if self.logging:
                    print('Rotating geometry...')
                if axis.lower() == 'x':
                    rot = [1, 0, 0]
                elif axis.lower() == 'y':
                    rot = [0, 1, 0]
                elif axis.lower() == 'z':
                    rot = [0, 0, 1]
                else:
                    raise Exception(f"Unknown rotation axis: {axis.lower()}")
                R = self.o3dGeometry.get_rotation_matrix_from_axis_angle(
                    np.array(rot) * float(angle) / 180. * np.pi)
                self.o3dGeometry.rotate(R, center=[0, 0, 0])

        # Apply scaling.
        if self.scale.scale != 1.0:
            if self.logging:
                print('Scaling geometry...')
            self.o3dGeometry.scale(self.scale.scale, [0, 0, 0])

        # Apply translation.
        self.translation.translation = [float(i) for i in self.translation.translation]
        if sum(self.translation.translation) != 0:
            if self.logging:
                print('Translating geometry...')
            self.points.translate(np.array(self.translation.translation, dtype=float), relative=True)
            self.o3dGeometry = o3d.geometry.VoxelGrid.create_from_point_cloud(self.points, voxel_size=self.voxel_size)


class VoxelScenepart(Scenepart):
    """
    Voxel scene part generated from point cloud file. Voxelsize determined in scene xml.
    For more information see parent class (Scenepart).
    """
    def __init__(self, xml_loc, logging_flag):
        super().__init__(xml_loc, logging_flag)
        self.type = 'voxel'
        self.voxel_size = 1.0
        self.points = o3d.geometry.PointCloud()

        if self.logging:
            print("------------------------------------------------------------------------")
            print("Voxel scenepart:")
            print("Source file: {}".format(self.path))

    def parse_tranformation(self):
        """
        Overloaded function to parse transformation for voxel scene parts. Runs super and then extracts further
        attribute voxel size.
        """
        super().parse_tranformation()
        # Get transformation.
        for filter in self.xml_loc.findall('filter'):
            # Voxelsize for voxel scene parts.
            if filter.attrib['type'] == 'xyzloader':
                for param in filter.findall('param'):
                    if param.attrib['key'] == 'voxelSize':
                        self.voxel_size = float(param.attrib['value'])
                        if self.logging:
                            print("Found voxelsize value.")
                            print("Voxelsize: {}".format(self.voxel_size))


    def gen_from_xml(self):
        """
        Creates open3d geometry for voxel scene part from scenepart path.
        """
        if self.logging:
            print('Loading Voxel Scenepart...')

        # Create o3d point cloud.
        self.points = (o3d.io.read_point_cloud(self.path))

        # Ptcloud to np for colour assignment.
        point_cloud = np.asarray(self.points.points)

        # Set random colours.
        self.points.colors = o3d.utility.Vector3dVector(
            np.random.uniform(0, 1, size=(point_cloud.shape[0], 3)))

        if self.logging:
            print("Voxelising...")

        # Voxelise.
        self.o3dGeometry = o3d.geometry.VoxelGrid.create_from_point_cloud(
            self.points, voxel_size=self.voxel_size)

        if self.logging:
            print("Voxel scenepart successfully loaded.")

    def apply_tf(self):
        """
        Apply transformation from attributes rotate, scale, translation to open3d geometry.
        Overloaded to translate first the voxel point cloud and then set new points to final voxelgrid geometry.
        """
        if self.logging:
            print("Applying transformation!")
        # Apply rotation
        if self.rotate.rotate:
            if self.rotate.method.lower() == 'local':
                print('WARNING: Local rotation mode not supported yet! Applying global rotations.')
            for axis, angle in self.rotate.rotate:
                if self.logging:
                    print('Rotating geometry...')
                if axis.lower() == 'x':
                    rot = [1, 0, 0]
                elif axis.lower() == 'y':
                    rot = [0, 1, 0]
                elif axis.lower() == 'z':
                    rot = [0, 0, 1]
                else:
                    raise Exception(f"Unknown rotation axis: {axis.lower()}")
                R = self.o3dGeometry.get_rotation_matrix_from_axis_angle(
                    np.array(rot) * float(angle) / 180. * np.pi)
                self.o3dGeometry.rotate(R, center=[0, 0, 0])

        # Apply scaling.
        if self.scale.scale != 1.0:
            if self.logging:
                print('Scaling geometry...')
            self.o3dGeometry.scale(self.scale.scale, [0, 0, 0])

        # Apply translation.
        self.translation.translation = [float(i) for i in self.translation.translation]
        if sum(self.translation.translation) != 0:
            if self.logging:
                print('Translating geometry...')
            self.points.translate(np.array(self.translation.translation, dtype=float), relative=True)
            self.o3dGeometry = o3d.geometry.VoxelGrid.create_from_point_cloud(self.points, voxel_size=self.voxel_size)


class ObjScenepart(Scenepart):
    """
    3d object scenepart generated from .obj file.
    For more information see parent class (Scenepart).
    """
    def __init__(self, xml_loc, logging_flag):
        super().__init__(xml_loc, logging_flag)
        self.type = 'obj'

        if self.logging:
            print("------------------------------------------------------------------------")
            print(".obj scenepart:")
            print("Source file: {}".format(self.path))

    def gen_from_xml(self):
        """
        Creates open3d geometry for .obj scene part from scenepart path.
        """
        from pyhelios.util import read_obj
        
        if self.logging:
            print('Loading .obj Scenepart...')

        # Create open3d object.
        self.o3dGeometry = read_obj.read_obj(self.path, logging=self.logging)

        if self.logging:
            print(".obj scenepart successfully loaded.")


class TiffScenepart(Scenepart):
    """
    Tiff scenepart generated from .tiff file.
    For more information see parent class (Scenepart).
    """
    def __init__(self, xml_loc, logging_flag):
        super().__init__(xml_loc, logging_flag)
        self.type = 'tiff'

        if self.logging:
            print("------------------------------------------------------------------------")
            print("tiff scenepart:")
            print("Source file: {}".format(self.path))

    def gen_from_xml(self):
        """
        Creates open3d geometry for tiff scene part from scenepart path. Raster arithmetics needed to generate correct
        values in visualizer coords and triangles.
        """
        if self.logging:
            print('Loading TIF Scenepart...')
        from osgeo import gdal

        file = self.path

        if self.logging:
            print("Opening file and getting geotransformation...")

        ds = gdal.Open(file)

        # Get raster information and band itself.
        width = ds.RasterXSize
        height = ds.RasterYSize
        # gt = ds.GetGeoTransform()
        band = ds.GetRasterBand(1)

        # Get transformation.
        ulx, xres, xskew, uly, yskew, yres = ds.GetGeoTransform()

        # Get lower right coordinates from upper left coordinates.
        lrx = ulx + (ds.RasterXSize * xres)
        lry = uly + (ds.RasterYSize * yres)

        if self.logging:
            print("Applying geotransformation...")

        # Point data as array. Array formatting. Handling of nodata vals.
        data = band.ReadAsArray(0, 0, width, height).astype(np.float)
        xs = np.linspace(ulx, lrx, ds.RasterXSize)
        ys = np.linspace(uly, lry, ds.RasterYSize)
        xm, ym = np.meshgrid(xs, ys)
        points = np.vstack([xm.flatten(), ym.flatten(), data.flatten()]).T
        where_nodata = points[:, 2] == band.GetNoDataValue()
        where_data = np.logical_not(where_nodata)
        points[where_nodata, 2] = np.mean(points[where_data, 2], axis=0)

        if self.logging:
            print("Calculating triangles...")

        # Create triangles based on indices of points in 3d array.
        tri1_1 = np.arange(0, points.shape[0] - width - 1)  # the last row has no triangles
        tri1_2 = tri1_1 + width  # up
        tri1_3 = tri1_1 + 1  # right
        tri1 = np.vstack([tri1_1, tri1_2, tri1_3]).T

        # second type of triangle
        tri2_2 = tri1_1 + width + 1  # up & right
        tri2 = np.vstack([tri1_2, tri2_2, tri1_3]).T

        # remove edge triangles
        to_del = np.arange(1, height - 1) * width - 1

        # remove nodata triangles
        nodata_del = np.nonzero(data[:-1, :].flatten() == band.GetNoDataValue())[0]
        to_del = np.unique(np.concatenate([to_del, nodata_del, nodata_del - 1,
                                           nodata_del - width, nodata_del - width - 1]))

        tri1 = np.delete(tri1, to_del, axis=0)
        tri2 = np.delete(tri2, to_del, axis=0)
        triangles = np.concatenate([tri1, tri2])

        if self.logging:
            print("Creating open3d geometries...")

        # Create open3d geom.
        self.o3dGeometry = o3d.geometry.TriangleMesh()
        self.o3dGeometry.vertices = o3d.utility.Vector3dVector(points)
        self.o3dGeometry.triangles = o3d.utility.Vector3iVector(triangles)
        self.o3dGeometry.compute_vertex_normals()

        if self.logging:
            print("tiff scenepart successfully loaded.")


class Translation:
    """
    Class that represents the translation of a scenepart.
    """
    def __init__(self):
        """
        Attribute translation is list with 3 elements [x, y, z].
        """
        self.translation = [0, 0, 0]

    def gen_from_xml(self, filter):
        """
        Parses xml and extracts translation to attribute.
        Args:
            filter: xml location of translation.
        """
        params = filter.findall('param')
        for param in params:
            if param.attrib['type'] == 'vec3' and param.attrib['key'] == 'offset':
                self.translation = param.attrib['value'].split(';')


class Rotation:
    """
    Class that represents the rotation of a scenepart.
    """
    def __init__(self):
        """
        Attributes are rotate [[axis, method], ...] and rotate method (str).
        """
        self.rotate = []
        self.method = ''

    def gen_from_xml(self, filter):
        """
        Parses xml and extracts rotation to attributes.
        Args:
            filter: xml location of rotation.
        """
        self.method = filter.attrib['rotations'] if 'rotations' in filter.attrib else 'global'
        for rotation in filter.find('param').findall('rot'):
            self.rotate.append([rotation.attrib['axis'], rotation.attrib['angle_deg']])


class Scale:
    """
    Class that represents the scaling of a scenepart.
    """
    def __init__(self):
        """
        Attribute is scale factor (float).
        """
        self.scale = 1.0

    def gen_from_xml(self, filter):
        """
        Parses xml and extracts scale factor to attribute.
        Args:
             filter: xml location of scale.
        """
        self.scale = float(filter.find('param').attrib['value'])