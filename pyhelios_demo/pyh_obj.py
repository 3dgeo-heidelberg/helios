import open3d as o3d
import numpy as np
import time
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import os


class Scene:
    def __init__(self, survey_file):
        """

        Args:
            survey_file:
        """
        self.survey = survey_file
        self.name = ET.parse(survey_file).find('survey').attrib['scene'].split('#')[1]
        self.parts = []
        self.dir = ET.parse(survey_file).find('survey').attrib['scene'].split('#')[0]
        self.visualizer = o3d.visualization.VisualizerWithKeyCallback()
        self.xml_root = ET.parse(self.dir).getroot()

    def gen_from_xml(self):
        """Parse the scenefile and generate scenepart objects. Then parse and apply transformation.
        """
        for scene in self.xml_root.findall('scene'):
            if scene.attrib['id'] == self.name:
                for scene_part in scene.findall('part'):
                    loader = scene_part.find('filter').attrib['type']
                    if loader == 'geotiffloader':
                        part = TiffScenepart(scene_part)

                    elif loader == 'objloader':
                        part = ObjScenepart(scene_part)

                    elif loader == 'xyzloader':
                        part = VoxelScenepart(scene_part)

                    part.gen_from_xml()
                    part.parse_tranformation()
                    part.apply_tf()
                    self.parts.append(part)

    def visualize(self):
        """Create open3d window and add the sceneparts to visualizer."""
        # Create o3d window.
        self.visualizer.create_window(window_name="PyHelios Simulation, Scene: {}".format(self.name))

        for part in self.parts:

            # Change colour of ground plane scenepart.
            if os.path.basename(part.path).split('.')[0] == 'groundplane':
                part.o3dGeometry.paint_uniform_color([0.866, 0.858, 0.792])

            # Add geometry to visualizer.
            self.visualizer.add_geometry(part.o3dGeometry)

    def print_scene(self):
        """Print formatted string with information on sceneparts in scene."""
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


class Scenepart:
    def __init__(self, xml_loc):
        self.o3dGeometry = None
        self.translation = Translation()  # Translation class.
        self.scale = Scale()  # Scale class.
        self.rotate = Rotation()  # Rotation class.
        self.type = None
        self.path = xml_loc.find('filter').find('param').attrib['value']
        self.xml_loc = xml_loc

    def apply_tf(self):
        # Apply rotation
        if self.rotate.rotate:
            if self.rotate.method.lower() == 'local':
                print('WARNING: Local rotation mode not supported yet! Applying global rotations.')
            for axis, angle in self.rotate.rotate:
                print('Rotating geometry: ', self.path)
                print('Axis, Angle: ', axis, ",", angle)
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
            print('Scaling geometry: ', self.path)
            self.o3dGeometry.scale(self.scale.scale, [0, 0, 0])

        # Apply translation.
        self.translation.translation = [float(i) for i in self.translation.translation]
        print('Translating geometry: ', self.path)
        print('Translation: ', self.translation.translation)
        self.o3dGeometry.translate(np.array(self.translation.translation, dtype=float), relative=True)

    def parse_tranformation(self):
        # Get transformation.
        for filter in self.xml_loc.findall('filter'):

            # Translate filter for point clouds and wavefront objects.
            if filter.attrib['type'] == 'translate':
                self.translation.gen_from_xml(filter)

            # Scale filter, is a parameter for each respective scene part datatype.
            if filter.attrib['type'] == 'scale':
                self.scale.gen_from_xml(filter)

            # Rotate filter, is a parameter for each respective scene part datatype.
            if filter.attrib['type'] == 'rotate':
                self.rotate.gen_from_xml(filter)

            # Voxelsize for voxel sceneparts.
            if filter.attrib['type'] == 'xyzloader':
                for param in filter.findall('param'):
                    if param.attrib['key'] == 'voxelSize':
                        self.voxel_size = float(param.attrib['value'])
            # TODO: Is this the correct implementation? Could also overwrite method in voxel scenepart def.


class PointCloudScenepart(Scenepart):
    def __init__(self, xml_loc):
        super().__init__(xml_loc)
        self.type = 'pt_cloud'
        # TODO: What exactly is this? Not found in wiki and was not implemented in previous script.

    def gen_from_xml(self):
        pass


class VoxelScenepart(Scenepart):
    def __init__(self, xml_loc):
        super().__init__(xml_loc)
        self.type = 'voxel'
        self.voxel_size = 1.0

    def gen_from_xml(self):
        print('Loading Voxel Scenepart...')
        # Create o3d point cloud.
        self.o3dGeometry = (o3d.io.read_point_cloud(self.path))

        # Ptcloud to np for colour assignment.
        point_cloud = np.asarray(self.o3dGeometry.points)

        # Set random colours.
        self.o3dGeometry.colors = o3d.utility.Vector3dVector(
            np.random.uniform(0, 1, size=(point_cloud.shape[0], 3)))

        # Voxelise.
        self.o3dGeometry = o3d.geometry.VoxelGrid.create_from_point_cloud(
            self.o3dGeometry, voxel_size=self.voxel_size)


class ObjScenepart(Scenepart):
    def __init__(self, xml_loc):
        super().__init__(xml_loc)
        self.type = 'obj'

    def gen_from_xml(self):
        print('Loading .obj Scenepart...')
        # Create open3d object.
        self.o3dGeometry = o3d.io.read_triangle_mesh(self.path)
        self.o3dGeometry.compute_vertex_normals()

class TiffScenepart(Scenepart):
    def __init__(self, xml_loc):
        super().__init__(xml_loc)
        self.type = 'tiff'

    def gen_from_xml(self):
        print('Loading TIF Scenepart...')
        from osgeo import gdal

        file = self.path

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

        # Point data as array. Array formatting. Handling of nodata vals.
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

        # Create open3d geom.
        self.o3dGeometry = o3d.geometry.TriangleMesh()
        self.o3dGeometry.vertices = o3d.utility.Vector3dVector(points)
        self.o3dGeometry.triangles = o3d.utility.Vector3iVector(triangles)
        self.o3dGeometry.compute_vertex_normals()


class Translation:
    def __init__(self):
        self.translation = [0, 0, 0]

    def gen_from_xml(self, filter):
        self.translation = filter.find('param').attrib['value'].split(';')


class Rotation:
    def __init__(self):
        self.rotate = []
        self.method = ''

    def gen_from_xml(self, filter):
        self.method = filter.attrib['rotations'] if 'rotations' in filter.attrib else 'global'
        for rotation in filter.find('param').findall('rot'):
            self.rotate.append([rotation.attrib['axis'], rotation.attrib['angle_deg']])


class Scale:
    def __init__(self):
        self.scale = 1.0

    def gen_from_xml(self, filter):
        self.scale = float(filter.find('param').attrib['value'])


#voxel_scene = r'C:\Users\Mark\Documents\helios_main\data\surveys\voxels\tls_sphere_xyzloader.xml'
#toyblocks_scene = r'C:\Users\Mark\Documents\helios_main\data\surveys\toyblocks\als_toyblocks.xml'
'''
toyblocks_scene = Scene(toyblocks_scene)
toyblocks_scene.gen_from_xml()
toyblocks_scene.print_scene()
toyblocks_scene.visualize()

'''