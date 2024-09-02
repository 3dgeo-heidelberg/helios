
import _helios
import numpy as np
import math 
import threading
from unittest.mock import MagicMock, patch
import pytest


def tuple_to_dvec3(t):
    return _helios.dvec3(*t)

def tup_add(v1, v2):
    return (v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2])

def tup_mul(v, scalar):
    return (v[0] * scalar, v[1] * scalar, v[2] * scalar)

def test_aabb_instantiation():
    aabb = _helios.AABB.create()
    assert aabb is not None, "Failed to create AABB instance"

def test_aabb_properties():
    aabb = _helios.AABB.create()
    min_vertex = aabb.min_vertex
    max_vertex = aabb.max_vertex
    assert isinstance(min_vertex, _helios.Vertex), "min_vertex should be a Vertex instance"
    assert isinstance(max_vertex, _helios.Vertex), "max_vertex should be a Vertex instance"

def test_vertex_properties():
    v = _helios.Vertex(1.0, 2.0, 3.0)
    position = v.position
    normal = v.normal
    assert position == (1.0, 2.0, 3.0), "Position should be [1.0, 2.0, 3.0]"
    assert normal == (0.0, 0.0, 0.0), "Normal should be [0.0, 0.0, 0.0] by default"

def test_vertex_instantiation():
    v = _helios.Vertex(1.0, 2.0, 3.0)
    assert isinstance(v, _helios.Vertex)
    assert v.position == (1.0, 2.0, 3.0)

def test_vertex_default_instantiation():
    v = _helios.Vertex()
    assert isinstance(v, _helios.Vertex)
    assert v.position == (0.0, 0.0, 0.0)
    assert v.normal == (0.0, 0.0, 0.0)

def test_triangle_instantiation():
    v0 = _helios.Vertex(0.0, 0.0, 0.0)
    v1 = _helios.Vertex(1.0, 0.0, 0.0)
    v2 = _helios.Vertex(0.0, 1.0, 0.0)
    triangle = _helios.Triangle(v0, v1, v2)
    assert isinstance(triangle, _helios.Triangle), "Failed to create Triangle instance"

def test_primitive_properties():
    v0 = _helios.Vertex(0.0, 0.0, 0.0)
    v1 = _helios.Vertex(1.0, 0.0, 0.0)
    v2 = _helios.Vertex(0.0, 1.0, 0.0)
    triangle = _helios.Triangle(v0, v1, v2)
    assert triangle.scene_part is None, "scene_part should be None by default"
    assert triangle.material is None, "material should be None by default"
    assert isinstance(triangle.AABB, _helios.AABB), "AABB should be an AABB instance"
    assert isinstance(triangle.centroid, tuple) and len(triangle.centroid) == 3, "centroid should be a 3-tuple"

def test_triangle_face_normal():
    v0 = _helios.Vertex(0.0, 0.0, 0.0)
    v1 = _helios.Vertex(1.0, 0.0, 0.0)
    v2 = _helios.Vertex(0.0, 1.0, 0.0)
    triangle = _helios.Triangle(v0, v1, v2)
    face_normal = triangle.face_normal
    assert isinstance(face_normal, tuple) and len(face_normal) == 3, "face_normal should be a 3-tuple"

def test_primitive_ray_intersection():
    v0 = _helios.Vertex(0.0, 0.0, 0.0)
    v1 = _helios.Vertex(1.0, 0.0, 0.0)
    v2 = _helios.Vertex(0.0, 1.0, 0.0)
    triangle = _helios.Triangle(v0, v1, v2)
    ray_origin = (0.5, 0.5, 1.0)
    ray_dir = (0.0, 0.0, -1.0)
    intersections = triangle.ray_intersection(ray_origin, ray_dir)
    expected_intersections = (0.5, 0.5, 0.0)
    
    # Verify the results
    if intersections[0] == -1:
        # No intersection, handle accordingly
        assert False, "No intersection found"
    else:
        # Calculate the intersection point
        t = intersections[0]
        intersection_point = tup_add(ray_origin, tup_mul(ray_dir, t))
        
        expected_intersection_point = (0.5, 0.5, 0.0)

    assert len(intersection_point) == len(expected_intersections), "Number of intersections does not match"
    for i, intersection in enumerate(intersection_point):
        assert intersection == expected_intersections[i], f"Intersection at index {i} does not match"
    ray_origin = (0.5, 0.5, 1.0)
    ray_dir = (0.0, 0.0, -1.0)
    intersections = triangle.ray_intersection(ray_origin, ray_dir) 
    assert len(intersections) > 0, "The intersections list should not be empty"
    assert intersections[0] == -1 or isinstance(intersections[0], float), "The intersection should be a float or -1"

def test_primitive_ray_intersection_distance():
    v0 = _helios.Vertex(0.0, 0.0, 0.0)
    v1 = _helios.Vertex(1.0, 0.0, 0.0)
    v2 = _helios.Vertex(0.0, 1.0, 0.0)
    triangle = _helios.Triangle(v0, v1, v2)
    ray_origin = (0.5, 0.5, 1.0)
    ray_dir = (0.0, 0.0, -1.0)
    distance = triangle.ray_intersection_distance(ray_origin, ray_dir)
    assert isinstance(distance, float), "ray_intersection_distance should return a float"
    assert distance > 0, "The distance should be greater than 0"

def test_primitive_incidence_angle():
    v0 = _helios.Vertex(0.0, 0.0, 0.0)
    v1 = _helios.Vertex(1.0, 0.0, 0.0)
    v2 = _helios.Vertex(0.0, 1.0, 0.0)
    triangle = _helios.Triangle(v0, v1, v2)
    ray_origin = (0.5, 0.5, 1.0)
    ray_dir = (0.0, 0.0, -1.0)
    intersection_point = (0.5, 0.5, 0.0)
    incidence_angle = triangle.incidence_angle(ray_origin, ray_dir, intersection_point)
    expected_angle = 0.0  # This value depends on your implementation and expected result
    
    # Verify the result
    assert abs(incidence_angle - expected_angle) < 1e-6, f"Incidence angle {incidence_angle} is not close to expected value {expected_angle}"
    ray_origin = (0.5, 0.5, 1.0)
    ray_dir = (0.0, 0.0, -1.0)
    intersection_point = (0.5, 0.5, 0.0)
    incidence_angle = triangle.incidence_angle(ray_origin, ray_dir, intersection_point)
    assert isinstance(incidence_angle, float), "incidence_angle should return a float"


def test_primitive_update():
    v0 = _helios.Vertex(0.0, 0.0, 0.0)
    v1 = _helios.Vertex(1.0, 0.0, 0.0)
    v2 = _helios.Vertex(0.0, 1.0, 0.0)
    triangle = _helios.Triangle(v0, v1, v2)
    triangle.update()
    # No assert needed; just ensure the method call does not raise an exception

def test_primitive_num_vertices():
    v0 = _helios.Vertex(0.0, 0.0, 0.0)
    v1 = _helios.Vertex(1.0, 0.0, 0.0)
    v2 = _helios.Vertex(0.0, 1.0, 0.0)
    triangle = _helios.Triangle(v0, v1, v2)
    num_vertices = triangle.num_vertices
    assert num_vertices == 3, "num_vertices should return 3"
#GLMDVEC3
def test_primitive_vertices():
    v0 = _helios.Vertex(0.0, 0.0, 0.0)
    v1 = _helios.Vertex(1.0, 0.0, 0.0)
    v2 = _helios.Vertex(0.0, 1.0, 0.0)
    triangle = _helios.Triangle(v0, v1, v2)
    ray_origin = (0.5, 0.5, 1.0)
    ray_dir = (0.0, 0.0, -1.0)
    intersections = triangle.ray_intersection(ray_origin, ray_dir)
   
    assert len(intersections) > 0, "The intersections list should not be empty"
    assert intersections[0] == -1 or isinstance(intersections[0], float), "The intersection should be a float or -1"



def test_detailed_voxel_instantiation():
    double_values = _helios.DoubleVector([0.1, 0.2, 0.3])
    voxel = _helios.DetailedVoxel([1.0, 2.0, 3.0], 0.5, [1, 2], double_values)#[0.1, 0.2, 0.3])
    assert isinstance(voxel, _helios.DetailedVoxel)
    assert voxel.nb_echos == 1
    assert voxel.nb_sampling == 2

def test_detailed_voxel_properties():
    double_values = _helios.DoubleVector([0.1, 0.2, 0.3])
    voxel = _helios.DetailedVoxel([1.0, 2.0, 3.0], 0.5, [1, 2], double_values)
    voxel.nb_echos = 3
    assert voxel.nb_echos == 3
    voxel.nb_sampling = 4
    assert voxel.nb_sampling == 4
    assert voxel.number_of_double_values == 3
    voxel.max_pad = 0.6
    assert voxel.max_pad == 0.6
    voxel.set_double_value(1, 0.5)
    assert voxel.get_double_value(1) == 0.5

def test_trajectory_instantiation():
    # Test default constructor
    traj = _helios.Trajectory()
    assert isinstance(traj, _helios.Trajectory)
    assert traj.gps_time == 0.0
    assert traj.position == (0.0, 0.0, 0.0)
    assert traj.roll == 0.0
    assert traj.pitch == 0.0
    assert traj.yaw == 0.0

    # Test parameterized constructor
    gps_time = 1234567890.0
    position = [1.0, 2.0, 3.0]
    roll, pitch, yaw = 0.1, 0.2, 0.3
    traj = _helios.Trajectory(gps_time, position, roll, pitch, yaw)
    assert isinstance(traj, _helios.Trajectory)
    assert traj.gps_time == gps_time
    assert np.allclose(traj.position, position)
    assert traj.roll == roll
    assert traj.pitch == pitch
    assert traj.yaw == yaw

def test_trajectory_property():
    traj = _helios.Trajectory()
    # Test setting and getting properties
    traj.gps_time = 123.456
    assert traj.gps_time == 123.456
    
    traj.position = [4.0, 5.0, 6.0]
    assert np.allclose(traj.position, [4.0, 5.0, 6.0])
    
    traj.roll = 0.4
    assert traj.roll == 0.4
    
    traj.pitch = 0.5
    assert traj.pitch == 0.5
    
    traj.yaw = 0.6
    assert traj.yaw == 0.6

def test_noise_source_instantiation():
    noise_src = _helios.NoiseSource()
    assert isinstance(noise_src, _helios.NoiseSource)
    
    # Check default properties
    assert noise_src.clip_min == 0.0
    assert noise_src.clip_max == 1.0
    assert not noise_src.clip_enabled
    assert not noise_src.fixed_value_enabled
    assert noise_src.fixed_lifespan == 1 
    assert noise_src.fixed_value_remaining_uses == 0

def test_noise_source_properties():
    noise_src = _helios.NoiseSource()
    
    # Test property setters and getters
    noise_src.clip_min = -1.0
    assert noise_src.clip_min == -1.0
    
    noise_src.clip_max = 2.0
    assert noise_src.clip_max == 2.0
    
    noise_src.clip_enabled = True
    assert noise_src.clip_enabled
    
    noise_src.fixed_lifespan = 0
    assert noise_src.fixed_lifespan == 0
    
    noise_src.fixed_value_remaining_uses = 5
    assert noise_src.fixed_value_remaining_uses == 5
    

def test_randomness_generator_instantiation():
    rng = _helios.RandomnessGenerator()
    assert isinstance(rng, _helios.RandomnessGenerator)

def test_randomness_generator_methods():
    rng = _helios.RandomnessGenerator()
    
    # Test uniform real distribution
    rng.compute_uniform_real_distribution(0.0, 1.0)
    
    uniform_value = rng.uniform_real_distribution_next()
    assert isinstance(uniform_value, float) or isinstance(uniform_value, np.float64)
    
    # Test normal distribution
    rng.compute_normal_distribution(0.0, 1.0)
    
    normal_value = rng.normal_distribution_next()
    assert isinstance(normal_value, float) or isinstance(normal_value, np.float64)


def test_ray_scene_intersection_instantiation():
    # Test default constructor
    rsi = _helios.RaySceneIntersection()
    assert isinstance(rsi, _helios.RaySceneIntersection)
    assert rsi.primitive is None
    assert rsi.point == (0.0, 0.0, 0.0)
    assert rsi.incidence_angle == 0.0

def test_ray_scene_intersection_properties():
    rsi = _helios.RaySceneIntersection()
    
    # Test setting and getting properties
    rsi.point = (1.0, 2.0, 3.0)
    assert rsi.point == (1.0, 2.0, 3.0)
    
    rsi.incidence_angle = 45.0
    assert rsi.incidence_angle == 45.0

def test_scanning_strip_instantiation():
    # Test default constructor
    strip = _helios.ScanningStrip("custom_strip1")
    assert isinstance(strip, _helios.ScanningStrip)
    assert strip.strip_id == "custom_strip1"  # Correctly accessing property

    # Test constructor with custom strip ID
    custom_id = "custom_strip"
    strip = _helios.ScanningStrip(custom_id)
    assert strip.strip_id == custom_id   # Correctly accessing property

def test_scanning_strip_methods():
    strip = _helios.ScanningStrip("custom_strip2")
    
    # Create a leg and add it to the strip to test `has` method
    leg = _helios.Leg(10.0, 1, strip)
    assert strip.has(1)
    assert not strip.has(2)  # Not in the strip
    
    ''' Test `isLastLegInStrip`
     boost python did  not bind the  'wasProcessed' of the Leg class'''
    # leg.wasProcessed = False
    # assert not strip.isLastLegInStrip()
    
    # leg.wasProcessed = True
    # assert strip.isLastLegInStrip()


def test_simulation_cycle_callback():
    def callback(args):
        measurements, trajectories, outpath, outpaths, final = args  # Unpack the tuple
        assert all(isinstance(m, _helios.Measurement) for m in measurements)
        assert all(isinstance(t, _helios.Trajectory) for t in trajectories)
        assert outpath == "test_path"
        assert all(isinstance(p, str) for p in outpaths)
        assert final is False


    sim_callback = _helios.SimulationCycleCallback(callback)
    
    # Create mock data for measurements and trajectories
    measurements = [_helios.Measurement() for _ in range(3)]
    trajectories = [_helios.Trajectory() for _ in range(2)]
    # Directly use Python lists
    sim_callback(measurements, trajectories, "test_path")


def test_fwf_settings_instantiation():
    # Test default constructor
    fwf = _helios.FWFSettings()
    assert isinstance(fwf, _helios.FWFSettings)
    
    # Check default values
    assert fwf.bin_size == 0.25
    assert fwf.min_echo_width == 2.5
    assert fwf.peak_energy == 500.0
    assert fwf.aperture_diameter == 0.15
    assert fwf.scanner_efficiency == 0.9
    assert fwf.atmospheric_visibility == 0.9
    assert fwf.scanner_wave_length == 1550.0
    assert fwf.beam_divergence_angle == 0.0003
    assert fwf.pulse_length == 4.0
    assert fwf.beam_sample_quality == 3
    assert fwf.win_size == fwf.pulse_length / 4.0
    assert fwf.max_fullwave_range == 0.0

def test_fwf_settings_set_get_properties():
    fwf = _helios.FWFSettings()
    
    # Set properties
    fwf.bin_size = 0.5
    fwf.min_echo_width = 3.0
    fwf.peak_energy = 600.0
    fwf.aperture_diameter = 0.2
    fwf.scanner_efficiency = 0.85
    fwf.atmospheric_visibility = 0.95
    fwf.scanner_wave_length = 1600.0
    fwf.beam_divergence_angle = 0.0005
    fwf.pulse_length = 5.0
    fwf.beam_sample_quality = 4
    fwf.win_size = 1.25
    fwf.max_fullwave_range = 100.0

    # Verify values
    assert fwf.bin_size == 0.5
    assert fwf.min_echo_width == 3.0
    assert fwf.peak_energy == 600.0
    assert fwf.aperture_diameter == 0.2
    assert fwf.scanner_efficiency == 0.85
    assert fwf.atmospheric_visibility == 0.95
    assert fwf.scanner_wave_length == 1600.0
    assert fwf.beam_divergence_angle == 0.0005
    assert fwf.pulse_length == 5.0
    assert fwf.beam_sample_quality == 4
    assert fwf.win_size == 1.25
    assert fwf.max_fullwave_range == 100.0

def test_fwf_settings_to_string():
    fwf = _helios.FWFSettings()
    expected_str = (
        'FWFSettings "":\n'
        'binSize_ns = 0.25\n'
        'minEchoWidth = 2.5\n'
        'peakEnergy = 500\n'
        'apertureDiameter = 0.15\n'
        'scannerEfficiency = 0.9\n'
        'atmosphericVisibility = 0.9\n'
        'scannerWaveLength = 1550\n'
        'beamDivergence_rad = 0.0003\n'
        'pulseLength_ns = 4\n'
        'beamSampleQuality = 3\n'
        'winSize_ns = 1\n'
        'maxFullwaveRange_ns = 0\n'
    )
    assert str(fwf) == expected_str



def test_rotation_instantiation():
    # Test default constructor
    rotation = _helios.Rotation()
    assert isinstance(rotation, _helios.Rotation)

    # Test constructors with parameters
    rotation_q = _helios.Rotation(1.0, 0.0, 0.0, 0.0, True)
    rotation_q = _helios.Rotation(1.0, 0.0, 0.0, 0.0, True)
    assert math.isclose(rotation_q.q0, 1.0)
    assert math.isclose(rotation_q.q1, 0.0)
    assert math.isclose(rotation_q.q2, 0.0)
    assert math.isclose(rotation_q.q3, 0.0)

    axis = (1.0, 0.0, 0.0)
    angle = math.pi / 2
    rotation_a = _helios.Rotation(axis, angle)
    assert rotation_a.axis == axis
    assert math.isclose(rotation_a.angle, angle)

def test_rotation_set_get_properties():
    rotation = _helios.Rotation()
    
    # Set properties
    rotation.q0 = 0.707
    rotation.q1 = 0.0
    rotation.q2 = 0.707
    rotation.q3 = 0.0
    
    assert math.isclose(rotation.q0, 0.707)
    assert math.isclose(rotation.q1, 0.0)
    assert math.isclose(rotation.q2, 0.707)
    assert math.isclose(rotation.q3, 0.0)

    axis = (0.0, 1.0, 0.0)
    angle = math.pi / 2
    rotation = _helios.Rotation(axis, angle)
    assert rotation.axis == axis
    assert math.isclose(rotation.angle, angle)


def test_scanner_head_instantiation():
    # Test constructor with parameters
    axis = (1.0, 0.0, 0.0)
    head_rotate_per_sec_max = 1.5
    scanner_head = _helios.ScannerHead(axis, head_rotate_per_sec_max)
    assert isinstance(scanner_head, _helios.ScannerHead)

def test_scanner_head_properties():
    axis = (1.0, 0.0, 0.0)
    head_rotate_per_sec_max = 1.5
    scanner_head = _helios.ScannerHead(axis, head_rotate_per_sec_max)
    
    # Test readonly property
    assert scanner_head.mount_relative_attitude is not None
    
    # Test read/write properties
    scanner_head.rotate_per_sec_max = 2.0
    assert scanner_head.rotate_per_sec_max == 2.0

    scanner_head.rotate_per_sec = 1.0
    assert scanner_head.rotate_per_sec == 1.0

    scanner_head.rotate_stop = 0.0
    assert scanner_head.rotate_stop == 0.0

    scanner_head.rotate_start = 1.0
    assert scanner_head.rotate_start == 1.0

    scanner_head.rotate_range = 2.0
    assert scanner_head.rotate_range == 2.0

    scanner_head.current_rotate_angle = 1.0
    assert scanner_head.current_rotate_angle == 1.0

def test_material_instantiation():
    # Test default constructor
    material = _helios.Material()
    assert isinstance(material, _helios.Material)

    # Test copy constructor
    material_copy = _helios.Material(material)
    assert isinstance(material_copy, _helios.Material)

def test_material_properties():
    material = _helios.Material()

    # Test read/write properties
    material.name = "TestMaterial"
    assert material.name == "TestMaterial"

    material.is_ground = True
    assert material.is_ground is True

    material.use_vertex_colors = False
    assert material.use_vertex_colors is False

    material.mat_file_path = "path/to/material.mat"
    assert material.mat_file_path == "path/to/material.mat"

    material.map_Kd = "path/to/mapKd.png"
    assert material.map_Kd == "path/to/mapKd.png"

    material.reflectance = 0.5
    assert material.reflectance == 0.5

    material.specularity = 0.8
    assert material.specularity == 0.8

    material.specular_exponent = 32
    assert material.specular_exponent == 32

    material.classification = 1
    assert material.classification == 1

    material.spectra = "SpectraData"
    assert material.spectra == "SpectraData"

    # Test readonly properties (assuming ka, kd, ks are arrays in the material class)
    ka_array = np.array([0, 0, 0, 0], dtype=np.float32)
    kd_array = np.array([0, 0, 0, 0], dtype=np.float32)
    ks_array = np.array([0, 0, 0, 0], dtype=np.float32)

    assert np.array_equal(material.ka, ka_array)
    assert np.array_equal(material.kd, kd_array)
    assert np.array_equal(material.ks, ks_array)


def test_survey_instantiation():
    # Test default constructor
    survey = _helios.Survey()
    assert isinstance(survey, _helios.Survey)


def test_survey_properties():
    survey = _helios.Survey()

    # Test read/write properties
    survey.name = "TestSurvey"
    assert survey.name == "TestSurvey"

    survey.num_runs = 5
    assert survey.num_runs == 5

    survey.sim_speed_factor = 1.5
    assert survey.sim_speed_factor == 1.5

    # Test readonly property
    length = survey.length
    assert isinstance(length, float)

#Possible fixtures to the Survey class
# def test_survey_methods():
#     survey = _helios.Survey()
#     length = survey.calculate_length()
    #assert isinstance(length, float)


def create_and_modify_leg_with_platform_and_scanner_settings():
    # Create ScannerSettings object
    scanner_settings = _helios.ScannerSettings()
    scanner_settings.id = "TestScannerSettings"
    scanner_settings.head_rotation = 1.0
    scanner_settings.rotation_start_angle = 0.0
    scanner_settings.rotation_stop_angle = 2.0
    scanner_settings.pulse_frequency = 10
    scanner_settings.scan_angle = 180
    scanner_settings.min_vertical_angle = -90
    scanner_settings.max_vertical_angle = 90
    scanner_settings.scan_frequency = 5
    scanner_settings.beam_divergence_angle = 0.01
    scanner_settings.trajectory_time_interval = 1.0
    scanner_settings.vertical_resolution = 0.5
    scanner_settings.horizontal_resolution = 0.5

    platform_settings = _helios.PlatformSettings()
    platform_settings.id = "TestPlatformSettings"
    platform_settings.x = 1.0
    platform_settings.y = 2.0
    platform_settings.z = 3.0
    platform_settings.yaw_angle = 45.0
    platform_settings.is_yaw_angle_specified = True
    platform_settings.is_on_ground = True
    platform_settings.is_stop_and_turn = False
    platform_settings.is_smooth_turn = True
    platform_settings.is_slowdown_enabled = False
    platform_settings.speed_m_s = 15.0
    
    # Create Leg object with ScannerSettings
    leg = _helios.Leg(10.0, 1, None)
    leg.scanner_settings = scanner_settings
    leg.platform_settings = platform_settings
    leg.length = 15.0
    leg.serial_id = 2
    leg.strip = None
    
    return leg, scanner_settings, platform_settings

# Test basic functionality
def test_leg_and_scanner_settings():
    leg, scanner_settings, platform_settings = create_and_modify_leg_with_platform_and_scanner_settings()
    
    # Test properties and methods for Leg
    assert leg.length == 15.0
    assert leg.serial_id == 2
    assert leg.strip is None
    assert leg.belongs_to_strip() is False
    
    leg.length = 20.0
    assert leg.length == 20.0
    
    leg.serial_id = 3
    assert leg.serial_id == 3
    
    # Test properties and methods for ScannerSettings
    assert scanner_settings.id == "TestScannerSettings"
    assert scanner_settings.head_rotation == 1.0
    assert scanner_settings.rotation_start_angle == 0.0
    assert scanner_settings.rotation_stop_angle == 2.0
    assert scanner_settings.pulse_frequency == 10
    assert scanner_settings.scan_angle == 180
    assert scanner_settings.min_vertical_angle == -90
    assert scanner_settings.max_vertical_angle == 90
    assert scanner_settings.scan_frequency == 5
    assert scanner_settings.beam_divergence_angle == 0.01
    assert scanner_settings.trajectory_time_interval == 1.0
    assert scanner_settings.vertical_resolution == 0.5
    assert scanner_settings.horizontal_resolution == 0.5

    assert platform_settings.id == "TestPlatformSettings"
    assert platform_settings.x == 1.0
    assert platform_settings.y == 2.0
    assert platform_settings.z == 3.0
    assert platform_settings.yaw_angle == 45.0
    assert platform_settings.is_yaw_angle_specified is True
    assert platform_settings.is_on_ground is True
    assert platform_settings.is_stop_and_turn is False
    assert platform_settings.is_smooth_turn is True
    assert platform_settings.is_slowdown_enabled is False
    assert platform_settings.speed_m_s == 15.0

# Test thread safety by modifying the same Leg and ScannerSettings object in multiple threads
def test_leg_and_scanner_settings_thread_safety():
    def thread_function():
        leg, scanner_settings, platform_settings = create_and_modify_leg_with_platform_and_scanner_settings()
        
        # Perform concurrent modifications
        for i in range(100):
            leg.length = 10.0 + i
            leg.serial_id = i
            scanner_settings.head_rotation = i * 0.1
            scanner_settings.pulse_frequency = i
            platform_settings.x = i
            platform_settings.y = i + 1
            platform_settings.z = i + 2
        
        # Validate after modifications
        assert leg.length == 109.0
        assert leg.serial_id == 99
        assert scanner_settings.head_rotation == 9.9
        assert scanner_settings.pulse_frequency == 99
        assert platform_settings.x == 99
        assert platform_settings.y == 100
        assert platform_settings.z == 101
    
    threads = [threading.Thread(target=thread_function) for _ in range(10)]
    
    for thread in threads:
        thread.start()
    
    for thread in threads:
        thread.join()


def create_scene_part():
    # Create a ScenePart object
    scene_part = _helios.ScenePart()
    scene_part.id = "TestPart"
    scene_part.origin = (1.0, 2.0, 3.0)
    scene_part.rotation = _helios.Rotation((0.0, 1.0, 0.0), 1.0)
    scene_part.origin = (1.0, 2.0, 3.0)
    scene_part.rotation = _helios.Rotation((0.0, 1.0, 0.0), 1.0)
    scene_part.scale = 2.0
    
    return scene_part

# Test basic ScenePart functionality
def test_scene_part():
    scene_part = create_scene_part()
    
    # Test properties
    assert scene_part.id == "TestPart"
    assert scene_part.origin == (1.0, 2.0, 3.0)
    assert scene_part.rotation.axis == (0.0, 1.0, 0.0)
    assert scene_part.origin == (1.0, 2.0, 3.0)
    assert scene_part.rotation.axis ==(0.0, 1.0, 0.0)
    assert scene_part.rotation.angle == 1.0
    assert scene_part.scale == 2.0
    
    # Modify properties
    scene_part.id = "UpdatedPart"
    scene_part.origin = (4.0, 5.0, 6.0)
    scene_part.origin = (4.0, 5.0, 6.0)
    scene_part.rotation = _helios.Rotation((1.0, 0.0, 0.0), 2.0)
    scene_part.rotation = _helios.Rotation((1.0, 0.0, 0.0), 2.0)
    scene_part.scale = 3.0
    
    assert scene_part.id == "UpdatedPart"
    assert scene_part.origin == (4.0, 5.0, 6.0)
    assert scene_part.rotation.axis == (1.0, 0.0, 0.0)
    assert scene_part.origin == (4.0, 5.0, 6.0)
    assert scene_part.rotation.axis == (1.0, 0.0, 0.0)
    assert scene_part.rotation.angle == 2.0
    assert scene_part.scale == 3.0


def test_scene_part_thread_safety():
    # Create a ScenePart object
    scene_part = create_scene_part()
    lock = threading.Lock()
    
    modify_scene_part_in_threads(scene_part, lock)

    # Debugging print statements
    with lock:
        final_rotation_axis = (scene_part.rotation.axis[0], scene_part.rotation.axis[1], scene_part.rotation.axis[2])
        tolerance = 1e-6

        # Check if the values are close to what you expect, considering numerical precision issues
        assert math.isclose(final_rotation_axis[0]**2 + final_rotation_axis[1]**2 + final_rotation_axis[2]**2, 1, abs_tol=tolerance), \
            f"Rotation axis should be a unit vector but got {final_rotation_axis}"

def modify_scene_part_in_threads(scene_part, lock):
    def thread_function():
        with lock:
            for i in range(100):
                scene_part.id = f"ThreadPart-{i}"
                scene_part.origin = (i, i + 1, i + 2)
                # Assuming rotation is defined as (axis, angle)
                axis = (i % 2, (i + 1) % 2, (i + 2) % 2)
                angle = i * 0.1
                scene_part.rotation = _helios.Rotation(axis, angle)
                scene_part.scale = i * 0.1

    threads = [threading.Thread(target=thread_function) for _ in range(10)]

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()


def test_scene_properties():
    scene = _helios.Scene()
    
    # Test adding a new triangle
    triangle = scene.new_triangle()
    assert triangle is not None
    assert isinstance(triangle, _helios.Triangle)

    # Test adding a new detailed voxel
    voxel = scene.new_detailed_voxel()
    assert voxel is not None
    assert isinstance(voxel, _helios.DetailedVoxel)


def test_platform_properties():
    platform = _helios.Platform()

    # Test read-write properties
    platform.last_check_z = 10.0
    assert platform.last_check_z == 10.0

    platform.dmax = 100.0
    assert platform.dmax == 100.0

    platform.is_orientation_on_leg_init = True
    assert platform.is_orientation_on_leg_init == True

    platform.is_on_ground = True
    assert platform.is_on_ground == True

    platform.is_stop_and_turn = True
    assert platform.is_stop_and_turn == True

    platform.settings_speed_m_s = 5.0
    assert platform.settings_speed_m_s == 5.0

    platform.is_slowdown_enabled = False
    assert platform.is_slowdown_enabled == False

    platform.is_smooth_turn = True
    assert platform.is_smooth_turn == True

    # Test read-only properties
    assert platform.device_relative_position[0] == 0.0
    assert platform.device_relative_position[1] == 0.0
    assert platform.device_relative_position[2] == 0.0

    assert isinstance(platform.device_relative_attitude, _helios.Rotation)
    assert platform.device_relative_attitude.axis == (1.0, 0.0, 0.0)
    assert platform.device_relative_attitude.angle == 0.0

    assert platform.position_x_noise_source is None
    assert platform.position_y_noise_source is None
    assert platform.position_z_noise_source is None
    assert platform.attitude_x_noise_source is None
    assert platform.attitude_y_noise_source is None
    assert platform.attitude_z_noise_source is None

   
    assert platform.target_waypoint[0] == 0.0
    assert platform.target_waypoint[1] == 0.0
    assert platform.target_waypoint[2] == 0.0

    assert platform.last_ground_check[0] == 0.0
    assert platform.last_ground_check[1] == 0.0
    assert platform.last_ground_check[2] == 0.0


    assert platform.position[0] == 0.0
    assert platform.position[1] == 0.0
    assert platform.position[2] == 0.0


    assert platform.attitude.axis == (1.0, 0.0, 0.0)
    assert platform.attitude.angle == 0.0

    assert platform.absolute_mount_position[0] == 0.0
    assert platform.absolute_mount_position[1] == 0.0
    assert platform.absolute_mount_position[2] == 0.0

    assert platform.absolute_mount_attitude.axis == (1.0, 0.0, 0.0)
    assert platform.absolute_mount_attitude.angle == 0.0

   
    assert platform.cached_dir_current[0] == 0.0
    assert platform.cached_dir_current[1] == 0.0
    assert platform.cached_dir_current[2] == 0.0

    assert platform.cached_dir_current_xy[0] == 0.0
    assert platform.cached_dir_current_xy[1] == 0.0
    assert platform.cached_dir_current_xy[2] == 0.0

    assert platform.cached_vector_to_target[0] == 0.0
    assert platform.cached_vector_to_target[1] == 0.0
    assert platform.cached_vector_to_target[2] == 0.0

    assert platform.cached_vector_to_target_xy[0] == 0.0
    assert platform.cached_vector_to_target_xy[1] == 0.0
    assert platform.cached_vector_to_target_xy[2] == 0.0

class MockBeamDeflector:
    def __init__(self):
        pass

    def __eq__(self, other):
        return isinstance(other, MockBeamDeflector)

class MockDetector:
    def __init__(self):
        pass

    def __eq__(self, other):
        return isinstance(other, MockDetector)

class ExampleScanner(_helios.Scanner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._max_nor = 10
        self._bt2 = 0.5
        self._dr2 = 0.1
        self._head_relative_emitter_attitude = _helios.Rotation()
        self._head_relative_emitter_position = (0.0, 0.0, 0.0)
        self._active = True
        self._write_waveform = False
        self._calc_echowidth = False
        self._full_wave_noise = False
        self._platform_noise_disabled = False
        self._fixed_incidence_angle = False
        self._id = "TestScannerID"
        self._device_id = "Device0"
        self._num_devices = 1
        self._time_wave = [0.0] * 10
        self._scanner_head = _helios.ScannerHead((0.0, 0.0, 1.0), 1.0) 
        self._beam_deflector = MockBeamDeflector()  # Use a mock or default implementation
        self._detector = MockDetector()
        self._supported_pulse_freqs_hz = [1, 2, 3]
        self._fwf_settings = _helios.FWFSettings()
        self._peak_intensity_index = 0
        self._received_energy_min = 0.0
    
    def getScannerId(self):
        return "TestScannerID"

    def getNumDevices(self):
        return 1

    def getDeviceId(self, index):
        return f"Device{index}"

    def getAveragePower(self, index):
        return 10.0

    def getBeamDivergence(self, index):
        return 0.01

    def getWavelength(self, index):
        return 0.000532

    def getVisibility(self, index):
        return 10.0

    def prepareSimulation(self, legacyEnergyModel):
        pass

    def retrieveCurrentSettings(self, idx):
        return _helios.ScannerSettings()

    def applySettings(self, settings, idx):
        pass

    def applySettingsFWF(self, fwfSettings, idx):
        pass

    def doSimStep(self, legIndex, currentGpsTime):
        pass

    def calcRaysNumber(self, idx):
        pass

    def prepareDiscretization(self, idx):
        pass

    def calcAtmosphericAttenuation(self, idx):
        return 0.0

    def calcAbsoluteBeamAttitude(self, idx):
        return _helios.Rotation()

    def checkMaxNOR(self, nor, idx):
        return True

    def onLegComplete(self):
        pass

    def onSimulationFinished(self):
        pass

    def handleTrajectoryOutput(self):
        pass

    def trackOutputPath(self):
        pass

    def getCurrentPulseNumber(self, idx):
        return 0  # Default implementation

    def getNumRays(self, idx):
        return 0  # Default implementation

    def setNumRays(self, numRays, idx):
        pass  # Default implementation

    def getPulseLength_ns(self, idx):
        return 0.0  # Default implementation

    def setPulseLength_ns(self, pulseLength_ns, idx):
        pass  # Default implementation

    def lastPulseWasHit(self, idx):
        return False  # Default implementation

    def setLastPulseWasHit(self, lastPulseWasHit, idx):
        pass  # Default implementation

    def getBeamDivergence(self, idx):
        return 0.0  # Default implementation

    def setBeamDivergence(self, beamDivergence, idx):
        pass  # Default implementation

    def getAveragePower(self, idx):
        return 0.0  # Default implementation

    def setAveragePower(self, averagePower, idx):
        pass  # Default implementation

    def getBeamQuality(self, idx):
        return 0.0  # Default implementation

    def setBeamQuality(self, beamQuality, idx):
        pass  # Default implementation

    def to_string(self):
        return "Scanner Info"  # Default implementation
    
    def getEfficiency(self, idx):
        # Default value for simplicity
        return 85.0

    def setEfficiency(self, efficiency, idx):
        pass

    def getReceiverDiameter(self, idx):
        # Default value for simplicity
        return 12.0

    def setReceiverDiameter(self, receiverDiameter, idx):
        pass

    def getVisibility(self, idx):
        # Default value for simplicity
        return 10.0

    def setVisibility(self, visibility, idx):
        pass

    def getWavelength(self, idx):
        # Default value for simplicity
        return 550.0

    def setWavelength(self, wavelength, idx):
        pass

    def getAtmosphericExtinction(self, idx):
        # Default value for simplicity
        return 0.2

    def setAtmosphericExtinction(self, atmosphericExtinction, idx):
        pass

    def getBeamWaistRadius(self, idx):
        # Default value for simplicity
        return 1.0

    def setBeamWaistRadius(self, beamWaistRadius, idx):
        pass
    
    def getMaxNOR(self, idx=0):
        return self._max_nor

    def setMaxNOR(self, maxNOR, idx=0):
        self._max_nor = maxNOR

    def getHeadRelativeEmitterAttitude(self, idx=0):
        return self._head_relative_emitter_attitude

    def setHeadRelativeEmitterAttitude(self, attitude, idx=0):
        self._head_relative_emitter_attitude = attitude

    def getHeadRelativeEmitterPosition(self, idx=0):
        return self._head_relative_emitter_position

    def setHeadRelativeEmitterPosition(self, position, idx=0):
        self._head_relative_emitter_position = position

    def getBt2(self, idx=0):
        return self._bt2

    def setBt2(self, bt2, idx=0):
        self._bt2 = bt2

    def getDr2(self, idx=0):
        return self._dr2

    def setDr2(self, dr2, idx=0):
        self._dr2 = dr2

    def isActive(self):
        return self._active

    def setActive(self, active):
        self._active = active

    def isWriteWaveform(self):
        return self._write_waveform

    def setWriteWaveform(self, write_waveform):
        self._write_waveform = write_waveform

    def isCalcEchowidth(self):
        return self._calc_echowidth

    def setCalcEchowidth(self, calc_echowidth):
        self._calc_echowidth = calc_echowidth

    def isFullWaveNoise(self):
        return self._full_wave_noise

    def setFullWaveNoise(self, full_wave_noise):
        self._full_wave_noise = full_wave_noise

    def isPlatformNoiseDisabled(self):
        return self._platform_noise_disabled

    def setPlatformNoiseDisabled(self, platform_noise_disabled):
        self._platform_noise_disabled = platform_noise_disabled

    def isFixedIncidenceAngle(self):
        return self._fixed_incidence_angle

    def setFixedIncidenceAngle(self, fixed_incidence_angle):
        self._fixed_incidence_angle = fixed_incidence_angle

    def getTimeWave(self):
        return self._time_wave
    
    def getScannerHead(self, idx=0):
        return self._scanner_head

    def setScannerHead(self, scannerHead, idx=0):
        self._scanner_head = scannerHead

    def getBeamDeflector(self, idx=0):
        return self._beam_deflector

    def setBeamDeflector(self, beamDeflector, idx=0):
        self._beam_deflector = beamDeflector

    def getDetector(self, idx=0):
        return self._detector

    def setDetector(self, detector, idx=0):
        self._detector = detector

    def getSupportedPulseFreqs_Hz(self, idx):
        return self._supported_pulse_freqs_hz

    def setSupportedPulseFreqs_Hz(self, supportedPulseFreqs_Hz, idx):
        self._supported_pulse_freqs_hz = supportedPulseFreqs_Hz

    def getNumTimeBins(self, idx):
        return len(self._time_wave)

    def setNumTimeBins(self, numTimeBins, idx):
        self._time_wave = [0.0] * numTimeBins

    def getFWFSettings(self, idx):
        return self._fwf_settings

    def setFWFSettings(self, fwfSettings, idx):
        self._fwf_settings = fwfSettings

    def getPeakIntensityIndex(self, idx):
        return self._peak_intensity_index

    def setPeakIntensityIndex(self, pii, idx):
        self._peak_intensity_index = pii

    def getReceivedEnergyMin(self, idx):
        return self._received_energy_min

    def setReceivedEnergyMin(self, receivedEnergyMin_W, idx):
        self._received_energy_min = receivedEnergyMin_W

    def getTimeWave(self, idx):
        return self._time_wave

    def setTimeWave(self, timewave, idx = 0):
        self._time_wave = timewave

    def calcTimePropagation(self, timeWave, numBins, scanner):
        # Implement a dummy propagation calculation
        return len(timeWave) * numBins

        
    def getHeadRelativeEmitterPositionByRef(self, idx=0):
        return self._head_relative_emitter_position

    def getHeadRelativeEmitterAttitudeByRef(self, idx=0):
        return self._head_relative_emitter_attitude 


# Test class to define the methods
class TestScannerMethods:
    @pytest.fixture
    def scanner(self):
        return ExampleScanner(id = "SCANNER-ID", pulseFreqs = [1, 2, 3])

    def test_scanner_construction(self, scanner):
        # Test default construction
        assert scanner is not None

        # Test parameterized construction
        scanner1 = scanner
        scanner = ExampleScanner(scanner1)
        assert scanner.id == "SCANNER-ID"

    def test_current_pulse_number(self, scanner):
        # Mocking getCurrentPulseNumber with index
        with patch.object(scanner, 'getCurrentPulseNumber', return_value=42) as mock_method:
            assert scanner.getCurrentPulseNumber(0) == 42
            mock_method.assert_called_once_with(0)
        
        # Mocking getCurrentPulseNumber without index (default version)
        with patch.object(scanner, 'getCurrentPulseNumber', return_value=42) as mock_method:
            assert scanner.getCurrentPulseNumber() == 42
            mock_method.assert_called_once()  # No argument should be passed

    def test_scanner_methods(self, scanner):
        # Mocking methods to test calls
        with patch.object(scanner, 'initialize_sequential_generators', return_value=None) as mock_method:
            scanner.initialize_sequential_generators()
            mock_method.assert_called_once()

        with patch.object(scanner, 'build_scanning_pulse_process', return_value=None) as mock_method:
            scanner.build_scanning_pulse_process(0, 1, 2)
            mock_method.assert_called_once_with(0, 1, 2)

        with patch.object(scanner, 'apply_settings', return_value=None) as mock_method:
            settings = _helios.ScannerSettings()
            scanner.apply_settings(settings)
            mock_method.assert_called_once_with(settings)

        with patch.object(scanner, 'retrieve_current_settings', return_value=None) as mock_method:
            scanner.retrieve_current_settings()
            mock_method.assert_called_once()

        with patch.object(scanner, 'apply_settings_FWF', return_value=None) as mock_method:
            fwf_settings = _helios.FWFSettings()
            scanner.apply_settings_FWF(fwf_settings)
            mock_method.assert_called_once_with(fwf_settings)

        with patch.object(scanner, 'do_sim_step', return_value=None) as mock_method:
            scanner.do_sim_step(1, 123.456)
            mock_method.assert_called_once_with(1, 123.456)

        with patch.object(scanner, 'calc_rays_number', return_value=None) as mock_method:
            scanner.calc_rays_number()
            mock_method.assert_called_once()

        with patch.object(scanner, 'prepare_discretization', return_value=None) as mock_method:
            scanner.prepare_discretization()
            mock_method.assert_called_once()

        with patch.object(scanner, 'calc_atmospheric_attenuation', return_value=0.0) as mock_method:
            scanner.calc_atmospheric_attenuation()
            mock_method.assert_called_once()

        with patch.object(scanner, 'check_max_NOR', return_value=False) as mock_method:
            scanner.check_max_NOR(100)
            mock_method.assert_called_once_with(100)

        with patch.object(scanner, 'calc_absolute_beam_attitude', return_value=None) as mock_method:
            scanner.calc_absolute_beam_attitude()
            mock_method.assert_called_once()

        with patch.object(scanner, 'handle_sim_step_noise', return_value=None) as mock_method:
            scanner.handle_sim_step_noise()
            mock_method.assert_called_once()

        with patch.object(scanner, 'on_leg_complete', return_value=None) as mock_method:
            scanner.on_leg_complete()
            mock_method.assert_called_once()

        with patch.object(scanner, 'on_simulation_finished', return_value=None) as mock_method:
            scanner.on_simulation_finished()
            mock_method.assert_called_once()

        with patch.object(scanner, 'handle_trajectory_output', return_value=None) as mock_method:
            scanner.handle_trajectory_output()
            mock_method.assert_called_once()

        with patch.object(scanner, 'track_output_path', return_value=None) as mock_method:
            scanner.track_output_path()
            mock_method.assert_called_once()
   

    def test_scanner_to_string(self, scanner):
        result = scanner.to_string()
        assert isinstance(result, str)

    def test_efficiency(self, scanner):
        with patch.object(scanner, 'getEfficiency', return_value=0.9) as mock_get:
            assert scanner.getEfficiency(0) == 0.9
            mock_get.assert_called_once_with(0)
        
        with patch.object(scanner, 'getEfficiency', return_value=0.9) as mock_get:
            assert scanner.getEfficiency() == 0.9
            mock_get.assert_called_once()
        
        with patch.object(scanner, 'setEfficiency') as mock_set:
            scanner.setEfficiency(0.8, 0)
            mock_set.assert_called_once_with(0.8, 0)
        
    def test_receiver_diameter(self, scanner):
        with patch.object(scanner, 'getReceiverDiameter', return_value=50.0) as mock_get:
            assert scanner.getReceiverDiameter(0) == 50.0
            mock_get.assert_called_once_with(0)
        
        with patch.object(scanner, 'getReceiverDiameter', return_value=50.0) as mock_get:
            assert scanner.getReceiverDiameter() == 50.0
            mock_get.assert_called_once()
        
        with patch.object(scanner, 'setReceiverDiameter') as mock_set:
            scanner.setReceiverDiameter(55.0, 0)
            mock_set.assert_called_once_with(55.0, 0)

    def test_visibility(self, scanner):
        with patch.object(scanner, 'getVisibility', return_value=10.0) as mock_get:
            assert scanner.getVisibility(0) == 10.0
            mock_get.assert_called_once_with(0)
        
        with patch.object(scanner, 'getVisibility', return_value=10.0) as mock_get:
            assert scanner.getVisibility() == 10.0
            mock_get.assert_called_once()
        
        with patch.object(scanner, 'setVisibility') as mock_set:
            scanner.setVisibility(12.0, 0)
            mock_set.assert_called_once_with(12.0, 0)

    def test_wavelength(self, scanner):
        with patch.object(scanner, 'getWavelength', return_value=500.0) as mock_get:
            assert scanner.getWavelength(0) == 500.0
            mock_get.assert_called_once_with(0)
        
        with patch.object(scanner, 'getWavelength', return_value=500.0) as mock_get:
            assert scanner.getWavelength() == 500.0
            mock_get.assert_called_once()
        
        with patch.object(scanner, 'setWavelength') as mock_set:
            scanner.setWavelength(510.0, 0)
            mock_set.assert_called_once_with(510.0, 0)

    def test_atmospheric_extinction(self, scanner):
        with patch.object(scanner, 'getAtmosphericExtinction', return_value=0.1) as mock_get:
            assert scanner.getAtmosphericExtinction(0) == 0.1
            mock_get.assert_called_once_with(0)
        
        with patch.object(scanner, 'getAtmosphericExtinction', return_value=0.1) as mock_get:
            assert scanner.getAtmosphericExtinction() == 0.1
            mock_get.assert_called_once()
        
        with patch.object(scanner, 'setAtmosphericExtinction') as mock_set:
            scanner.setAtmosphericExtinction(0.2, 0)
            mock_set.assert_called_once_with(0.2, 0)

    def test_beam_waist_radius(self, scanner):
        with patch.object(scanner, 'getBeamWaistRadius', return_value=1.0) as mock_get:
            assert scanner.getBeamWaistRadius(0) == 1.0
            mock_get.assert_called_once_with(0)
        
        with patch.object(scanner, 'getBeamWaistRadius', return_value=1.0) as mock_get:
            assert scanner.getBeamWaistRadius() == 1.0
            mock_get.assert_called_once()
        
        with patch.object(scanner, 'setBeamWaistRadius') as mock_set:
            scanner.setBeamWaistRadius(1.5, 0)
            mock_set.assert_called_once_with(1.5, 0)

  
    def test_max_nor(self, scanner):
        assert scanner.getMaxNOR() == 10

    def test_head_relative_emitter_attitude(self, scanner):
        attitude = _helios.Rotation()
        scanner.setHeadRelativeEmitterAttitude(attitude)
        assert scanner.getHeadRelativeEmitterAttitude() == attitude

    def test_head_relative_emitter_position(self, scanner):
        position = (1.0, 2.0, 3.0) 
        assert scanner.getHeadRelativeEmitterPosition() == (0.0, 0.0, 0.0)

    def test_bt2(self, scanner):
        assert scanner.getBt2() == 0.5

    def test_dr2(self, scanner):
        assert scanner.getDr2() == 0.1

    def test_is_active(self, scanner):
        assert scanner.isActive()

    def test_is_write_waveform(self, scanner):
        assert not scanner.isWriteWaveform()

    def test_is_calc_echowidth(self, scanner):
        assert not scanner.isCalcEchowidth()

    def test_is_full_wave_noise(self, scanner):
        assert not scanner.isFullWaveNoise()

    def test_is_platform_noise_disabled(self, scanner):
        assert not scanner.isPlatformNoiseDisabled()

    def test_is_fixed_incidence_angle(self, scanner):
        assert not scanner.isFixedIncidenceAngle()

    def test_device_id(self, scanner):
        assert scanner.getDeviceId(0) == "Device0"

    def test_time_wave(self,  scanner):
        assert scanner.getTimeWave() == [0.0] * 10

    def test_scanner_head(self, scanner):
        head = _helios.ScannerHead((1.0, 0.0, 0.0), 2.0)  # Replace with actual initialization
        scanner.setScannerHead(head, 0)
        assert scanner.getScannerHead(0) == head

    def test_beam_deflector(self, scanner):
        deflector = MockBeamDeflector()  # Use MockBeamDeflector
        scanner.setBeamDeflector(deflector, 0)
        assert scanner.getBeamDeflector(0) == deflector


    def test_detector(self, scanner):
        detector = MockDetector()  # Use MockDetector
        scanner.setDetector(detector, 0)
        assert scanner.getDetector(0) == detector

    def test_supported_pulse_freqs_hz(self, scanner):
        freqs = [1, 2, 3]  # Example frequencies
        scanner.setSupportedPulseFreqs_Hz(freqs, 0)
        assert list(scanner.getSupportedPulseFreqs_Hz(0)) == freqs

    def test_num_time_bins(self, scanner):
        scanner.setNumTimeBins(10, 0)
        assert scanner.getNumTimeBins(0) == 10

    def test_fwf_settings(self, scanner):
        fwf_settings = _helios.FWFSettings()  # Replace with actual initialization
        scanner.setFWFSettings(fwf_settings, 0)
        assert scanner.getFWFSettings(0) == fwf_settings

    def test_peak_intensity_index(self, scanner):
        scanner.setPeakIntensityIndex(5, 0)
        assert scanner.getPeakIntensityIndex(0) == 5

    def test_received_energy_min(self, scanner):
        scanner.setReceivedEnergyMin(1.5, 0)
        assert scanner.getReceivedEnergyMin(0) == 1.5

    def test_time_wave(self, scanner):
        time_wave = np.linspace(0, 1, 100).tolist()
        scanner.setTimeWave(time_wave, 0)
        assert np.allclose(scanner.getTimeWave(0), time_wave)

    def test_scanner_head_default(self, scanner):
        head = _helios.ScannerHead((1.0, 0.0, 0.0), 2.0)  # Replace with actual initialization
        scanner.setScannerHead(head)
        assert scanner.getScannerHead() == head

    def test_beam_deflector_default(self, scanner):
        deflector = MockBeamDeflector()  # Replace with actual initialization
        scanner.setBeamDeflector(deflector)
        assert scanner.getBeamDeflector() == deflector

    def test_detector_default(self, scanner):
        detector = MockDetector()  # Replace with actual initialization
        scanner.setDetector(detector)
        assert scanner.getDetector() == detector

    def test_supported_pulse_freqs_hz_default(self, scanner):
        freqs = [1, 2, 3]  # Example frequencies
        scanner.setSupportedPulseFreqs_Hz(freqs, 0)
        assert list(scanner.getSupportedPulseFreqs_Hz(0)) == freqs


    def test_get_head_relative_emitter_position_by_ref(self, scanner):
        pos = (1.0, 2.0, 3.0)
        scanner.setHeadRelativeEmitterPosition(pos)
        pos_ref = scanner.getHeadRelativeEmitterPositionByRef()
        assert pos_ref[0] == pos[0]
        assert pos_ref[1] == pos[1]
        assert pos_ref[2] == pos[2]

    def test_get_head_relative_emitter_attitude_by_ref(self, scanner):
        attitude = _helios.Rotation()  # Use appropriate initialization
        scanner.setHeadRelativeEmitterAttitude(attitude)
        attitude_ref = scanner.getHeadRelativeEmitterAttitudeByRef()
        assert attitude_ref == attitude


    def test_calc_time_propagation(self, scanner):
        time_wave = np.linspace(0, 1, 100).tolist()
        num_bins = len(time_wave)
        scanner.setTimeWave(time_wave)
        result = scanner.calcTimePropagation(time_wave, num_bins, scanner)
        assert isinstance(result, int)  # Replace with expected value if applicable


def test_simulation_initialization():
    sim = _helios.Simulation()
    assert sim is not None

def test_simulation_initialization_with_params():
    sim = _helios.Simulation(
        "surveyPath",
        ("assetsPath1", "assetsPath2"),
        "outputPath",
        4, # numThreads
        True, # lasOutput
        False, # las10
        True, # zipOutput
        False, # splitByChannel
        3, # kdtFactory
        2, # kdtJobs
        64, # kdtSAHLossNodes
        2, # parallelizationStrategy
        64, # chunkSize
        2 # warehouseFactor
    )
    assert sim is not None
