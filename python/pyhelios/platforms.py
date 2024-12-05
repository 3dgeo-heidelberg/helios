from pyhelios.utils import Validatable, ValidatedCppManagedProperty, AssetManager
from pyhelios.primitives import Rotation
from pyhelios.scene import Scene
import math
import xml.etree.ElementTree as ET
from typing import Optional, List, Tuple
import _helios

class PlatformSettings(Validatable):
    def __init__(self, id: Optional[str] = "DEFAULT_TEMPLATE1_HELIOSCPP", x: Optional[float] = 0.0, y: Optional[float] = 0.0, z: Optional[float] = 0.0, is_on_ground: Optional[bool] = False,
                 position: Optional[List[float]] = None, is_yaw_angle_specified: Optional[bool] = False, yaw_angle: Optional[float] = 0.0,
                 is_stop_and_turn: Optional[bool] = True, is_smooth_turn: Optional[bool] = False, is_slowdown_enabled: Optional[bool] = True, speed_m_s: Optional[float] = 70.0) -> None:
        
        self._cpp_object = _helios.PlatformSettings()
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.is_on_ground = is_on_ground
        self.position = position or [0, 0, 0]
        self.is_yaw_angle_specified = is_yaw_angle_specified
        self.yaw_angle = yaw_angle
        self.is_stop_and_turn = is_stop_and_turn
        self.is_smooth_turn = is_smooth_turn
        self.is_slowdown_enabled = is_slowdown_enabled
        self.speed_m_s = speed_m_s
    
    id: Optional[str] = ValidatedCppManagedProperty("id")
    x: Optional[float] = ValidatedCppManagedProperty("x")
    y: Optional[float] = ValidatedCppManagedProperty("y")
    z: Optional[float] = ValidatedCppManagedProperty("z")
    is_on_ground: Optional[bool] = ValidatedCppManagedProperty("is_on_ground")
    position: Optional[List[float]] = ValidatedCppManagedProperty("position")
    is_yaw_angle_specified: Optional[bool] = ValidatedCppManagedProperty("is_yaw_angle_specified")
    yaw_angle: Optional[float] = ValidatedCppManagedProperty("yaw_angle")
    is_stop_and_turn: Optional[bool] = ValidatedCppManagedProperty("is_stop_and_turn")
    is_smooth_turn: Optional[bool] = ValidatedCppManagedProperty("is_smooth_turn")
    is_slowdown_enabled: Optional[bool] = ValidatedCppManagedProperty("is_slowdown_enabled")
    speed_m_s: Optional[float] = ValidatedCppManagedProperty("speed_m_s")
    
    @classmethod
    def from_xml_node(cls, node: ET.Element, template: Optional['PlatformSettings'] = None) -> 'PlatformSettings':   
        settings = cls.copy_template(template) if template else cls()
  
        settings.id = node.get('id', settings.id)
        settings.x = float(node.get('x', settings.x))
        settings.y = float(node.get('y', settings.y))
        settings.z = float(node.get('z', settings.z))
        settings.is_on_ground = bool(node.get('onGround', settings.is_on_ground))
        settings.is_stop_and_turn = bool(node.get('stopAndTurn', settings.is_stop_and_turn))
        settings.is_smooth_turn = bool(node.get('smoothTurn', settings.is_smooth_turn))
        settings.is_slowdown_enabled = bool(node.get('slowdownEnabled', settings.is_slowdown_enabled))
        settings.speed_m_s = float(node.get('movePerSec_m', settings.speed_m_s))
        yaw_at_departure_deg = node.get("yawAtDeparture_deg")
        if yaw_at_departure_deg is not None:
            settings.is_yaw_angle_specified = True
            settings.yaw_angle = math.radians(float(yaw_at_departure_deg))
        if settings.is_stop_and_turn and settings.is_smooth_turn:
            raise ValueError("Platform cannot be both stop-and-turn and smooth-turn")
        settings.position = [settings.x, settings.y, settings.z]
        return cls._validate(settings)

    @classmethod
    def copy_template(cls, template: 'PlatformSettings') -> 'PlatformSettings':
        """Create a copy of the template to be used"""

        return PlatformSettings(
            id=template.id,
            x=template.x,
            y=template.y,
            z=template.z,
            is_on_ground=template.is_on_ground,
            position=template.position,
            is_yaw_angle_specified=template.is_yaw_angle_specified,
            yaw_angle=template.yaw_angle,
            is_stop_and_turn=template.is_stop_and_turn,
            is_smooth_turn=template.is_smooth_turn,
            is_slowdown_enabled=template.is_slowdown_enabled,
            speed_m_s=template.speed_m_s
        )
    

class Platform(Validatable):
    def __init__(self, id: Optional[str] = '', platform_settings: Optional[PlatformSettings] = None, last_check_z: Optional[float] = 0.0, dmax: Optional[float] = 0.0, is_orientation_on_leg_init: Optional[bool] = False,
                 is_on_ground: Optional[bool] = False, is_stop_and_turn: Optional[bool] = True, settings_speed_m_s: Optional[float] = 70.0,
                 is_slowdown_enabled: Optional[bool] = False, is_smooth_turn: Optional[bool] = False, position: Optional[List[float]] = None, scene: Optional[Scene] = None) -> None:
        
        self._cpp_object = _helios.Platform()

        self.id = id
        self.last_check_z = last_check_z
        self.dmax = dmax
        self.is_orientation_on_leg_init = is_orientation_on_leg_init
        self.is_on_ground = is_on_ground
        self.is_stop_and_turn = is_stop_and_turn
        self.settings_speed_m_s = settings_speed_m_s
        self.is_slowdown_enabled = is_slowdown_enabled
        self.is_smooth_turn = is_smooth_turn
        self.position = position or [0, 0, 0]
        self.device_relative_position = [0.0, 0.0, 0.0]
        self.device_relative_attitude = Rotation(0.0, 0.0, 1.0, 0.0)
        self.attitude = Rotation(0.0, 0.0, 1.0, 0.0)
        self.scene = scene
        self.target_waypoint = [0.0, 0.0, 0.0]
        self.next_waypoint = [0.0, 0.0, 0.0]
        self.origin_waypoint = [0.0, 0.0, 0.0]
        
        self.absolute_mount_position = [0.0, 0.0, 0.0]
        self.absolute_mount_attitude = Rotation(0.0, 0.0, 1.0, 0.0)
        self.cached_dir_current = [0.0, 0.0, 0.0]
        self.cached_dir_current_xy = [0.0, 0.0, 0.0]

        self.cached_vector_to_target = [0.0, 0.0, 0.0]
        self.cached_vector_to_target_xy = [0.0, 0.0, 0.0]
        self.cached_origin_to_target_dir_xy = [0.0, 0.0, 0.0]
        self.cached_target_to_next_dir_xy = [0.0, 0.0, 0.0]
        self.cached_distance_to_target_xy = 0.0

        self.cached_current_angle_xy
        self.cached_end_target_angle_xy 
        self.cached_origin_to_target_angle_xy 
        self.cached_target_to_next_angle_xy 
    

    last_check_z: Optional[float] = ValidatedCppManagedProperty("last_check_z")
    dmax: Optional[float] = ValidatedCppManagedProperty("dmax")
    is_orientation_on_leg_init: Optional[bool] = ValidatedCppManagedProperty("is_orientation_on_leg_init")
    is_on_ground: Optional[bool] = ValidatedCppManagedProperty("is_on_ground")
    is_stop_and_turn: Optional[bool] = ValidatedCppManagedProperty("is_stop_and_turn")
    settings_speed_m_s: Optional[float] = ValidatedCppManagedProperty("settings_speed_m_s")
    is_slowdown_enabled: Optional[bool] = ValidatedCppManagedProperty("is_slowdown_enabled")
    is_smooth_turn: Optional[bool] = ValidatedCppManagedProperty("is_smooth_turn")
    position: Optional[List[float]] = ValidatedCppManagedProperty("position")
    scene: Optional[Scene] = ValidatedCppManagedProperty("scene")
    device_relative_position: Optional[List[float]] = ValidatedCppManagedProperty("device_relative_position")
    device_relative_attitude: Rotation = ValidatedCppManagedProperty("device_relative_attitude")
    absolute_mount_position: List[float] = ValidatedCppManagedProperty("absolute_mount_position")
    absolute_mount_attitude: Rotation = ValidatedCppManagedProperty("absolute_mount_attitude")
    cached_dir_current: List[float] = ValidatedCppManagedProperty("cached_dir_current")
    cached_dir_current_xy: List[float] = ValidatedCppManagedProperty("cached_dir_current_xy")
    cached_vector_to_target: List[float] = ValidatedCppManagedProperty("cached_vector_to_target")
    cached_vector_to_target_xy: List[float] = ValidatedCppManagedProperty("cached_vector_to_target_xy")
    cached_origin_to_target_dir_xy: List[float] = ValidatedCppManagedProperty("cached_origin_to_target_dir_xy")
    cached_target_to_next_dir_xy: List[float] = ValidatedCppManagedProperty("cached_target_to_next_dir_xy")
    cached_distance_to_target_xy: float = ValidatedCppManagedProperty("cached_distance_to_target_xy")
    cached_current_angle_xy: float = ValidatedCppManagedProperty("cached_current_angle_xy")
    cached_end_target_angle_xy: float = ValidatedCppManagedProperty("cached_end_target_angle_xy")
    cached_origin_to_target_angle_xy: float = ValidatedCppManagedProperty("cached_origin_to_target_angle_xy")
    cached_target_to_next_angle_xy: float = ValidatedCppManagedProperty("cached_target_to_next_angle_xy")

    @classmethod
    def from_xml(cls, filename: str, id: Optional[str] = None) -> 'Platform':
        file_path = AssetManager().find_file_by_name(filename, auto_add=True)
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        platform_element = root.find(f".//platform[@id='{id}']")
        if platform_element is None:
            raise ValueError(f"No platform found with id: {id}")
        
        platform = cls(id=id)
        platform = cls._initialize_platform_from_xml(platform, platform_element)
        
        return cls._validate(platform)

    @classmethod
    def _initialize_platform_from_xml(cls, platform: 'Platform', platform_element: ET.Element) -> 'Platform':
        platform_type = platform_element.get('type').lower()
        
        # Select platform subclass based on type
        if platform_type == "groundvehicle":
            platform = GroundVehiclePlatform(id=platform.id)
        elif platform_type == "linearpath":
            platform = LinearPathPlatform(id=platform.id)
        elif platform_type == "multicopter":
            platform = HelicopterPlatform(id=platform.id)
        
        # Apply type-specific settings
        cls._apply_platform_specific_settings(platform, platform_element)
        
        # Parse scanner mount and other general settings
        platform.device_relative_position, platform.device_relative_attitude = cls._parse_scanner_mount(platform_element)
        
        return platform

    @classmethod
    def _apply_platform_specific_settings(cls, platform: 'Platform', platform_element: ET.Element):
        if isinstance(platform, SimplePhysicsPlatform):
            platform.drag_magnitude = float(platform_element.get('drag', 1.0))
        if isinstance(platform, HelicopterPlatform):
            platform._apply_helicopter_specific_settings(platform_element)
        
    def _apply_helicopter_specific_settings(platform: 'HelicopterPlatform', platform_element: ET.Element):
        platform.speedup_magnitude = float(platform_element.get('speedup_magnitude', 2.0))
        platform.slowdown_magnitude = float(platform_element.get('slowdown_magnitude', 2.0))
        platform.max_engine_force_xy = float(platform_element.get('engine_max_force', 0.1))
        platform.base_pitch_angle = math.radians(float(platform_element.get('base_pitch_deg', -5.0)))
        platform.pitch_speed = math.radians(float(platform_element.get('pitch_speed_deg', 85.94)))
        platform.roll_speed = math.radians(float(platform_element.get('roll_speed_deg', 28.65)))
        platform.yaw_speed = math.radians(float(platform_element.get('yaw_speed_deg', 85.94)))
        platform.max_pitch_offset = math.radians(float(platform_element.get('max_pitch_offset_deg', 35.0)))
        platform.max_roll_offset = math.radians(float(platform_element.get('max_roll_offset_deg', 25.0)))
        platform.max_pitch = platform.base_pitch_angle + platform.max_pitch_offset
        platform.min_pitch = platform.base_pitch_angle - platform.max_pitch_offset
        platform.slowdown_distance_xy = float(platform_element.get('slowdown_distance_xy', 5.0))
    
    def _parse_scanner_mount(platform_element) -> Tuple[List[float], Rotation]:
        scanner_mount = platform_element.find('scannerMount')
        device_relative_position = [0.0, 0.0, 0.0]

        if scanner_mount is not None:
            x = float(scanner_mount.get('x', '0.0'))
            y = float(scanner_mount.get('y', '0.0'))
            z = float(scanner_mount.get('z', '0.0'))
            device_relative_position = [x, y, z]

        # Parse the rotation
        device_relative_attitude = Rotation.from_xml_node(scanner_mount)
       
        return device_relative_position, device_relative_attitude
    
    def retrieve_current_settings(self) -> PlatformSettings:
        current_settings = PlatformSettings()
        current_settings.speed_m_s = self.settings_speed_m_s
        current_settings.is_on_ground = self.is_on_ground
        current_settings.position = self.position
        return current_settings
    
    def update_static_cache(self):
        self.cached_origin_to_target_dir_xy = self._normalize([
            self.target_waypoint[0] - self.origin_waypoint[0],
            self.target_waypoint[1] - self.origin_waypoint[1],
            0
        ])
        
        self.cached_target_to_next_dir_xy = self._normalize([
            self.next_waypoint[0] - self.target_waypoint[0],
            self.next_waypoint[1] - self.target_waypoint[1],
            0
        ])

        self.cached_end_target_angle_xy = self._angle_between_vectors(
            self.cached_origin_to_target_dir_xy, 
            self.cached_target_to_next_dir_xy
        )

        # Convert directions to angles
        self.cached_origin_to_target_angle_xy = self._direction_to_angle_xy(
            self.cached_origin_to_target_dir_xy
        )

        self.cached_target_to_next_angle_xy = self._direction_to_angle_xy(
            self.cached_target_to_next_dir_xy
        )

        # Update dynamic cache
        self.update_dynamic_cache()

    def update_dynamic_cache(self):
        self.cached_vector_to_target = [
            self.target_waypoint[i] - self.position[i] for i in range(3)
        ]
        self.cached_dir_current = self.attitude.apply_vector_rotation([0,1,0])
        # Projected Vector in XY-plane
        self.cached_vector_to_target_xy = [
            self.cached_vector_to_target[0],
            self.cached_vector_to_target[1],
            0
        ]

        # Distance to Target in XY-plane
        self.cached_distance_to_target_xy = self._l2_norm(
            self.cached_vector_to_target_xy
        )

        # Current Direction (normalized)
        self.cached_dir_current_xy = self._normalize([
            self.cached_dir_current[0],
            self.cached_dir_current[1],
            0
        ])

        # Angle between current direction and Target-to-Next
        self.cached_current_angle_xy = self._angle_between_vectors(
            self.cached_dir_current_xy,
            self.cached_target_to_next_dir_xy
        )

    def _normalize(self, vector):
        length = math.sqrt(sum(comp ** 2 for comp in vector))
        if length == 0:
            return [0, 0, 0]
        return [comp / length for comp in vector]
    
    def _angle_between_vectors(self, v1, v2):
        # Compute dot product
        dot_product = sum(v1[i] * v2[i] for i in range(3))
        # Clamp to avoid precision errors
        dot_product = max(-1.0, min(1.0, dot_product))
        # Compute angle (in radians)
        return math.acos(dot_product)

    def _direction_to_angle_xy(self, vector):
        # Angle in radians from X-axis
        return math.atan2(vector[1], vector[0])

    def _l2_norm(self, vector):
        # Compute Euclidean norm (L2 norm)
        return math.sqrt(sum(comp ** 2 for comp in vector))

    def prepare_simulation(self, pulse_frequency: int):
        self.cached_absolute_attitude = self.attitude.apply_rotation(self.device_relative_attitude)
        self.update_static_cache()

        
class MovingPlatform(Platform):
    def __init__(self, id: str | None = '', platform_settings: PlatformSettings | None = None, velocity: Optional[List[float]] = [0, 0, 0],  last_check_z: float | None = 0, dmax: float | None = 0, is_orientation_on_leg_init: bool | None = False, is_on_ground: bool | None = False, is_stop_and_turn: bool | None = True, settings_speed_m_s: float | None = 70, is_slowdown_enabled: bool | None = False, is_smooth_turn: bool | None = False) -> None:
        super().__init__()
        self._cpp_object = _helios.MovingPlatform()
        self.velocity = velocity
        self.id = id

    velocity: List[float] = ValidatedCppManagedProperty("velocity")

class SimplePhysicsPlatform(MovingPlatform):
    def __init__(self, id: str | None = '', platform_settings: PlatformSettings | None = None, drag_magnitude: Optional[float] = 1.0, last_check_z: float | None = 0, dmax: float | None = 0, is_orientation_on_leg_init: bool | None = False, is_on_ground: bool | None = False, is_stop_and_turn: bool | None = True, settings_speed_m_s: float | None = 70, is_slowdown_enabled: bool | None = False, is_smooth_turn: bool | None = False) -> None:
        super().__init__()
        self._cpp_object = _helios.SimplePhysicsPlatform()
        self.drag_magnitude = drag_magnitude
        self.id = id
        self.speed_step_magnitude = 0.0

    drag_magnitude: float = ValidatedCppManagedProperty("drag_magnitude")

    def prepare_simulation(self, pulse_frequency: int):
        self.speed_step_magnitude = self.drag_magnitude / float(pulse_frequency)

class GroundVehiclePlatform(SimplePhysicsPlatform):
    def __init__(self, id: str | None = '', platform_settings: PlatformSettings | None = None, last_check_z: float | None = 0, dmax: float | None = 0, is_orientation_on_leg_init: bool | None = False, is_on_ground: bool | None = False, is_stop_and_turn: bool | None = True, settings_speed_m_s: float | None = 70, is_slowdown_enabled: bool | None = False, is_smooth_turn: bool | None = False) -> None:
        super().__init__()
        self._cpp_object = _helios.GroundVehiclePlatform()
        self.id = id

    def prepare_simulation(self, pulse_frequency: int):
        SimplePhysicsPlatform.prepare_simulation(self, pulse_frequency)
    

class LinearPathPlatform(MovingPlatform):
    def __init__(self, id: str | None = '', platform_settings: PlatformSettings | None = None, last_check_z: float | None = 0, dmax: float | None = 0, is_orientation_on_leg_init: bool | None = False, is_on_ground: bool | None = False, is_stop_and_turn: bool | None = True, settings_speed_m_s: float | None = 70, is_slowdown_enabled: bool | None = False, is_smooth_turn: bool | None = False) -> None:
        super().__init__()
        self._cpp_object = _helios.LinearPathPlatform()
        self.id = id

class HelicopterPlatform(SimplePhysicsPlatform):
    def __init__(self, id: str | None = '', platform_settings: PlatformSettings | None = None, drag_magnitude: float | None = 1.0, last_check_z: float | None = 0, dmax: float | None = 0, is_orientation_on_leg_init: bool | None = False, is_on_ground: bool | None = False, is_stop_and_turn: bool | None = True, settings_speed_m_s: float | None = 70, is_slowdown_enabled: bool | None = False, is_smooth_turn: bool | None = False) -> None:
                
        super().__init__()
        self._cpp_object = _helios.HelicopterPlatform()
        self.id = id
        self.slowdown_distance_xy = 5.0
        self.slowdown_magnitude = 2.0
        self.speedup_magnitude = 2.0
        self.max_engine_force_xy = 0.1
        self.heading_rad = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.last_sign = 1.0
        self.base_pitch_angle = -0.087
        self.pitch_speed = 1.5
        self.roll_speed = 0.5
        self.yaw_speed = 1.5
        self.max_pitch_offset = 0.61
        self.max_roll_offset = 0.45
        self.max_pitch = self.base_pitch_angle + self.max_pitch_offset
        self.min_pitch = self.base_pitch_angle - self.max_pitch_offset
        self.pitch_step_magnitude = 0.0
        self.roll_step_magnitude = 0.0
        self.yaw_step_magnitude = 0.0
        self.slowdown_factor = 0.0
        self.speedup_factor = 0.0
    
    slowdown_distance_xy: float = ValidatedCppManagedProperty("slowdown_distance_xy")
    slowdown_magnitude: float = ValidatedCppManagedProperty("slowdown_magnitude")
    speedup_magnitude: float = ValidatedCppManagedProperty("speedup_magnitude")
    max_engine_force_xy: float = ValidatedCppManagedProperty("max_engine_force_xy")
    heading_rad: float = ValidatedCppManagedProperty("heading_rad")
    roll: float = ValidatedCppManagedProperty("roll")
    pitch: float = ValidatedCppManagedProperty("pitch")
    last_sign: float = ValidatedCppManagedProperty("last_sign")
    base_pitch_angle: float = ValidatedCppManagedProperty("base_pitch_angle")
    pitch_speed: float = ValidatedCppManagedProperty("pitch_speed")
    roll_speed: float = ValidatedCppManagedProperty("roll_speed")
    yaw_speed: float = ValidatedCppManagedProperty("yaw_speed")
    max_pitch_offset: float = ValidatedCppManagedProperty("max_pitch_offset")
    max_roll_offset: float = ValidatedCppManagedProperty("max_roll_offset")
    max_pitch: float = ValidatedCppManagedProperty("max_pitch")
    min_pitch: float = ValidatedCppManagedProperty("min_pitch")

    def prepare_simulation(self, pulse_frequency: int):
        self.pitch_step_magnitude = self.pitch_speed / pulse_frequency
        self.roll_step_magnitude = self.roll_speed / pulse_frequency
        self.yaw_step_magnitude = self.yaw_speed / pulse_frequency
        self.slowdown_factor = 1.0 - self.slowdown_magnitude /pulse_frequency
        self.speedup_factor = 1.0 + self.speedup_magnitude / pulse_frequency
        SimplePhysicsPlatform.prepare_simulation(pulse_frequency)
        Platform.prepare_simulation(self, pulse_frequency)
        

SR22 = Platform.from_xml("data/platforms.xml", id="sr22")

QUADCOPTER = Platform.from_xml("data/platforms.xml", id="quadcopter")

COPTER_LIN_PATH = Platform.from_xml("data/platforms.xml", id="copter_linearpath")

TRACTOR = Platform.from_xml("data/platforms.xml", id="tractor")

TRACTOR_LEFT_SIDE = Platform.from_xml("data/platforms.xml", id="tractor_leftside")

VEHILE_LIN_PATH = Platform.from_xml("data/platforms.xml", id="vehicle_linearpath")

VMX_450_CAR_LEFT = Platform.from_xml("data/platforms.xml", id="vmx-450-car-left")

VMX_450_CAR_RIGHT = Platform.from_xml("data/platforms.xml", id="vmx-450-car-right")

VMQ_1HA_CAR_0 = Platform.from_xml("data/platforms.xml", id="vmq-1ha-car-0")

SIMPLE_LIN_PATH = Platform.from_xml("data/platforms.xml", id="simple_linearpath")

TRIPOD = Platform.from_xml("data/platforms.xml", id="tripod")
