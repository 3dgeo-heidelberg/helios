from pyhelios.utils import Validatable, ValidatedCppManagedProperty
from pyhelios.primitives import Rotation, Primitive, AABB
from pyhelios.scene import ScenePart, Scene

from typing import Optional, List, Tuple, Annotated, Type, Any
import _helios


class PlatformSettings(Validatable):
    def __init__(self, id: Optional[str] = "", x: Optional[float] = 0.0, y: Optional[float] = 0.0, z: Optional[float] = 0.0, is_on_ground: Optional[bool] = False,
                 position: Optional[List[float]] = [0, 0, 0], is_yaw_angle_specified: Optional[bool] = False, yaw_angle: Optional[float] = 0.0,
                 is_stop_and_turn: Optional[bool] = True, is_smooth_turn: Optional[bool] = False, speed_m_s: Optional[float] = 70.0) -> None:
        
        self._cpp_object = _helios.PlatformSettings()
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.is_on_ground = is_on_ground
        self.position = position
        self.is_yaw_angle_specified = is_yaw_angle_specified
        self.yaw_angle = yaw_angle
        self.is_stop_and_turn = is_stop_and_turn
        self.is_smooth_turn = is_smooth_turn
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
    speed_m_s: Optional[float] = ValidatedCppManagedProperty("speed_m_s")


class Platform(Validatable):
    def __init__(self, platform_settings: Optional[PlatformSettings] = None, last_check_z: Optional[float] = 0.0, dmax: Optional[float] = 0.0, is_orientation_on_leg_init: Optional[bool] = False,
                 is_on_ground: Optional[bool] = False, is_stop_and_turn: Optional[bool] = True, settings_speed_m_s: Optional[float] = 70.0,
                 is_slowdown_enabled: Optional[bool] = False, is_smooth_turn: Optional[bool] = False) -> None:
        
        self._cpp_object = _helios.Platform()

        self.last_check_z = last_check_z
        self.dmax = dmax
        self.is_orientation_on_leg_init = is_orientation_on_leg_init
        self.is_on_ground = is_on_ground
        self.is_stop_and_turn = is_stop_and_turn
        self.settings_speed_m_s = settings_speed_m_s
        self.is_slowdown_enabled = is_slowdown_enabled
        self.is_smooth_turn = is_smooth_turn

    

    last_check_z: Optional[float] = ValidatedCppManagedProperty("last_check_z")
    dmax: Optional[float] = ValidatedCppManagedProperty("dmax")
    is_orientation_on_leg_init: Optional[bool] = ValidatedCppManagedProperty("is_orientation_on_leg_init")
    is_on_ground: Optional[bool] = ValidatedCppManagedProperty("is_on_ground")
    is_stop_and_turn: Optional[bool] = ValidatedCppManagedProperty("is_stop_and_turn")
    settings_speed_m_s: Optional[float] = ValidatedCppManagedProperty("settings_speed_m_s")
    is_slowdown_enabled: Optional[bool] = ValidatedCppManagedProperty("is_slowdown_enabled")
    is_smooth_turn: Optional[bool] = ValidatedCppManagedProperty("is_smooth_turn")


