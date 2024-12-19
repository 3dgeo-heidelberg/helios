
# from typing import Optional
# from pydantic import BaseModel
# import numpy as np
# import math
# from helios.scene import Scene
# from helios.platformsettings import PlatformSettings

# class Platform(BaseModel):
#     device_relative_position: np.ndarray = np.array([0.0, 0.0, 0.0])
#     device_relative_attitude: Rotation = Rotation()
#     scene: Optional[Scene] = None

#     position_x_noise_source: Optional[NoiseSource] = None
#     position_y_noise_source: Optional[NoiseSource] = None
#     position_z_noise_source: Optional[NoiseSource] = None
#     attitude_x_noise_source: Optional[NoiseSource] = None
#     attitude_y_noise_source: Optional[NoiseSource] = None
#     attitude_z_noise_source: Optional[NoiseSource] = None

#     settings_speed_m_s: float = 0.0
#     _origin_waypoint: np.ndarray = np.array([0.0, 0.0, 0.0])
#     _target_waypoint: np.ndarray = np.array([0.0, 0.0, 0.0])
#     _next_waypoint: np.ndarray = np.array([0.0, 0.0, 0.0])
#     is_on_ground: bool = False
#     is_stop_and_turn: bool = True
#     is_smooth_turn: bool = False
#     is_slowdown_enabled: bool = True

#     _position: np.ndarray = np.array([0.0, 0.0, 0.0])
#     _attitude: Rotation = Rotation()
#     #caches
#     cached_absolute_mount_position: np.ndarray = np.array([0.0, 0.0, 0.0])
#     cached_absolute_mount_attitude: Rotation = Rotation()
#     cached_dir_current: np.ndarray = np.array([0.0, 0.0, 0.0])
#     cached_dir_current_xy: np.ndarray = np.array([0.0, 0.0, 0.0])
#     cached_array_to_target: np.ndarray = np.array([0.0, 0.0, 0.0])
#     cached_array_to_target_xy: np.ndarray = np.array([0.0, 0.0, 0.0])
#     cached_distance_to_target_xy: float = 0.0
#     cached_origin_to_target_dir_xy: np.ndarray = np.array([0.0, 0.0, 0.0])
#     cached_target_to_next_dir_xy: np.ndarray = np.array([0.0, 0.0, 0.0])
#     cached_end_target_angle_xy: float = 0.0
#     cached_current_angle_xy: float = 0.0
#     cached_origin_to_target_angle_xy: float = 0.0
#     cached_target_to_next_angle_xy: float = 0.0


#     #Проверить нужны ли эти декораторы во всех файлах!!!!!!
#     @property
#     def absolute_mount_attitude(self) -> Rotation:
#         return self.cached_absolute_mount_attitude

#     @property
#     def absolute_mount_position(self) -> np.ndarray:
#         return self.cached_absolute_mount_position
    
#     @property
#     def attitude(self) -> Rotation:
#         return self._attitude

#     @attitude.setter
#     def attitude(self, value: Rotation):
#         self._attitude = value
#         self.cached_absolute_mount_attitude = self._attitude.Rotation.apply_to(self.device_relative_attitude)

#     @property
#     def position(self) -> np.ndarray:
#         return self._position
    
#     @position.setter
#     def position(self, value: np.ndarray):
#         self._position = value
#         self.cached_absolute_mount_position = self._position + self.device_relative_position
#         self.update_dynamic_cache()

#     @property
#     def origin_waypoint(self) -> np.ndarray:
#         return self._origin_waypoint
    
#     @origin_waypoint.setter
#     def origin_waypoint(self, value: np.ndarray):
#         self._origin_waypoint = value
#         self.update_static_cache()
    
#     @property
#     def target_waypoint(self) -> np.ndarray:
#         return self._target_waypoint
    
#     @target_waypoint.setter
#     def target_waypoint(self, value: np.ndarray):
#         self._target_waypoint = value
#         self.update_static_cache()

#     @property
#     def next_waypoint(self) -> np.ndarray:
#         return self._next_waypoint
    
#     @next_waypoint.setter
#     def next_waypoint(self, value: np.ndarray):
#         self._next_waypoint = value
#         self.update_static_cache()

#     @property
#     def current_settings(self) -> PlatformSettings:
#         current_settings = PlatformSettings()
#         current_settings.speed_m_s = self.settings_speed_m_s
#         current_settings.is_on_ground = self.is_on_ground
#         current_settings.position(self.position)
#         return self.current_settings
    
#     def apply_settings(self, settings: PlatformSettings) -> None:
#         self.settings_speed_m_s = settings.speed_m_s
#         self.is_on_ground = settings.is_on_ground
#         self.position = settings.position
    
#     @property
#     def current_direction(self) -> np.ndarray:
#         return self.cached_current_direction
    
#     @property
#     def is_interpolated(self) -> bool:
#         return self.cached_is_interpolating

        
#     def update_static_cache(self) -> None:
#         self.cached_origin_to_target_dir_xy = np.array([
#             self.target_waypoint[0] - self.origin_waypoint[0],
#             self.target_waypoint[1] - self.origin_waypoint[1],
#             0.0
#         ])
#         self.cached_target_to_next_dir_xy = np.array([
#             self.next_waypoint[0] - self.target_waypoint[0],
#             self.next_waypoint[1] - self.target_waypoint[1],
#             0.0
#         ])
#         self.cached_end_target_angle_xy = np.arccos(
#             np.dot(self.cached_origin_to_target_dir_xy, self.cached_target_to_next_dir_xy) /
#             (np.linalg.norm(self.cached_origin_to_target_dir_xy) * np.linalg.norm(self.cached_target_to_next_dir_xy))
#         )
#         if np.isnan(self.cached_end_target_angle_xy):
#             self.cached_end_target_angle_xy = 0.0
#         self.cached_origin_to_target_angle_xy = self.direction_to_angle_xy(self.cached_origin_to_target_dir_xy, True) # this function should be moved to a separate class
#         self.cached_target_to_next_angle_xy = self.direction_to_angle_xy(self.cached_target_to_next_dir_xy, True) # this function should be moved to a separate class
#         self.update_dynamic_cache()

#     def update_dynamic_cache(self) -> None:
#         self.cached_array_to_target = self.target_waypoint - self.position
#         self.cached_array_to_target_xy = np.array([self.cached_array_to_target[0], self.cached_array_to_target[1], 0.0])
#         self.cached_distance_to_target_xy = np.linalg.norm(self.cached_array_to_target_xy)
#         self.cached_dir_current = self.current_direction()
#         self.cached_dir_current_xy = np.array([self.cached_dir_current[0], self.cached_dir_current[1], 0.0])
#         self.cached_current_angle_xy = np.arccos(
#             np.dot(self.cached_dir_current_xy, self.cached_target_to_next_dir_xy) /
#             (np.linalg.norm(self.cached_dir_current_xy) * np.linalg.norm(self.cached_target_to_next_dir_xy))
#         )
#         if np.isnan(self.cached_current_angle_xy):
#             self.cached_current_angle_xy = 0.0


#     def direction_to_angle_xy(self, direction: np.ndarray, is_normalized: bool) -> float: # this function should be moved to a separate class
#         angle = np.arctan2(direction[0], direction[1])
#         if is_normalized and angle < 0.0:
#             angle += np.pi * 2
#         return angle
    
#     def get_by_name(self, name: str) -> Optional['Platform']:
#         if self.name == name:
#             return self
#         return None