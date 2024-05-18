# from pydantic import BaseModel
# from typing import Optional, Union, List
# from helios.scanner import Scanner
# from helios.leg import Leg 
# import math
# from helios.scene import Scene
# from helios.scannersettings import ScannerSettings
# from helios.platform import Platform
# from helios.platformsettings import PlatformSettings

# import xml.etree.ElementTree as ET
# import numpy as np

# class Survey(BaseModel):
#     name: str = "Unnamed Survey Playback"
#     num_runs: int = -1
#     scanner: Optional[Scanner] = None
#     sim_speed_factor: float = 1.0
#     legs: List[Leg] = []
#     length: float = 0.0

#     scene: Optional[Scene] = None
#     scanner_settings: Optional[ScannerSettings] = None
#     platform: Optional[Platform] = None
#     platform_settings: Optional[PlatformSettings] = None
#     trajectory_time_interval: float = 0.0
#     trajectory: Optional[Trajectory] = None

#     fullwave_settings: Optional[dict] = None
#     run_threads: int = 0
#     save_config: bool = False
#     output_format: Optional[str] = None
#     output_path: Optional[str] = None
#     write_waveform: bool = False
#     calc_echo_width: bool = False
#     gps_start_time: Optional[str] = None
#     on_finished_callback: Optional[callable] = None
#     on_progress_callback: Optional[callable] = None
#     is_running_flag: bool = False
    
#     #instead of propose functionality we first would create a Leg object, and then add it to survey legs 
#     def add_leg(self, leg: Leg):
#         if self._trajectory is not None:
#             raise ValueError("Adding legs to a survey with an existing trajectory is not supported. "
#                              "Set the trajectory to None first explicitly.")
#         else:
#             if leg not in self.legs:
#                 self.legs.append(leg)

#     def add_legs(self, horizontal_fov: float, scan_pattern: Union[ScannerSettings, List[ScannerSettings]], pos: List[List[float]]):
#         legs = create_legs(horizontal_fov, scan_pattern, pos)
#         self.legs.extend(legs)

#     def remove_leg(self, leg_index: int):
#         try:
#             del self.legs[leg_index]
#         except IndexError:
#              raise ValueError(f"Leg index {leg_index} is out of range!")

#     def calculate_length(self): 
#         self._length = 0
#         for i in range(len(self.legs) - 1):
#             leg_distance = math.dist(self.legs[i].position, self.legs[i + 1].position)
#             self.legs[i].set_length(leg_distance)
#             self._length += self.legs[i].get_length()

    
#     def fullwave(self, **kwargs):
#         self._fullwave_settings = kwargs

#     def output(self):
#         pass

    
#     def preview(self):
#         ''' # some kind of visualization? e.g., 2D map preview, 3D preview with scene and markers for SPs/waypoints, etc.
#             # survey.preview("2D")
#             # survey.preview("3D")  # similar to scene.show(), but with marker (e.g. diamonds) for leg positions;
#             # for static platforms also show movement direction and orientation of platform
#         '''
#         pass

#     def save_report(self, filename: str):
#         '''
#         Alberto:
#         Also as this is a research software, I think writing some tables
#         and/or CSV files with the characteristics of the survey would be nice.
#         I mean things such as the number of primitives, vertices, the simulation
#         time (not the execution time), the volume of the bounding box, the
#         depth for each KDTree in the KDGrove, etc.
#         Hannah: Yes, so something like a "report" or "summary", see below
#         '''
#         pass

#     def set_trajectory(self, trajectory: Union[Trajectory, str]) -> None:
#         if self.trajectory is not None:
#             raise ValueError("Overwriting a trajectory in a survey is not supported. Set it to None first explicitly.")
#         if isinstance(trajectory, str):
#             self.trajectory = Trajectory.from_file(trajectory)
#         else:
#             self.trajectory = trajectory



#     def run(self,filename: str = None):
#         #if filename is none, then run the survey without saving the output):
#         self.is_running = True
#         callback_counter = 0
#         mpoints = []
#         tpoints = []

#         while self.is_running:
#             if len(mpoints) > 0:
#                 # update points in visualization
#                 scene_vis.add_sim_points(mpoints)
#                 scene_vis.add_traj_points(tpoints)

#                 # + apply colour, refresh gui, etc.
#             time.sleep(0.1)

#     def stop(self):
#         self.is_running = False
    
    
#     def is_running(self):
#         return self.is_running


# def create_legs(horizontal_fov: float, scan_pattern: Union[ScannerSettings, List[ScannerSettings]], pos: List[List[float]]) -> List[Leg]:
#     legs = []
#     if isinstance(scan_pattern, list):
#         for p, s in zip(pos, scan_pattern):
#             legs.append(Leg(horizontal_fov=horizontal_fov, scan_pattern=s, position=p))
#     else:
#         for p in pos:
#             legs.append(Leg(horizontal_fov=horizontal_fov, scan_pattern=scan_pattern, position=p))
#     return legs