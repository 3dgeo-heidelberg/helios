# from typing import Optional, List
# from pydantic import BaseModel, Field
# import numpy as np
# import threading
# import json
# from xml.etree.ElementTree import Element, SubElement, tostring, ElementTree

# from helios.platform import Platform
# from helios.platformsettings import PlatformSettings
# from helios.scannersettings import ScannerSettings


# class Scanner(BaseModel):
#     name: str = Field(default="SCANNER-ID")
#     write_waveform: bool = Field(default=False)
#     write_pulse: bool = Field(default=False)
#     calc_echowidth: bool = Field(default=False)
#     full_wave_noise: bool = Field(default=False)
#     platform_noise_disabled: bool = Field(default=False)
#     fixed_incidence_angle: bool = Field(default=False)
#     pulse_frequency: int = Field(default=0)
#     is_state_active: bool = Field(default=True)
#     trajectory_time_interval: float = Field(default=0.0)
#     last_trajectory_time: float = Field(default=0.0)

#     platform: Optional[Platform] = None
#     scanning_pulse_process: Optional[ScanningPulseProcess] = None
#     fms: Optional[FMSFacade] = None 
#     output_paths: Optional[List[str]] = None
#     all_measurements: Optional[List[Measurement]] = None
#     all_trajectories: Optional[List[Trajectory]] = None
#     all_measurements_mutex: Optional[threading.Lock] = None
#     cycle_measurements: Optional[List[Measurement]] = None
#     cycle_trajectories: Optional[List[Trajectory]] = None
#     cycle_measurements_mutex: Optional[threading.Lock] = None
#     rand_gen1: Optional[RandomnessGenerator] = None
#     rand_gen2: Optional[RandomnessGenerator] = None
#     intersection_handling_noise_source: Optional[UniformNoiseSource] = None


#     @property
#     def current_settings(self) -> ScannerSettings:
#         current_settings = ScannerSettings(
#             name=f"{self.name}_settings",
#             pulse_frequency=self.pulse_frequency,
#             is_active=self.is_state_active,
#             beam_divergence_angle=self.beam_divergence_angle(0),
#             trajectory_time_interval=self.trajectory_time_interval / 1e9,
#             head_rotation=self.get_scanner_head(0).get_rotate_start(),
#             rotation_start_angle=self.get_scanner_head(0).get_rotate_current(),
#             rotation_stop_angle=self.get_scanner_head(0).get_rotate_stop(),
#             scan_angle=self.get_beam_deflector(0).cfg_setting_scanAngle_rad,
#             scan_frequency=self.get_beam_deflector(0).cfg_setting_scanFreq_Hz,
#             min_vertical_angle=self.get_beam_deflector(0).cfg_setting_minVerticalAngle_rad,
#             max_vertical_angle=self.get_beam_deflector(0).cfg_setting_maxVerticalAngle_rad
#         )
#         return current_settings

#     def get_scanner_by_name(self, name: str) -> Optional['Scanner']:
#         if self.name == name:
#             return self
#         return None

#     def to_file(self, filename: str, format: str = 'json') -> None:
#         if format == 'json':
#             with open(filename, 'w') as file:
#                 json.dump(self.dict(), file)
#         elif format == 'xml':
#             with open(filename, 'w') as file:
#                 file.write(self.to_xml())
#         else:
#             raise ValueError(f"Unsupported format: {format}")

#     def to_xml(self) -> str:
#         root = Element('Scanner')
#         for field, value in self.dict().items():
#             child = SubElement(root, field)
#             child.text = str(value)
#         return tostring(root, encoding='unicode')
