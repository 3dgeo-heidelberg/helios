# from pydantic import BaseModel, Field
# from typing import Optional, Set
# import numpy as np
# import json


# class ScannerSettings(BaseModel):
#     name: str = Field("#nullid#", description="The name of the scanner settings.")
#     _basic_template: Optional["ScannerSettings"] = Field(None, description="A template to base settings off of.")
#     is_active: Optional[bool] = Field(True, description="Whether the scanner is active.")
#     head_rotation: Optional[float] = Field(0.0, description="The rotation angle of the scanner head.")
#     rotation_start_angle: Optional[float] = Field(0.0, description="The starting angle for rotation.")
#     rotation_stop_angle: Optional[float] = Field(0.0, description="The stopping angle for rotation.")
#     pulse_frequency: Optional[int] = Field(0, description="The frequency of pulses emitted by the scanner.")
#     scan_angle: Optional[float] = Field(0.0, description="The scanning angle of the scanner.")
#     min_vertical_angle: Optional[float] = Field(None, description="Minimum vertical scanning angle.")
#     max_vertical_angle: Optional[float] = Field(None, description="Maximum vertical scanning angle.")
#     scan_frequency: Optional[float] = Field(0.0, description="The frequency at which the scanner operates.")
#     beam_divergence_angle: Optional[float] = Field(0.003, description="The divergence angle of the scanner beam.")
#     trajectory_time_interval: Optional[float] = Field(0.0, description="Time interval for the trajectory.")
#     vertical_resolution: Optional[float] = Field(0.0, description="Vertical resolution of the scanner.")
#     horizontal_resolution: Optional[float] = Field(0.0, description="Horizontal resolution of the scanner.")
#     vertical_fov: Optional[int] = Field(0, description="Vertical field of view.")
#     horizontal_fov: Optional[int] = Field(0, description="Horizontal field of view.")
#     optics: Optional[str] = Field(None, description="Type of optics used.")
#     accuracy: Optional[float] = Field(None, description="Accuracy of the scanner.")
#     beam_divergence_radius: Optional[float] = Field(None, description="Beam divergence radius.")
#     max_nor: Optional[float] = Field(None, description="Maximum number of returns.")
#     max_scan_angle: Optional[float] = Field(None, description="Maximum scan angle.")
#     max_effective_scan_angle: Optional[float] = Field(None, description="Maximum effective scan angle.")
#     min_scanner_frequency: Optional[float] = Field(None, description="Minimum scanner frequency.")
#     max_scanner_frequency: Optional[float] = Field(None, description="Maximum scanner frequency.")
#     beam_sample_quality: Optional[float] = Field(None, description="Quality of beam sampling.")
#     beam_origin: Optional[np.ndarray] = Field(None, description="Origin of the scanner beam.")
#     head_rotation_axis: Optional[np.ndarray] = Field(None, description="Axis of head rotation.")


#     def cherry_pick(self, cherries: "ScannerSettings", fields: Set[str], template_fields: Optional[Set[str]] = None) -> "ScannerSettings":
#         settings = self.model_copy(deep=True)
#         for field in fields:
#             if hasattr(cherries, field):
#                 setattr(settings, field, getattr(cherries, field))

#         if "basic_template" in fields and cherries._basic_template:
#             if template_fields:
#                 settings._basic_template = cherries._basic_template.cherry_pick(cherries._basic_template, template_fields)
#             else:
#                 settings._basic_template = cherries._basic_template.copy(deep=True)

#         return settings

#     @property
#     def has_template(self) -> bool:
#         """
#         Checks if the ScannerSettings object has a basic template.

#         Returns:
#             bool: True if the _basic_template is set, False othyour_moduleerwise.
#         """
#         return self._basic_template is not None

#     @property
#     def basic_template(self) -> 'ScannerSettings':
#         """
#         Returns the basic template associated with the ScannerSettings object.

#         Returns:
#             ScannerSettings: The basic template.

#         Raises:
#             ValueError: If no template is associated.
#         """
#         if self._basic_template is None:
#             raise ValueError("No template associated with this ScannerSettings.")
#         return self._basic_template

#     @property
#     def has_default_resolution(self) -> bool:
#         """
#         Checks if the ScannerSettings object has default resolution values.

#         Returns:
#             bool: True if both vertical and horizontal resolutions are 0.0, False otherwise.
#         """
#         return self.vertical_resolution == 0.0 and self.horizontal_resolution == 0.0

#     def fit_to_resolution(self, scan_angle_max_rad: float) -> None:
#         """
#         Adjusts the scan frequency and head rotation per second based on the resolution and maximum scan angle.

#         Args:
#             scan_angle_max_rad (float): The maximum scan angle in radians.
#         """
#         self.scan_frequency = (self.pulse_frequency * self.vertical_resolution) / (2.0 * scan_angle_max_rad)
#         self.head_rotation = self.horizontal_resolution * self.scan_frequency

#     @classmethod
#     def create_preset(cls, name: str, pulse_frequency: int, horizontal_resolution: float, vertical_resolution: float,
#                       horizontal_fov: int, min_vertical_angle: float, max_vertical_angle: float,
#                       save_as: Optional[str] = None) -> 'ScannerSettings':

#         preset = ScannerSettings(name=name, pulse_frequency=pulse_frequency,
#                                  horizontal_resolution=horizontal_resolution,
#                                  vertical_resolution=vertical_resolution,
#                                  horizontal_fov=horizontal_fov,
#                                  min_vertical_angle=min_vertical_angle,
#                                  max_vertical_angle=max_vertical_angle)
#         if save_as:
#             preset.to_file(save_as)
#         return preset

#     def to_file(self, file_path: str) -> None:
#         with open(file_path, 'w') as file:
#             json.dump(self.dict(), file)

#     @staticmethod
#     def load_preset(file_path: str) -> 'ScannerSettings':
#         with open(file_path, 'r') as file:
#             preset_data = json.load(file)
#         return ScannerSettings(**preset_data)
