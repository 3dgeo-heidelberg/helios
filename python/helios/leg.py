# from pydantic import BaseModel, Field, NonNegativeInt
# from typing import Optional, Union
# from helios.scannersettings import ScannerSettings
# from helios.platformsettings import PlatformSettings

# import numpy as np

# class Leg(BaseModel):
#     scanner_settings: Optional[ScannerSettings] = Field(default=None)
#     platform_settings: Optional[PlatformSettings] = Field(default=None)
#     trajectory_settings: Optional[TrajectorySettings] = Field(default=None)
#     length: float = Field(default=0.0)
#     serial_id: NonNegativeInt
#     strip: Optional[ScanningStrip] = Field(default=None)
#     was_processed: bool = Field(default=False)
#     look_at: Optional[Union[np.ndarray, int]] = Field(default=None)  # Only for TLS!! Might transfer to utils
#     position: Optional[np.ndarray] = Field(default=None)
#     horizontal_fov: Optional[float] = Field(default=None)
#     vertical_fov: Optional[float] = Field(default=None)
#     horizontal_resolution: Optional[float] = Field(default=None)
#     vertical_resolution: Optional[float] = Field(default=None)
#     is_active: bool = Field(default=True)

#     @property
#     def belongs_to_strip(self) -> bool:
#         return self.strip is not None
