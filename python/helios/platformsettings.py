# from pydantic import BaseModel
# from typing import Optional, Set


# class PlatformSettings(BaseModel):
#     name: str = "#nullid#"
#     basic_template: Optional["PlatformSettings"] = None
#     x: float = 0.0
#     y: float = 0.0
#     z: float = 0.0 # combine 3 coordinates into a tuple?
#     is_yaw_angle_specified: bool = False
#     yaw_angle: float = 0.0
#     is_on_ground: bool = False
#     is_stop_and_turn: bool = True
#     is_smooth_turn: bool = False
#     is_slowdown_enabled: bool = True
#     speed_m_s: float = 70.0
#     altitude: float = 0.0


#     def cherry_pick(self, cherries: "PlatformSettings", fields: Set[str], template_fields: Optional[Set[str]] = None) -> "PlatformSettings":
#         settings = self.model_copy(deep=True)
#         for field in fields:
#             if hasattr(cherries, field):
#                 setattr(settings, field, getattr(cherries, field))
#         if "basic_template" in fields and cherries.basic_template:
#             if template_fields:
#                 settings.basic_template = cherries.basic_template.cherry_pick(cherries.basic_template, template_fields)
#             else:
#                 settings.basic_template = cherries.basic_template.model_copy(deep=True)
#         return settings

#     @property
#     def has_template(self) -> bool:
#         return self.basic_template is not None
