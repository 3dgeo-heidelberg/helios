# import numpy as np
# from helios.platform import Platform
# from helios.platformsettings import PlatformSettings

# def test_current_settings():
#     platform = Platform()
#     platform.settings_speed_m_s = 10.0
#     platform.is_on_ground = True
#     platform.position = np.array([100.0, 200.0, 300.0])

#     current_settings = platform.current_settings
#     import pytest
#     assert current_settings.speed_m_s == 10.0
#     assert current_settings.is_on_ground == True
#     assert current_settings.position == [100.0, 200.0, 300.0]

# def test_apply_settings():
#     platform = Platform()
#     settings = PlatformSettings(speed_m_s=20.0, is_on_ground=False, position=[500.0, 600.0, 700.0])

#     platform.apply_settings(settings)

#     assert platform.settings_speed_m_s == 20.0
#     assert platform.is_on_ground == False
#     assert np.array_equal(platform.position, np.array([500.0, 600.0, 700.0]))

# def test_update_static_cache():
#     platform = Platform()
#     platform.origin_waypoint = np.array([0.0, 0.0, 0.0])
#     platform.target_waypoint = np.array([100.0, 100.0, 0.0])
#     platform.next_waypoint = np.array([200.0, 0.0, 0.0])

#     platform.update_static_cache()

#     assert np.allclose(platform.cached_origin_to_target_dir_xy, np.array([100.0, 100.0, 0.0]))
#     assert np.allclose(platform.cached_target_to_next_dir_xy, np.array([100.0, -100.0, 0.0]))
#     assert platform.cached_end_target_angle_xy == pytest.approx(np.pi / 2)
