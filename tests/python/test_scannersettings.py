# from pydantic import ValidationError
# from typing import Optional, Set
# import numpy as np
# from helios.scannersettings import ScannerSettings

# def test_has_template():
#     template = ScannerSettings(name="Template")
#     settings = ScannerSettings(name="Settings", _basic_template=template)
#     assert settings.has_template is True
#     assert settings.basic_template == template

# def test_no_template():
#     settings = ScannerSettings(name="Settings")
#     assert settings.has_template is False
#     with pytest.raises(ValueError):
#         _ = settings.basic_template

# def test_has_default_resolution():
#     settings = ScannerSettings(name="Settings")
#     assert settings.has_default_resolution is True
#     settings.vertical_resolution = 1.0
#     assert settings.has_default_resolution is False

# def test_fit_to_resolution():
#     settings = ScannerSettings(name="Settings", pulse_frequency=100, vertical_resolution=1.0, horizontal_resolution=1.0)
#     settings.fit_to_resolution(np.pi / 2)
#     assert settings.scan_frequency == pytest.approx(31.831, rel=1e-2)
#     assert settings.head_rotation == pytest.approx(31.831, rel=1e-2)

# def test_create_preset():
#     preset = ScannerSettings.create_preset(
#         name="Preset",
#         pulse_frequency=1000,
#         horizontal_resolution=0.5,
#         vertical_resolution=0.5,
#         horizontal_fov=90,
#         min_vertical_angle=-10.0,
#         max_vertical_angle=10.0
#     )
#     assert preset.name == "Preset"
#     assert preset.pulse_frequency == 1000
#     assert preset.horizontal_resolution == 0.5
#     assert preset.vertical_resolution == 0.5
#     assert preset.horizontal_fov == 90
#     assert preset.min_vertical_angle == -10.0
#     assert preset.max_vertical_angle == 10.0

# def test_to_file(tmp_path):
#     settings = ScannerSettings(name="Settings")
#     file_path = tmp_path / "settings.json"
#     settings.to_file(str(file_path))
#     assert file_path.exists()

# def test_load_preset(tmp_path):
#     settings = ScannerSettings(name="Settings")
#     file_path = tmp_path / "settings.json"
#     settings.to_file(str(file_path))
#     loaded_settings = ScannerSettings.load_preset(str(file_path))
#     assert loaded_settings == settings
