import copy
import pytest
import numpy as np

import helios.scanner as scanner_module
from helios.platforms import tripod
from helios.scene import ScenePart, StaticScene
from helios.settings import ExecutionSettings, OutputFormat
from helios.scanner import *
from helios.survey import Survey
from helios.utils import set_rng_seed


def test_preinstantiated_scanners():
    for scanner_name in list_scanners():
        scanner_factory = getattr(scanner_module, scanner_name)
        assert isinstance(scanner_factory(), Scanner)


def test_list_scanners_matches_registry():
    assert list_scanners() == list(scanner_module.SCANNER_REGISTRY.keys())


def test_scanner_from_name():
    for scanner_name in list_scanners():
        assert isinstance(scanner_from_name(scanner_name), Scanner)


def test_scanner_from_name_invalid():
    with pytest.raises(ValueError, match="Unknown scanner"):
        scanner_from_name("not-a-valid-scanner")


def test_scanneer_flag_from_xml_set():
    from helios.utils import is_xml_loaded

    scanner = Scanner.from_xml("data/scanners_als.xml", scanner_id="leica_als50")
    assert is_xml_loaded(scanner)


def test_scanner_settings_clone_and_deepcopy():
    settings = ScannerSettings(
        pulse_frequency=1200,
        scan_frequency=30,
        head_rotation="12 deg/s",
    )
    settings._is_loaded_from_xml = True
    settings.runtime_note = "temporary"

    for copied in (settings.clone(), copy.deepcopy(settings)):
        assert copied is not settings
        assert copied.pulse_frequency == 1200
        assert copied.scan_frequency == 30
        assert copied.head_rotation == settings.head_rotation
        assert not hasattr(copied, "_is_loaded_from_xml")
        assert not hasattr(copied, "runtime_note")

        copied.pulse_frequency = 2000
        assert settings.pulse_frequency == 1200

        survey = Survey(
            scanner=riegl_vz_400(),
            platform=tripod(),
            scene=StaticScene(
                scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
            ),
        )
        survey.add_leg(
            scanner_settings=copied,
            x=0,
            y=0,
            z=0,
            rotation_start_angle="0 deg",
            rotation_stop_angle="10 deg",
        )
        points, trajectory = survey.run(
            format=OutputFormat.NPY, execution_settings=ExecutionSettings(num_threads=1)
        )
        assert points.shape[0] > 0
        assert trajectory.shape[0] > 0


def test_scanner_clone_and_deepcopy():
    for copy_fn in (lambda s: s.clone(), copy.deepcopy):
        scanner = riegl_vz_400()
        scanner.runtime_note = "temporary"
        copied = copy_fn(scanner)

        assert copied is not scanner
        assert copied._cpp_object is not scanner._cpp_object
        if scanner._cpp_object.platform is None:
            assert copied._cpp_object.platform is None
        else:
            assert copied._cpp_object.platform is not scanner._cpp_object.platform
        assert copied._cpp_object.detector is not scanner._cpp_object.detector
        assert not hasattr(copied, "runtime_note")

        def run_with(scanner_obj):
            survey = Survey(
                scanner=scanner_obj,
                platform=tripod(),
                scene=StaticScene(
                    scene_parts=[
                        ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
                    ]
                ),
            )
            survey.add_leg(
                scanner_settings=ScannerSettings(
                    pulse_frequency=2000,
                    scan_angle="20 deg",
                    head_rotation="10 deg/s",
                    rotation_start_angle="0 deg",
                    rotation_stop_angle="10 deg",
                    scan_frequency=120,
                ),
                x=0,
                y=0,
                z=0,
            )
            return survey.run(
                format=OutputFormat.NPY,
                execution_settings=ExecutionSettings(num_threads=1),
            )

        set_rng_seed(42)
        points, trajectory = run_with(scanner)
        set_rng_seed(42)
        copied_points, copied_trajectory = run_with(copied)

        np.testing.assert_array_equal(points, copied_points)
        np.testing.assert_array_equal(trajectory, copied_trajectory)


@pytest.mark.parametrize(
    "settings_cls",
    [
        RotatingOpticsScannerSettings,
        OscillatingOpticsScannerSettings,
        LineOpticsScannerSettings,
        ConicOpticsScannerSettings,
        RisleyOpticsScannerSettings,
    ],
)
def test_specialized_scanner_settings_clone_and_deepcopy(settings_cls):
    settings = settings_cls()
    settings.runtime_note = "temporary"

    for copied in (settings.clone(), copy.deepcopy(settings)):
        assert copied is not settings
        assert isinstance(copied, settings_cls)
        assert copied._cpp_object is not settings._cpp_object
        assert not hasattr(copied, "runtime_note")
