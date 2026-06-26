import copy
import pytest
import numpy as np
from dataclasses import dataclass

import helios.scanner as scanner_module
from helios.platforms import tripod, PlatformSettings, StaticPlatformSettings
from helios.scene import ScenePart, StaticScene
from helios.settings import ExecutionSettings, OutputFormat
from helios.scanner import *
from helios.survey import Survey
from helios.utils import set_rng_seed
from helios import HeliosException


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
        assert copied._provided_fields == settings._provided_fields

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


def test_scanner_settings_max_duration_from_xml():
    survey = Survey.from_xml("data/surveys/demo/tls_livox.xml")
    points, _ = survey.run()
    assert points.shape[0] > 0
    leg1_duration = (
        points[points["point_source_id"] == 0][-1]["gps_time"]
        - points[points["point_source_id"] == 0][0]["gps_time"]
    )
    leg2_duration = (
        points[points["point_source_id"] == 1][-1]["gps_time"]
        - points[points["point_source_id"] == 1][0]["gps_time"]
    )
    # for leg1 maxDuration_s is set to 0.2 in the xml, so the duration should be around that
    assert 0.19 < leg1_duration < 0.201
    assert (
        leg2_duration != leg1_duration
    )  # leg2 should not be affected by leg1's maxDuration_s


def test_scanner_settings_max_duration_manual():
    box = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene = StaticScene([box])
    scanner = scanner_from_name("livox_mid40")
    platform = tripod()
    plat_set1 = PlatformSettings(x=0, y=0, z=0)
    plat_set2 = PlatformSettings(x=0, y=0, z=0)
    scan_set1 = ScannerSettings(pulse_frequency=100000, max_duration=0.4)
    scan_set2 = ScannerSettings(pulse_frequency=100000, max_duration=5.2)
    survey = Survey(scanner=scanner, platform=platform, scene=scene)
    survey.add_leg(scanner_settings=scan_set1, platform_settings=plat_set1)
    survey.add_leg(scanner_settings=scan_set2, platform_settings=plat_set2)
    points, _ = survey.run()
    assert points.shape[0] > 0
    leg1_duration = (
        points[points["point_source_id"] == 0][-1]["gps_time"]
        - points[points["point_source_id"] == 0][0]["gps_time"]
    )
    leg2_duration = (
        points[points["point_source_id"] == 1][-1]["gps_time"]
        - points[points["point_source_id"] == 1][0]["gps_time"]
    )

    assert 0.39 < leg1_duration < 0.401
    assert leg2_duration != leg1_duration
    assert 5.19 < leg2_duration < 5.201


def test_max_duration_no_infinite_run():
    box = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene = StaticScene([box])
    scanner = scanner_from_name("livox_mid70")
    platform = tripod()
    plat_set = PlatformSettings(x=0, y=0, z=10)
    scan_set = ScannerSettings(pulse_frequency=50000, trajectory_time_interval=0.1)
    survey = Survey(scanner=scanner, platform=platform, scene=scene)
    survey.add_leg(scanner_settings=scan_set, platform_settings=plat_set)
    with pytest.raises(
        HeliosException,
        match="o platform movement, scanner head rotation or maximum duration set",
    ):
        survey.run()


def test_scanner_settings_provided_fields():
    settings = ScannerSettings(
        pulse_frequency=1200,
        scan_frequency=30,
        head_rotation="12 deg/s",
    )

    assert settings._provided_fields == {
        "pulse_frequency",
        "scan_frequency",
        "head_rotation",
    }
    assert settings.pulse_frequency == 1200
    assert settings.scan_frequency == 30
    target = ScannerSettings()
    target.update_from_object(settings)
    assert target.pulse_frequency == 1200
    assert target.scan_frequency == 30


def test_scanner_settings_uses_supported_scanner_frequency():
    scanner = scanner_from_name("livox_mid40")

    settings = ScannerSettings(
        scan_frequency=20,
        head_rotation="20 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="1 deg",
    )

    settings._resolve_for_scanner(scanner)

    supported = list(scanner._cpp_object.supported_pulse_freqs_hz)
    assert settings.pulse_frequency == supported[0]
    assert settings._provided_fields == {
        "scan_frequency",
        "head_rotation",
        "rotation_start_angle",
        "rotation_stop_angle",
    }


def test_scanner_settings_resolve_for_scanner_keeps_explicit_pulse_frequency():
    scanner = scanner_from_name("livox_mid40")

    settings = ScannerSettings(
        pulse_frequency=100000,
        scan_frequency=20,
        head_rotation="20 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="1 deg",
    )

    original = settings.pulse_frequency
    settings._resolve_for_scanner(scanner)

    assert settings.pulse_frequency == original
    assert "pulse_frequency" in settings._provided_fields


def test_survey_add_leg_resolves_scanner_pulse_frequency_when_not_provided():
    box = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene = StaticScene([box])
    scanner = scanner_from_name("livox_mid40")
    platform = tripod()

    survey = Survey(scanner=scanner, platform=platform, scene=scene)

    settings = ScannerSettings(
        scan_frequency=20,
        head_rotation="20 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="1 deg",
    )

    survey.add_leg(
        scanner_settings=settings,
        platform_settings=PlatformSettings(x=0, y=0, z=0),
    )
    survey.add_leg(scan_frequency=120)
    survey.add_leg(pulse_frequency=200000)

    assert (
        survey.legs[0].scanner_settings.pulse_frequency
        == list(scanner._cpp_object.supported_pulse_freqs_hz)[0]
    )
    assert (
        survey.legs[1].scanner_settings.pulse_frequency
        == list(scanner._cpp_object.supported_pulse_freqs_hz)[0]
    )
    assert survey.legs[2].scanner_settings.pulse_frequency == 200000


@dataclass
class WarmupTestCase:
    scanner_name: str
    settings: dict


TEST_CASES = [
    pytest.param(
        WarmupTestCase(
            scanner_name="riegl_vq_880g",
            settings=dict(
                pulse_frequency=100000,
                scan_angle="18 deg",
                scan_frequency=50,
                head_rotation="20 deg/s",
                rotation_start_angle="0 deg",
                rotation_stop_angle="10 deg",
            ),
        ),
        id="riegl_vq_880g",
    ),
    pytest.param(
        WarmupTestCase(
            scanner_name="livox_mid70",
            settings=dict(
                pulse_frequency=100000,
                head_rotation="20 deg/s",
                rotation_start_angle="0 deg",
                rotation_stop_angle="10 deg",
            ),
        ),
        id="livox_mid70",
    ),
    pytest.param(
        WarmupTestCase(
            scanner_name="optech_2033",
            settings=dict(
                pulse_frequency=100000,
                scan_angle="15 deg",
                scan_frequency=20,
                head_rotation="20 deg/s",
                rotation_start_angle="0 deg",
                rotation_stop_angle="10 deg",
            ),
        ),
        id="optech_2033",
    ),
    pytest.param(
        WarmupTestCase(
            scanner_name="riegl_vz_400",
            settings=dict(
                pulse_frequency=100000,
                min_vertical_angle="-40 deg",
                max_vertical_angle="60 deg",
                scan_frequency=20,
                head_rotation="20 deg/s",
                rotation_start_angle="0 deg",
                rotation_stop_angle="10 deg",
            ),
        ),
        id="riegl_vz400",
    ),
]


@pytest.mark.parametrize("case", TEST_CASES)
def test_optics_warmup_phase(case):
    box1 = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene1 = StaticScene([box1])
    scanner1 = scanner_from_name(case.scanner_name)
    platform1 = tripod()
    plat_set1 = StaticPlatformSettings(x=0, y=0, z=0, force_on_ground=False)
    settings = case.settings.copy()
    scan_set1 = ScannerSettings(
        **settings, optics_warmup_phase=0.002, max_duration=0.01
    )
    survey1 = Survey(
        scanner=scanner1,
        platform=platform1,
        scene=scene1,
        gps_time="2022-01-01 00:00:00",
    )
    survey1.add_leg(scanner_settings=scan_set1, platform_settings=plat_set1)

    box2 = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene2 = StaticScene([box2])
    scanner2 = scanner_from_name(case.scanner_name)
    platform2 = tripod()
    plat_set2 = StaticPlatformSettings(x=0, y=0, z=0, force_on_ground=False)
    scan_set2 = ScannerSettings(**settings, optics_warmup_phase=0, max_duration=0.012)
    survey2 = Survey(
        scanner=scanner2,
        platform=platform2,
        scene=scene2,
        gps_time="2022-01-01 00:00:00",
    )
    survey2.add_leg(scanner_settings=scan_set2, platform_settings=plat_set2)

    points_warmup, _ = survey1.run()
    points_no_warmup, _ = survey2.run()
    assert points_warmup.shape[0] > 0
    assert points_no_warmup.shape[0] > 0
    # Check that both runs start at the same GPS time ( with respect to warmup)
    points_no_warmup_clipped = points_no_warmup[
        points_no_warmup["gps_time"] >= points_no_warmup["gps_time"].min() + 0.002
    ]
    assert (
        abs(points_warmup[0]["gps_time"] - points_no_warmup_clipped[0]["gps_time"])
        < 1e-2
    )

    # Validate that warmup data falls within no-warmup time window
    warmup_times = points_warmup["gps_time"]
    no_warmup_times = points_no_warmup["gps_time"]

    assert warmup_times.min() >= no_warmup_times.min()
    assert warmup_times.max() <= no_warmup_times.max()

    delta_t_warmup = warmup_times.max() - warmup_times.min()
    delta_t_nowarmup = no_warmup_times.max() - no_warmup_times.min()

    assert np.isclose(delta_t_warmup, 0.01, atol=1e-4)
    assert np.isclose(delta_t_nowarmup, 0.012, atol=1e-4)

    points_nowarmup_clipped = points_no_warmup[
        no_warmup_times >= (no_warmup_times.min() + 0.002)
    ]
    # make sure points are ordered by gps_time
    points_warmup = points_warmup[np.argsort(points_warmup["gps_time"])]
    points_nowarmup_clipped = points_nowarmup_clipped[
        np.argsort(points_nowarmup_clipped["gps_time"])
    ]

    assert abs(len(points_warmup) - len(points_nowarmup_clipped)) <= 1
    # check distance between three points: index 0, 500 and 1000
    warmup = points_warmup[np.argsort(points_warmup["fullwave_index"])]
    nowarmup = points_no_warmup_clipped[
        np.argsort(points_no_warmup_clipped["fullwave_index"])
    ]

    dist = np.linalg.norm(
        warmup["position"] - nowarmup["position"],
        axis=1,
    )
    assert np.percentile(dist, 99.0) < 0.1
