from pathlib import Path

import helios
from helios.platforms import tripod as tripod_platform, sr22
from helios.scanner import (
    leica_als50,
    riegl_vq_1560i,
    riegl_vz_400,
    vlp16,
)
from helios.scene import DynamicScene, Material, ScenePart, StaticScene
from helios.settings import (
    ExecutionSettings,
    FullWaveformSettings,
    OutputSettings,
    ProgressBarStrategy,
    set_execution_settings,
    set_output_settings,
)
from helios.survey import Survey
from helios.utils import add_asset_directory, set_rng_seed

import math
import numpy as np
import pytest


@pytest.fixture(autouse=True)
def rng_seed():
    """Reset the RNG before each test"""

    set_rng_seed(42)


@pytest.fixture
def reset_global_state():
    """Reset global state after a test alters it"""

    yield

    set_execution_settings(ExecutionSettings())
    set_output_settings(OutputSettings())


@pytest.fixture
def single_tls_scanner():
    return riegl_vz_400()


@pytest.fixture
def single_als_scanner():
    return leica_als50()


@pytest.fixture
def multi_tls_scanner():
    return vlp16()


@pytest.fixture
def multi_als_scanner():
    return riegl_vq_1560i()


# A few more generalized fixture names to be used
# if the more detailed specs do not matter
tls_scanner = single_tls_scanner
als_scanner = single_als_scanner
scanner = tls_scanner


@pytest.fixture
def tripod():
    return tripod_platform()


@pytest.fixture
def airplane():
    return sr22()


@pytest.fixture
def box_f():
    return lambda: ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")


@pytest.fixture
def box(box_f):
    return box_f()


@pytest.fixture
def wall_f():
    return (
        lambda: ScenePart.from_obj("data/sceneparts/basic/plane/plane.obj")
        .scale(200)
        .rotate(angle=90 * helios.units.deg, axis=(1, 0, 0))
        .translate([0, 50, 0])
    )


@pytest.fixture
def wall(wall_f):
    return wall_f()


@pytest.fixture
def scene(box):
    return StaticScene(scene_parts=[box])


@pytest.fixture
def wall_scene(wall):
    return StaticScene(scene_parts=[wall])


@pytest.fixture
def tls_survey(tls_scanner, tripod, scene):
    survey = Survey(scanner=tls_scanner, platform=tripod, scene=scene)
    survey.add_leg(
        x=0,
        y=0,
        z=0,
        pulse_frequency=2000,
        scan_angle="20 deg",
        head_rotation="10 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="10 deg",
        scan_frequency=120,
    )
    return survey


survey = tls_survey


@pytest.fixture
def dynamic_test_execution_settings():
    return ExecutionSettings(
        num_threads=1,
        kdt_num_threads=1,
        kdt_geom_num_threads=1,
        progressbar=ProgressBarStrategy.NONE,
    )


@pytest.fixture
def dynamic_test_scene():
    return DynamicScene.from_xml("data/test/dynamic_scene.xml")


@pytest.fixture
def dynamic_test_static_control_scene():
    return StaticScene.from_xml("data/test/dynamic_scene_static_control.xml")


@pytest.fixture
def dynamic_test_survey_f():
    def create(scene):
        survey = Survey(scanner=riegl_vz_400(), platform=tripod_platform(), scene=scene)
        survey.add_leg(
            x=-30,
            y=-30,
            z=0,
            force_on_ground=True,
            pulse_frequency=5000,
            min_vertical_angle="-40 deg",
            max_vertical_angle="60 deg",
            scan_frequency=120,
            head_rotation="-60 deg/s",
            rotation_start_angle="340 deg",
            rotation_stop_angle="280 deg",
        )
        return survey

    return create


@pytest.fixture
def manual_dynamic_test_survey(dynamic_test_survey_f, dynamic_test_scene):
    return dynamic_test_survey_f(dynamic_test_scene)


@pytest.fixture
def xml_dynamic_test_survey():
    return Survey.from_xml("data/test/dynamic_survey.xml")


@pytest.fixture
def dynamic_test_static_control_survey(
    dynamic_test_survey_f, dynamic_test_static_control_scene
):
    return dynamic_test_survey_f(dynamic_test_static_control_scene)


@pytest.fixture(
    params=("manual_dynamic_test_survey", "xml_dynamic_test_survey"),
    ids=("dynamic-scene-from-xml", "survey-from-xml"),
)
def dynamic_test_survey(request):
    return request.getfixturevalue(request.param)


@pytest.fixture
def light_als_multiscanner_survey():
    survey = Survey.from_xml("data/surveys/demo/light_als_toyblocks_multiscanner.xml")
    for leg in survey.legs:
        leg.scanner_settings.pulse_frequency = 2000
        leg.scanner_settings.scan_frequency = 20
        leg.scanner_settings.trajectory_time_interval = 0.2
    return survey


@pytest.fixture()
def assetdir(tmp_path):
    add_asset_directory(tmp_path)
    tmp_path = tmp_path / "root"
    tmp_path.mkdir()

    a = tmp_path / "a"
    b = tmp_path / "b" / "bb"
    c = tmp_path / "c"
    a.mkdir()
    b.mkdir(parents=True)
    c.mkdir()

    a1 = a / "some.obj"
    a2 = a / "second.obj"
    a3 = a / "notobj.smth"
    bb1 = b / "other.obj"
    c1 = c / "notobj.smt"
    a1.touch()
    a2.touch()
    a3.touch()
    bb1.touch()
    c1.touch()

    return tmp_path

@pytest.fixture
def groundplane_f():
    """Factory: groundplane rotated about the x-axis through the origin."""
    return lambda tilt_deg=0.0: (
        ScenePart.from_obj("data/sceneparts/basic/groundplane/groundplane.obj")
        .rotate(angle=tilt_deg * helios.units.deg, axis=(1, 0, 0))
    )

@pytest.fixture
def ground_scene(groundplane_f, tilt_angle):
    groundplane = groundplane_f(tilt_angle)
    return StaticScene(scene_parts=[groundplane])


# The three light models of EnergyMaths::computeBDRF. Which one a material uses is
# decided by its diffuse and specular component vectors, exactly as in
# Material::isPhong, isLambert and isDirectionIndependent (src/scene/Material.cpp:61-82).
LIGHT_MODELS = ("direction_independent", "lambert", "phong")

_NONZERO = np.array([0.5, 0.5, 0.5, 1.0], dtype=np.float64)
_ZERO = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

# Kd and Ks vectors that select each light model.
_LIGHT_MODEL_COMPONENTS = {
    #                      diffuse (Kd), specular (Ks)
    "direction_independent": (_ZERO, _ZERO),
    "lambert": (_NONZERO, _ZERO),
    "phong": (_NONZERO, _NONZERO),
}


@pytest.fixture
def material_f():
    """Factory for a material with a given light model and reflectance.

    The light model follows from the component vectors:
        Kd != 0, Ks != 0  ->  Phong
        Kd != 0, Ks == 0  ->  Lambert
        Kd == 0, Ks == 0  ->  direction independent
    The remaining combination, Kd == 0 with Ks != 0, matches none of the three
    and makes computeBDRF throw, so it cannot be requested here.

    Keep apart the two specular quantities. The Ks *vector* only selects the
    model, while the scalar `specularity` is the k_s weight inside the Phong
    formula and `specular_exponent` is its N_s.
    """

    def _make(
        light_model="lambert",
        reflectance=50.0,
        specularity=0.5,
        specular_exponent=10.0,
        name="None",
    ):
        if light_model not in _LIGHT_MODEL_COMPONENTS:
            raise ValueError(
                f"unknown light model {light_model!r}, expected one of {LIGHT_MODELS}"
            )
        diffuse, specular = _LIGHT_MODEL_COMPONENTS[light_model]
        return Material(
            name=name,
            reflectance=reflectance,
            specularity=specularity,
            specular_exponent=specular_exponent,
            diffuse_components=diffuse.copy(),
            specular_components=specular.copy(),
        )

    return _make


@pytest.fixture
def plane_scene_f(groundplane_f):
    """Factory for a plane scene with a given tilt, material and extent.

    The plane is rotated about the x-axis through the origin, so it keeps
    passing through (0, 0, 0). A beam pointing straight down therefore always
    meets it at the same range, and the tilt sets the incidence angle alone.

    `scale` widens the plane, which the unscaled 2 x 2 m one needs beyond a
    nadir range of roughly 900 m, and at steep tilts.
    """

    def _make(tilt_deg=0.0, material=None, scale=1.0):
        part = groundplane_f(tilt_deg)
        if scale != 1.0:
            part = part.scale(scale)
        if material is not None:
            names = list(part.materials.keys())
            if len(names) != 1:
                raise AssertionError(f"expected exactly one material, found {names}")
            part.materials[names[0]] = material
        return StaticScene(scene_parts=[part])

    return _make

@pytest.fixture
def step_scene(groundplane_f, step_size, edge_offset):
    """
    step_size controls the height difference between the steps (planes)
    edge offset controls the location of the edge; set to 0, the step will go through the origin
    move along the x-direction by setting <= or >= 0 (to slide the edge across the footprint)
    """
    upper_step = groundplane_f(0).translate([1 + edge_offset, 0, step_size])
    lower_step = groundplane_f(0)  # this one stays in place
    return StaticScene(scene_parts=[upper_step, lower_step])

@pytest.fixture
def small_box_scene(box_f, box_size):
    initial_box_side_length = 100
    scale_factor = box_size / initial_box_side_length
    # scale as intended and scale such that the upper surface sitz at z=0
    box = box_f().scale(scale_factor).translate([0, 0, -box_size/2])
    return StaticScene(scene_parts=[box])

@pytest.fixture
def tripod_down():
    platform_file = "python/helios/data/platforms.xml"
    return helios.Platform.from_xml(platform_file, platform_id="tripod_down")

# Range from the beam origin to a target at the origin, for the nadir survey below.
NADIR_RANGE_M = 100.0

# The beam origin sits higher than the leg position by the scanner's own mount offset,
# riegl_vz400 beamOrigin z = 0.2 (python/helios/data/scanners_tls.xml). The tripod_down
# scannerMount was set to z = 0 so that this is the only contribution.
# Verified against the reported beam_origin: a leg at z = 100 puts it at z = 100.2.
SCANNER_Z_OFFSET_M = 0.2


# The default of ScannerSettings.beam_divergence_angle. A leg that sets exactly this
# value is indistinguishable from a leg that does not set it at all, so the scanner
# keeps its XML divergence instead. Verified: asking for 0.003 leaves the VZ-400 at
# its XML value of 0.0003, while asking for 0.0001 does take effect.
_BEAM_DIVERGENCE_SETTINGS_DEFAULT = 0.003


@pytest.fixture
def nadir_survey_f(tls_scanner, tripod_down):
    """Factory for a downward-looking survey over a given scene.

    Call it with the scene under test, for example nadir_survey_f(ground_scene).

    Sweepable parameters:

    nadir_range         distance from the beam origin to a target in the z = 0
                        plane, in metres.
    beam_sample_quality number of concentric subray rings (BSQ). Left at the
                        scanner default when not given.
    beam_divergence     full divergence angle in radians. Left at the scanner
                        XML value when not given.

    Any other leg setting can be overridden by keyword, for example
    pulse_frequency or the head rotation range.

    Note that the scan sweeps roughly +-1.05e-3 * nadir_range metres either side
    of nadir, so scenes must be wide enough for the chosen range. The unscaled
    groundplane spans +-1 m and is therefore good to about 900 m.

    Only one survey may be built per test, because a scanner cannot be shared
    between surveys.
    """

    def _make(
        scene,
        nadir_range=NADIR_RANGE_M,
        beam_sample_quality=None,
        beam_divergence=None,
        **leg_overrides,
    ):
        leg_settings = dict(
            x=0,
            y=0,
            z=nadir_range - SCANNER_Z_OFFSET_M,
            pulse_frequency=100000,
            vertical_resolution="0.03 deg",
            horizontal_resolution="0.03 deg",
            scan_angle="0.06 deg",
            # A head rotation range is required, otherwise the leg has zero
            # duration and no pulse is emitted. The rotation speed does not
            # affect where the pulses land, so it is left at the default.
            # Keep the sweep narrow so the footprint stays close to nadir.
            rotation_start_angle="-0.03 deg",
            rotation_stop_angle="0.03 deg",
        )
        if beam_divergence is not None:
            if beam_divergence == _BEAM_DIVERGENCE_SETTINGS_DEFAULT:
                raise ValueError(
                    f"beam_divergence={beam_divergence} equals the ScannerSettings "
                    "default and is therefore silently ignored, leaving the scanner "
                    "at its XML divergence. Pick a different value."
                )
            leg_settings["beam_divergence_angle"] = beam_divergence
        leg_settings.update(leg_overrides)

        survey = Survey(scanner=tls_scanner, platform=tripod_down, scene=scene)
        if beam_sample_quality is not None:
            survey.full_waveform_settings = FullWaveformSettings(
                beam_sample_quality=beam_sample_quality
            )
        survey.add_leg(**leg_settings)
        return survey

    return _make