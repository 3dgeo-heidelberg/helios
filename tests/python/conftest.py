from helios.platform import tripod as tripod_platform, sr22
from helios.scanner import (
    leica_als50,
    riegl_vq_1560i,
    riegl_vz_400,
    vlp16,
)
from helios.scene import ScenePart, StaticScene
from helios.settings import (
    ExecutionSettings,
    OutputSettings,
    set_execution_settings,
    set_output_settings,
)
from helios.survey import Survey
from helios.util import set_rng_seed

import math
import pytest


@pytest.fixture(autouse=True)
def rng_seed():
    """Reset the RNG before each test"""

    set_rng_seed(42)


@pytest.fixture(autouse=True)
def reset_global_state():
    """Reset global state after a test alters it"""

    # NB: This fixture being autouse=True is a stop-gap measure until we
    #     implemented clone behaviour for all model objects, so that we
    #     avoid accidentally editing the global state.

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
def box():
    return lambda: ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")


@pytest.fixture
def scene(box):
    return StaticScene(scene_parts=[box()])


@pytest.fixture
def tls_survey(tls_scanner, tripod, scene):
    def torad(x):
        return (x / 180.0) * math.pi

    survey = Survey(scanner=tls_scanner, platform=tripod, scene=scene)
    survey.add_leg(
        x=0,
        y=0,
        z=0,
        pulse_frequency=2000,
        head_rotation=torad(10),
        rotation_start_angle=torad(0),
        rotation_stop_angle=torad(10),
    )
    return survey


survey = tls_survey
