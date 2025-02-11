from helios.settings import ExecutionSettings, set_execution_settings
from helios.util import set_rng_seed

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
