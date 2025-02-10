from helios.settings import ExecutionSettings, set_execution_settings

import pytest


@pytest.fixture
def reset_global_state():
    """Reset global state after a test alters it"""

    yield

    set_execution_settings(ExecutionSettings())
