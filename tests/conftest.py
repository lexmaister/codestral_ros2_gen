import pytest
from pathlib import Path
from codestral_ros2_gen import get_project_root, load_config
from codestral_ros2_gen import get_config_path, load_config
from codestral_ros2_gen.generators.generation_attempt import AttemptMetrics


@pytest.fixture(scope="session")
def project_config():
    """Fixture to load project config.yaml."""
    config_path = get_config_path()
    return load_config(config_path)


sm_path = "tests/data/sample_metrics.jsonl"
sc_path = "tests/data/sample_config.yaml"


@pytest.fixture(scope="session")
def sample_metrics():
    """Fixture to load sample object height metrics."""
    return AttemptMetrics(
        attempt_time=1.0,
        success=True,
        tests_passed=10,
        tests_failed=10,
        tests_skipped=10,
        prompt_tokens=10,
        completion_tokens=10,
        total_tokens=10,
    )


@pytest.fixture(scope="session")
def sample_metrics_path():
    """Fixture to use sample object height metrics file path."""
    return get_project_root() / Path(sm_path)


@pytest.fixture(scope="session")
def sample_config():
    """Fixture to load sample config.yaml."""
    config_path = get_project_root() / Path(sc_path)
    return load_config(config_path)


@pytest.fixture(scope="session")
def sample_config_path():
    """Fixture to use sample config.yaml path."""
    return get_project_root() / Path(sc_path)


@pytest.fixture(scope="session")
def sample_attempt_metrics():
    """Fixture to get dummy sample attempt metrics."""
    return get_project_root() / Path("tests/data/sample_config.yaml")
