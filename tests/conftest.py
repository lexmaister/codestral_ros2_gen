import pytest
from codestral_ros2_gen import get_config_path, load_config


@pytest.fixture(scope="session")
def project_config():
    """Fixture to load project config.yaml."""
    config_path = get_config_path()
    return load_config(config_path)
