import pytest
from codestral_ros2_gen import logger, get_project_root, get_config_path


@pytest.fixture(scope="session")
def project_config():
    """Fixture to load project config.yaml."""
    import yaml

    config_path = get_config_path()
    if not config_path.exists():
        pytest.skip(f"Config file not found at {config_path}")

    with open(config_path, "r") as f:
        return yaml.safe_load(f)
