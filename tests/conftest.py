import pytest
from codestral_ros2_gen import get_config_path, load_config
from codestral_ros2_gen.utils.metrics_handler import MetricsHandler


@pytest.fixture(scope="session")
def project_config():
    """Fixture to load project config.yaml."""
    config_path = get_config_path()
    return load_config(config_path)


@pytest.fixture(scope="session")
def sample_metrics():
    """Fixture to load sample object height metrics."""
    mh = MetricsHandler({}, metrics_file="tests/data/sample_metrics.jsonl")
    mh.load_metrics()
    return mh.metrics_df
