import pytest
from unittest.mock import MagicMock
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
        final_state="success",
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


@pytest.fixture(scope="session")
def mock_ros2_logger():
    """
    Fixture providing a mock ROS2 logger for testing.

    This mock implements all standard ROS2 logging methods (debug, info, warn, error, fatal)
    and records calls for assertion in tests.

    Returns:
        MagicMock: A configured mock object that mimics a ROS2 logger
    """
    logger = MagicMock()

    # Ensure all ROS2 logger methods are available
    logger.debug = MagicMock()
    logger.info = MagicMock()
    logger.warn = MagicMock()
    logger.warning = MagicMock()  # ROS2 supports both warn and warning
    logger.error = MagicMock()
    logger.fatal = MagicMock()

    # Add get_name method which ROS2 loggers have
    logger.get_name = MagicMock(return_value="ros2_logger")

    # Track all messages for verification
    logger.messages = {"debug": [], "info": [], "warn": [], "error": [], "fatal": []}

    # Configure each method to store messages
    def store_message(level, *args, **kwargs):
        msg = args[0] if args else kwargs.get("msg", "")
        logger.messages[level].append(msg)

    logger.debug.side_effect = lambda *args, **kwargs: store_message(
        "debug", *args, **kwargs
    )
    logger.info.side_effect = lambda *args, **kwargs: store_message(
        "info", *args, **kwargs
    )
    logger.warn.side_effect = lambda *args, **kwargs: store_message(
        "warn", *args, **kwargs
    )
    logger.warning.side_effect = lambda *args, **kwargs: store_message(
        "warn", *args, **kwargs
    )
    logger.error.side_effect = lambda *args, **kwargs: store_message(
        "error", *args, **kwargs
    )
    logger.fatal.side_effect = lambda *args, **kwargs: store_message(
        "fatal", *args, **kwargs
    )

    return logger
