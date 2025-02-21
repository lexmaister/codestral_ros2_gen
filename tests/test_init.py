import pytest
from codestral_ros2_gen import (
    get_project_root,
    get_config_path,
    load_config,
    setup_logger,
)


def test_get_project_root():
    root = get_project_root()
    assert root.exists()
    assert root.is_dir()


def test_get_config_path():
    with pytest.raises(FileNotFoundError):
        get_config_path("non_existent_config.yaml")


def test_load_config():
    config_path = get_project_root() / "config" / "config.yaml"
    if config_path.exists():
        config = load_config(config_path)
        assert isinstance(config, dict)
    else:
        with pytest.raises(RuntimeError):
            load_config(config_path)


def test_setup_logger():
    config_path = get_project_root() / "config" / "logger_config.yaml"
    if config_path.exists():
        logger = setup_logger(config_path=config_path)
        assert logger is not None
        assert logger.hasHandlers()
    else:
        with pytest.raises(SystemExit):
            setup_logger(config_path=config_path)
