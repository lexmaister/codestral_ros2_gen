#!/usr/bin/env python3

import logging
from pathlib import Path
import yaml


def get_project_root() -> Path:
    """
    Get the project root directory.

    Returns:
        Path: Absolute path to project root
    """
    return Path(__file__).parent.parent.parent


def get_config_path(config_name: str = "config.yaml") -> Path:
    """
    Get the full path to a config file.

    Args:
        config_name: Name of the config file, default is "config.yaml"

    Returns:
        Path: Full path to the config file

    Raises:
        FileNotFoundError: If config file doesn't exist
    """
    config_path = get_project_root() / "config" / config_name
    if not config_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
    return config_path


def load_config(config_path: Path) -> dict:
    """
    Load configuration from YAML file.

    Args:
        config_path: Path to configuration file

    Returns:
        Dict: Configuration dictionary

    Raises:
        RuntimeError: If config file cannot be loaded
    """
    try:
        with open(config_path, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        raise RuntimeError(f"Error loading config: {str(e)}")


def setup_logger(name: str = "root", config_path: Path = None) -> logging.Logger:
    """
    Setup centralized logger for the package. Exits program if setup fails.
    Logger config file is expected to be in the 'config' directory.
    Args:
        config_path: Path to the config file 'logger_config.yaml', default is None (search in config directory)
        name: Logger name, default is "root"

    Returns:
        logging.Logger: Configured logger instance

    Note:
        Exits program (SystemExit) if:
        - Config file is missing (FileNotFoundError)
        - Config is invalid (ValueError)
        - Log file/directory operations fail (OSError)
        Error message will be printed to stdout before exit.
    """
    if config_path is None:
        config_path = get_config_path(config_name="logger_config.yaml")

    log_config = load_config(config_path)

    # Validate required fields
    required_fields = ["level", "file", "format", "handlers"]
    missing_fields = [field for field in required_fields if field not in log_config]
    if missing_fields:
        raise ValueError(
            f"Missing required logging configuration fields: {', '.join(missing_fields)}"
        )

    # Create logger
    logger = logging.getLogger(name)
    logger.handlers.clear()
    logger.setLevel(getattr(logging, log_config["level"]))

    # Create formatters
    formatter = logging.Formatter(log_config["format"])

    # Setup console handler
    if log_config["handlers"]["console"]["enabled"]:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    # Setup file handler
    if log_config["handlers"]["file"]["enabled"]:
        log_file = get_project_root() / log_config["file"]
        log_file.parent.mkdir(parents=True, exist_ok=True)

        file_handler = logging.FileHandler(
            log_file, mode=log_config["handlers"]["file"].get("mode", "w")
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    # Add startup delimiter
    logger.info("=" * 50)
    logger.info("Logging session started")
    logger.info(f"Project root: {get_project_root()}")
    logger.info(f"Log file: {get_project_root() / log_config['file']}")
    logger.info(f"Logger level: {log_config['level']}")
    logger.info("=" * 50)

    return logger


# Package-level logger name
logger_main = "root"

# Public interface and reusability
__all__ = [
    "setup_logger",
    "logger_main",
    "get_project_root",
    "get_config_path",
    "load_config",
]
