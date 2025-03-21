from codestral_ros2_gen import logger_main, setup_logger
import logging


def init_pkg_logger() -> logging.Logger:
    """
    Initialize a root logger for a package with the name `logger_name`.

    Returns:
        logging.Logger: The logger for the package.

    Raises:
        SystemExit: If logger initialization fails due to configuration errors or unexpected exceptions.
    """
    try:
        logger = setup_logger(logger_main)
        return logger

    except (FileNotFoundError, ValueError) as e:
        print(f"Failed to initialize logger: {str(e)}")
        raise SystemExit(1)

    except Exception as e:
        print(f"Unexpected error during logger initialization: {str(e)}")
        raise SystemExit(1)
