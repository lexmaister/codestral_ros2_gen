"""Utilities for network scanning operations.
This module provides utility functions for network scanning operations,
such as
"""

import logging


def get_codestral_ros2_gen_logger() -> logging.Logger | None:
    """Get the codestral_ros2_gen logger for the network scanner.

    Tries to import codestral_ros2_gen logger, if not available returns None.

    Returns:
        logging.Logger | None: The codestral_ros2_gen logger for the network scanner.
    """
    try:
        #     from codestral_ros2_gen import logger_main, setup_logger

        #     try:
        #         default_logger = setup_logger(f"{logger_main}.network_scan")
        #     except (FileNotFoundError, ValueError) as e:
        #         print(f"Failed to initialize logger: {str(e)}")
        #         raise
        #     except Exception as e:
        #         print(f"Unexpected error during logger initialization: {str(e)}")
        #         raise
        default_logger = logging.getLogger("codestral_ros2_gen.network_scan")

    except ImportError:
        default_logger = None  # should work with exteral logger

    finally:
        return default_logger
