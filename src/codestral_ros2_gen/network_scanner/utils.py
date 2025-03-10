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
        from codestral_ros2_gen import logger_main, logging

        default_logger = logging.getLogger(f"{logger_main}.network_scan")

    except ImportError:
        default_logger = None  # should work with exteral logger

    finally:
        return default_logger
