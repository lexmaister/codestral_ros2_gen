# Use codestral_ros2_gen pkg logger if possible
try:
    from codestral_ros2_gen import logger_main

    nscan_logger = f"{logger_main}.nscan"

except ImportError:
    nscan_logger = None
    print("Cannot import codestral_ros2_gen logger, using external logger")

except Exception as e:
    raise RuntimeError(f"Failed to initialize logger: {str(e)}")
