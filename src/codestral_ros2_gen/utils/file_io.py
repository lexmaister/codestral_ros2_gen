import os
from pathlib import Path
from codestral_ros2_gen import logger


def save_code(code: str, output_path: Path) -> bool:
    """
    Save the generated code to the given output path and set executable permissions.

    This function writes the code using basic file I/O and applies chmod +x.

    Args:
        code (str): The generated code to save.
        output_path (Path): The file path where the code will be written.

    Returns:
        bool: True if the code is saved and made executable successfully, False otherwise.
    """
    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w") as f:
            f.write(code)
        os.chmod(output_path, 0o755)  # Make file executable
        logger.info(f"Code successfully saved and made executable: {output_path}")
        return True
    except Exception as e:
        logger.error(f"Error saving code to {output_path}: {str(e)}")
        return False
