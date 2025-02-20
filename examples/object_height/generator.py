from pathlib import Path
import subprocess
from typing import Optional

from codestral_ros2_gen import logger
from codestral_ros2_gen.generators.base_generator import BaseGenerator


class ObjectHeightGenerator(BaseGenerator):
    """Generator for ObjectHeight service implementation."""

    def __init__(
        self,
        config_path: Optional[Path] = None,
        metrics_file: Optional[str] = None,
        api_key: Optional[str] = None,
    ):
        """Initialize generator with config."""
        super().__init__(
            config_path=config_path, metrics_file=metrics_file, api_key=api_key
        )

    def prepare_prompt(self, **kwargs) -> str:
        """Prepare generation prompt with service definition and test cases."""
        ws_path = Path.cwd()

        try:
            # Validate required config paths
            for path_key in ["interface_file", "test_file"]:
                if path_key not in self.config["paths"]:
                    raise KeyError(f"Missing required path in config: {path_key}")

            srv_path = ws_path / self.config["paths"]["interface_file"]
            test_path = ws_path / self.config["paths"]["test_file"]

            # Validate files exist
            if not srv_path.exists():
                raise FileNotFoundError(f"Service definition not found: {srv_path}")
            if not test_path.exists():
                raise FileNotFoundError(f"Test file not found: {test_path}")

            # Read files
            srv_def = srv_path.read_text().strip()
            test_code = test_path.read_text().strip()

            # Get system prompt from config or use default
            system_prompt = self.config.get("model", {}).get(
                "system_prompt",
                "You are a ROS2 expert. Generate clean, efficient code that follows ROS2 best practices.",
            )

            # Use triple backticks on separate lines and strip extra indentation.
            prompt = f"""{system_prompt}

1. Service Definition:
```
{srv_def}
```

2. Test Cases:
```python
{test_code}
```

Generate a complete implementation that:
- Uses proper ROS2 Python API
- Includes all necessary imports
- Handles edge cases (including invalid inputs)
- Follows ROS2 service implementation best practices
- Correctly processes input units and provides output in millimeters
- Passes all the provided test cases
"""
            return prompt.strip()

        except Exception as e:
            logger.error(f"Error preparing prompt: {e}")
            raise

    def save_output(self, code: str, output_path: Path) -> bool:
        """Save generated code and verify it's importable in ROS2 workspace."""
        try:
            # Validate workspace structure
            ws_path = Path.cwd()
            if not (ws_path / "install").exists():
                logger.error(f"ROS2 workspace not found at {ws_path}")
                return False

            # Ensure output directory exists
            output_path.parent.mkdir(parents=True, exist_ok=True)

            # Save new implementation
            output_path.write_text(code)
            logger.info(f"Saved service implementation to {output_path}")

            # Get package name from config or path
            package_name = output_path.parent.parent.name

            # Rebuild package
            build_cmd = f"cd {ws_path} && colcon build --packages-select {package_name}"
            result = subprocess.run(
                build_cmd, shell=True, check=True, capture_output=True, text=True
            )
            logger.info("Successfully rebuilt package")
            return True

        except subprocess.CalledProcessError as e:
            logger.error(f"Build failed: {e.stderr}")
            return False
        except Exception as e:
            logger.error(f"Failed to save and build code: {e}")
            return False


def main():
    """Run generator within ROS2 workspace context."""
    try:
        config_path = Path(__file__).parent / "config.yaml"
        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        generator = ObjectHeightGenerator(config_path=config_path)
        ws_path = Path.cwd()
        output_path = ws_path / generator.config["paths"]["output_file"]

        success, _ = generator.generate(
            output_path=output_path,
            max_attempts=generator.config["generation"]["max_attempts"],
            timeout=generator.config["generation"]["timeout"],
        )

        logger.info(
            "Service generation successful!"
            if success
            else "Service generation failed!"
        )
        return 0 if success else 1

    except Exception as e:
        logger.error(f"Generator failed: {e}")
        return 1


if __name__ == "__main__":
    exit(main())
