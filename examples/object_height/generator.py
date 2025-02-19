from pathlib import Path
from codestral_ros2_gen import logger
from codestral_ros2_gen.generators.base_generator import BaseGenerator
import subprocess


class ObjectHeightGenerator(BaseGenerator):
    """Generator for ObjectHeight service implementation."""

    def prepare_prompt(self, **kwargs) -> str:
        """Prepare generation prompt with service definition and test cases."""
        # Get paths relative to workspace
        ws_path = Path.cwd()  # Now using current directory (test_ws)
        srv_path = ws_path / self.config["paths"]["interface_file"]
        test_path = ws_path / self.config["paths"]["test_file"]

        try:
            with open(srv_path) as f:
                srv_def = f.read().strip()
            with open(test_path) as f:
                test_code = f.read().strip()

            # Format prompt with system prompt from config
            return f"""{self.config['model']['system_prompt']}

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

        except FileNotFoundError as e:
            logger.error(f"Required file not found: {e}")
            raise
        except Exception as e:
            logger.error(f"Error preparing prompt: {e}")
            raise

    def save_output(self, code: str, output_path: Path) -> bool:
        """Save generated code and verify it's importable in ROS2 workspace."""
        try:
            # Using current directory as workspace
            ws_path = Path.cwd()
            if not (ws_path / "install").exists():
                logger.error(f"ROS2 workspace not found at {ws_path}")
                return False

            # Save new implementation
            output_path.write_text(code)
            logger.info(f"Updated service implementation in {output_path}")

            # Rebuild package
            build_cmd = f"cd {ws_path} && colcon build --packages-select object_height"
            result = subprocess.run(build_cmd, shell=True, check=True)
            if result.returncode != 0:
                logger.error("Failed to build package with new implementation")
                return False

            logger.info("Successfully rebuilt package with new implementation")
            return True

        except Exception as e:
            logger.error(f"Failed to save and build code: {e}")
            return False


def main():
    """Run generator within ROS2 workspace context."""
    try:
        # Get config path relative to generator script
        script_path = Path(__file__)
        config_path = script_path.parent / "config.yaml"
        generator = ObjectHeightGenerator(config_path=config_path)

        # Get path to service node using current directory as workspace
        ws_path = Path.cwd()
        output_path = ws_path / generator.config["paths"]["output_file"]

        if not output_path.exists():
            logger.error(f"Service node template not found at {output_path}")
            return 1

        # Run performance evaluation
        logger.info("Starting service generation with evaluation...")
        metrics = generator.evaluate_performance(
            output_path,
            iterations=generator.config["generation"]["evaluation_iterations"],
        )

        # Log results
        logger.info("\nPerformance Evaluation Results:")
        logger.info(f"Total iterations: {metrics['total_iterations']}")
        logger.info(f"Success rate: {metrics['success_rate']*100:.1f}%")
        logger.info(f"Average generation time: {metrics['avg_generation_time']:.2f}s")
        logger.info(f"Average attempts per success: {metrics['avg_attempts']:.1f}")
        if metrics["failures"]:
            logger.warning(f"Failed iterations: {len(metrics['failures'])}")
            for failure in metrics["failures"]:
                logger.warning(
                    f"- Iteration {failure['iteration']}: {failure['error_patterns']}"
                )

        return 0 if metrics["success_rate"] > 0 else 1

    except Exception as e:
        logger.error(f"Generator failed: {e}")
        return 1


if __name__ == "__main__":
    exit(main())
