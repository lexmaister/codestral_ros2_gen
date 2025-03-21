from pathlib import Path
from codestral_ros2_gen import load_config
from codestral_ros2_gen.generators.base_generator import BaseGenerator, logger
from codestral_ros2_gen.utils.init_pkg_logger import init_pkg_logger


logger = init_pkg_logger()


class ExampleServiceNodeGenerator(BaseGenerator):
    """
    Generator for creating a ROS2 service node.

    Reads input file paths from config["input"] (e.g. interface_file, test_file)
    and constructs a composite prompt with a clear task description.
    """

    def _read_input_file(self, rel_path: str) -> str:
        """Helper to read file content, resolving relative paths with respect to config."""
        # file_path = Path(rel_path)
        # if not file_path.is_absolute():
        #     file_path = self.config_path.parent / file_path
        with open(rel_path, "r") as f:
            return f.read()

    def prepare_prompt(self, **kwargs) -> str:
        try:
            # Define detailed task instructions.
            task_description = (
                "Your task is to generate a ROS2 service node in Python that meets these requirements:\n"
                "  - Implement the service interface as specified in the provided interface file.\n"
                "  - Follow ROS2 best practices (proper error handling, input validation, etc.).\n"
                "  - Pass all tests described in the provided test file.\n"
                "  - Ensure the code is clean, efficient, and well-commented.\n"
            )
            # Read all input files into a dictionary.
            parts = {
                key: self._read_input_file(rel_path)
                for key, rel_path in self.config["input"].items()
            }

            # Build the composite prompt as a single formatted multiline string.
            prompt = (
                f"{task_description}\n"
                f"--- Interface File ---\n"
                f"File: {self.config['input'].get('interface_file', 'N/A')}\n"
                f"{parts.get('interface_file', '')}\n\n"
                f"--- Test File ---\n"
                f"File: {self.config['input'].get('test_file', 'N/A')}\n"
                f"{parts.get('test_file', '')}"
            )
            logger.info("The prompt has been prepared successfully.")
            return prompt
        except Exception as e:
            logger.error(f"Error preparing prompt: {str(e)}")
            raise


def main():
    # Load config from the local config.yaml in the examples folder.
    config_path = Path(__file__).parent / "config.yaml"
    config = load_config(config_path)

    # Create generator instance and assign config.
    generator = ExampleServiceNodeGenerator(config_path=config_path)
    generator.config = config

    # Prepare prompt - also will be run during the generation process. This is here to debug.
    prompt = generator.prepare_prompt(
        service_name="object_height_service", node_name="service_node"
    )
    logger.debug("Generated prompt:")
    logger.debug(prompt)

    # Run generation process.
    generator.run(service_name="object_height_service", node_name="service_node")


if __name__ == "__main__":
    main()
