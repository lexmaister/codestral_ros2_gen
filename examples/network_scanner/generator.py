from pathlib import Path
from codestral_ros2_gen import load_config, logger
from codestral_ros2_gen.generators.base_generator import BaseGenerator


class ExampleNetworkScannerGenerator(BaseGenerator):
    """
    Generator for creating a ROS2 network scanner node.

    Reads input file paths from config["input"] (e.g. interface_file, test_file)
    and constructs a composite prompt with a clear task description.
    """

    def prepare_prompt(self, **kwargs) -> str:
        try:
            # Read the input file paths from configuration.
            msg1_path = self.config["input"].get("msg_1_file", "N/A")
            msg2_path = self.config["input"].get("msg_2_file", "N/A")
            test_path = self.config["input"].get("test_file", "N/A")

            # Read file contents.
            with open(msg1_path, "r") as f:
                msg1_content = f.read()
            with open(msg2_path, "r") as f:
                msg2_content = f.read()
            with open(test_path, "r") as f:
                test_content = f.read()

            # Updated task description.
            task_description = (
                "Your task is to generate a ROS2 node 'network_scanner_node.py' for network scanning that:\n"
                "  - Contains NetworkScannerNode class describing publisher.\n"
                "  - Accepts two runtime parameters:\n"
                "      - network (string): The target network to scan - using in scan method.\n"
                "This value is required and must not be empty. Default is 8.8.8.8\n"
                "      - scan_period (int): The number of seconds between each repeated scan execution.\n"
                "This value is required and must be greater than 0. Default is 10.\n"
                "  - Imports the NetworkScanner class from codestral_ros2_gen.network_scanner.network_scanner.\n"
                "  - Instantiates a NetworkScanner object and calls its scan method, e.g., scan('192.168.1.0/24').\n"
                "    Example usage:\n"
                "      from codestral_ros2_gen.network_scanner.network_scanner import NetworkScanner\n"
                "      scanner = NetworkScanner()\n"
                "      results = scanner.scan('192.168.1.0/24')\n"
                "  - Expects the scan method to return a dictionary mapping IP addresses to a dict containing at least:\n"
                "      - state (e.g., 'UP' or 'DOWN'),\n"
                "      - response_time_ms (integer or None), and\n"
                "      - error (string, if any).\n"
                "  - Implements a loop that calls scan() every scan_period seconds. If a scan takes longer than scan_period,\n"
                "    the node should buffer and avoid overlapping scans by waiting for the current scan to finish before starting a new one.\n"
                "  - Sends the scan results as a ROS2 message on the 'network_status' topic.\n"
                "  - Logs the scan results and any errors using the node's logger.\n"
                "  - Publishes the scan results every scan_period seconds.\n"
                "  - Uses the ROS2 clock to set the timestamp in the message.\n"
                "  - Uses proper error handling.\n"
                "  - Adheres to ROS2 best practices."
            )

            # Build composite prompt.
            prompt = (
                f"{task_description}\n\n"
                f"--- Message File 1 ---\n"
                f"File: {msg1_path}\n"
                f"{msg1_content}\n\n"
                f"--- Message File 2 ---\n"
                f"File: {msg2_path}\n"
                f"{msg2_content}\n\n"
                f"--- Test File ---\n"
                f"File: {test_path}\n"
                f"{test_content}"
            )

            logger.info(f"The prompt has been prepared successfully:\n{prompt}")
            return prompt
        except Exception as e:
            logger.error(f"Error preparing prompt: {str(e)}")
            raise


def main():
    # Load configuration from the local config.yaml in the network_scanner example directory.
    config_path = Path(__file__).parent / "config.yaml"
    config = load_config(config_path)

    # Create generator instance and assign configuration.
    generator = ExampleNetworkScannerGenerator(config_path=config_path)
    generator.config = config

    # Prepare the prompt (for debugging or inspection purposes).
    prompt = generator.prepare_prompt(scanner_name="network_scanner")
    logger.debug("Generated prompt:")
    logger.debug(prompt)

    # Run the generation process.
    generator.run(scanner_name="network_scanner")


if __name__ == "__main__":
    main()
