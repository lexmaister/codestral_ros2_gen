from pathlib import Path
from codestral_ros2_gen import load_config
from codestral_ros2_gen.generators.base_generator import BaseGenerator
from codestral_ros2_gen.utils.init_pkg_logger import init_pkg_logger


logger = init_pkg_logger()


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

            task_description = """
                Your task is to generate a ROS2 node 'network_scanner_node.py' for network scanning that:
                  - Contains a NetworkScannerNode class which acts as a publisher.
                  - Accepts two runtime parameters:
                      - network (string): The target network to scan. This value is required and must not be empty.
                      - scan_period (int): The number of seconds between each repeated scan execution.
                      This value is required and must be greater than 0.
                  - Node must validate the input parameters and exit with error code 1 when provided invalid parameters.
                  - Executes the compiled nscan binary (subprocess.Popen(['nscan', network]) to perform network scanning.
                  - The node should call the binary as a subprocess, pass the network target as an argument,
                  capture its JSON output from '/tmp/nscan_results.json', and parse it to obtain the scan results.
                JSON example:
                {
                '8.8.8.8': {
                    'state': 'UP',
                    'response_time_ms': 1,
                    'error': null
                   },
                }
                  - If the JSON is not valid, node must log the error and exit with error code 2 without publishing any messages.
                  - Implements a loop that invokes the nscan binary every scan_period seconds. If a scan takes longer than scan_period,
                    the node should wait for the current scan to finish before starting a new one to avoid overlapping scans.
                  - Sends the parsed scan results as a ROS2 message on the 'network_status' topic.
                  - The message should be of type NetworkStatus, which is defined in the provided message files.
                  - It also must check for time JSON was last modified and if it is older than 3*scan_period
                it must send an empty addresses list using the NetworkStatus message until the JSON is updated.
                  - Node should log each its step (start, sending scan results, etc.) and any errors using the ros2 logger.
                  - Uses the ROS2 clock to set the timestamp in the message and adheres to ROS2 best practices.
            """

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

    # Run the generation process.
    generator.run(scanner_name="network_scanner")


if __name__ == "__main__":
    main()
