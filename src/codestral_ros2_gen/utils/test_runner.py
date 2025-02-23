import subprocess
import time
import os
import signal
import logging

from codestral_ros2_gen import logger_main


logger = logging.getLogger(f"{logger_main}.{__name__.split('.')[-1]}")


class TestRunner:
    """
    TestRunner encapsulates methods to launch a ROS2 node, run pytest tests in the ROS2 environment,
    capture the test output for analysis, and terminate the node.
    """

    def __init__(self, node_command: str, test_command: str):
        """
        Initialize the TestRunner.

        Args:
            node_command (str): Shell command to launch the ROS2 node.
            test_command (str): Shell command to run tests (pytest).
        """
        self.node_command = node_command
        self.test_command = test_command
        self.node_process = None
        self.test_output = ""

    def start_node(self) -> None:
        """
        Start the ROS2 node using the node_command.
        """
        logger.info("Starting the ROS2 node...")
        self.node_process = subprocess.Popen(
            self.node_command, shell=True, executable="/bin/bash"
        )
        time.sleep(2)  # Allow time for the node to start

    def run_tests(self) -> int:
        """
        Run pytest using the test_command and print output live.

        Returns:
            int: The return code of the test command.
        """
        logger.info("Running tests with live output...")
        # Start the test process with live output (line-buffered)
        process = subprocess.Popen(
            self.test_command,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

        stdout_lines = []
        stderr_lines = []

        # Stream stdout live
        if process.stdout:
            for line in iter(process.stdout.readline, ""):
                print(line, end="")  # live printing to stdout
                stdout_lines.append(line)
            process.stdout.close()

        # Stream stderr live (if needed)
        if process.stderr:
            for line in iter(process.stderr.readline, ""):
                print(line, end="")  # live printing to stdout (or use sys.stderr)
                stderr_lines.append(line)
            process.stderr.close()

        return_code = process.wait()
        self.test_output = (
            f"stdout:\n{''.join(stdout_lines)}\nstderr:\n{''.join(stderr_lines)}"
        )
        logger.info("Test output captured.")
        return return_code

    def kill_node(self) -> None:
        """
        Terminate the running ROS2 node cleanly, and force-kill if necessary.
        """
        if self.node_process is not None:
            logger.info("Terminating node process...")
            try:
                self.node_process.terminate()
                self.node_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning("Node did not shutdown gracefully; force killing.")
                os.kill(self.node_process.pid, signal.SIGKILL)
            self.node_process = None

    def run(self) -> tuple[bool, str]:
        """
        Execute the full test run: start node, run tests, and terminate the node.

        Returns:
            (bool, str): Tuple with overall test success status and captured test output.
        """
        self.start_node()
        return_code = self.run_tests()
        self.kill_node()
        success = return_code == 0
        return success, self.test_output
