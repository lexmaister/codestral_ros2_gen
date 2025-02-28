import subprocess
import time
import os
import signal
import logging
import re
import psutil

from codestral_ros2_gen import logger_main

logger = logging.getLogger(f"{logger_main}.{__name__.split('.')[-1]}")


class ROS2Runner:
    """
    ROS2Runner encapsulates methods to launch a ROS2 node, run pytest tests in the ROS2 environment,
    capture the test output for analysis, and terminate the node.

    Attributes:
        node_command (str): Shell command to launch the ROS2 node.
        test_command (str): Shell command to run tests (pytest).
        test_timeout (int): How many seconds to wait for the process before timing out. Defaults to 30 seconds.
        node_process (subprocess.Popen): The process object for the ROS2 node.
        test_output (str): Captured output from the test command.
        tests_passed (int): Number of tests that passed.
        tests_failed (int): Number of tests that failed.
        tests_skipped (int): Number of tests that were skipped.
    """

    def __init__(self, node_command: str, test_command: str, test_timeout: int = 30):
        """
        Initialize the ROS2Runner.

        Args:
            node_command (str): Shell command to launch the ROS2 node.
            test_command (str): Shell command to run tests (pytest).
            test_timeout (int): How many seconds to wait for the process before timing out. Defaults to 30 seconds.
        """
        self.node_command = node_command
        self.test_command = test_command
        self.test_timeout = test_timeout
        self.node_process = None
        self.test_output = ""
        self.tests_passed = 0
        self.tests_failed = 0
        self.tests_skipped = 0

    def start_node(self) -> None:
        """
        Start the ROS2 node and verify it launched successfully.

        Raises:
            RuntimeError: If the node fails to start or register with ROS2.
        """
        logger.info("Starting the ROS2 node...")
        self.node_process = subprocess.Popen(
            self.node_command,
            shell=True,
            executable="/bin/bash",
            stderr=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )

        # Wait briefly for node to initialize
        time.sleep(2)

        # Check if process is still running
        if self.node_process.poll() is not None:
            # Process terminated - get error output
            stdout, stderr = self.node_process.communicate()
            error_msg = f"Node process terminated immediately.\nStdout: {stdout}\nStderr: {stderr}"
            raise RuntimeError(error_msg)

        logger.info("Node started successfully.")

    def run_tests(self) -> int:
        """
        Run pytest using the test_command and capture the output.

        Returns:
            int: The return code of the test command. If the command times out,
                 it returns 1 by default.
        """
        logger.info("Running tests...")
        process = subprocess.Popen(
            self.test_command,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            stdout, stderr = process.communicate(timeout=self.test_timeout)
            return_code = process.returncode
        except subprocess.TimeoutExpired as e:
            msg = f"Test process timed out after {self.test_timeout} seconds."
            process.kill()  # Terminate the process
            # Optionally catch partial output
            stdout, stderr = e.output, msg
            return_code = 1

        self.test_output = f"stdout:\n{stdout}\nstderr:\n{stderr}"
        logger.info(f"Test output captured:\n{self.test_output}")

        return return_code

    def kill_node(self) -> None:
        """
        Terminate the running ROS2 node cleanly, and force-kill if necessary.
        Uses psutil to find and terminate the actual ROS2 node process, not just the shell.
        """
        if self.node_process is not None:
            shell_pid = self.node_process.pid
            logger.info(
                f"Finding and terminating ROS2 node (shell pid: {shell_pid})..."
            )

            try:
                # Get the shell process
                shell_process = psutil.Process(shell_pid)

                # Find all child processes (includes the ROS2 node)
                children = shell_process.children(recursive=True)

                # Find and terminate actual ROS2 nodes among children
                ros2_nodes_found = False
                for process in children:
                    try:
                        cmd_line = " ".join(process.cmdline())
                        # Look for ROS2 node process - adjust pattern as needed for your specific node
                        if (
                            "ros2 run" in cmd_line
                            or "object_height_service" in cmd_line
                        ):
                            logger.info(
                                f"Found ROS2 node with PID: {process.pid}, cmd: {cmd_line[:60]}..."
                            )
                            ros2_nodes_found = True
                            process.terminate()  # Send SIGTERM
                    except (
                        psutil.NoSuchProcess,
                        psutil.AccessDenied,
                        psutil.ZombieProcess,
                    ) as e:
                        logger.warning(f"Error accessing process: {e}")
                        continue

                # Wait for graceful termination (5 seconds)
                gone, alive = psutil.wait_procs(children, timeout=5)

                # Force kill any remaining processes
                for process in alive:
                    try:
                        logger.warning(
                            f"Process {process.pid} did not terminate gracefully, force killing"
                        )
                        process.kill()  # Send SIGKILL
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass

                # Finally terminate the shell
                if shell_process.is_running():
                    shell_process.terminate()
                    try:
                        shell_process.wait(timeout=3)
                    except psutil.TimeoutExpired:
                        shell_process.kill()

                if not ros2_nodes_found:
                    logger.warning("No ROS2 node processes identified among children")

            except psutil.NoSuchProcess:
                logger.warning(f"Shell process {shell_pid} no longer exists")
            except Exception as e:
                logger.error(f"Error during process termination: {e}")
                # Fallback to original method
                try:
                    self.node_process.terminate()
                    self.node_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    logger.warning("Node did not shutdown gracefully; force killing.")
                    os.kill(self.node_process.pid, signal.SIGKILL)

            self.node_process = None

    def _get_tests_stat(self) -> None:
        """
        Parse the test output to get the number of tests passed, failed, and skipped.
        """
        logger.info("Parsing test output for statistics...")
        for typ in ("passed", "failed", "skipped"):
            m = re.search(rf"(\d+) {typ}.+?in", self.test_output)
            if m:
                setattr(self, f"tests_{typ}", int(m.group(1)))

        logger.info(
            f"Tests failed: {self.tests_failed!r}, passed: {self.tests_passed!r}, skipped: {self.tests_skipped!r}"
        )

    def run(self) -> tuple[bool, str, tuple[int, int, int]]:
        """
        Execute the full test run: start node, run tests, and terminate the node.

        Returns:
            tuple[bool, str, tuple[int, int, int]]: Tuple with overall test success status,
            captured test output and a tuple with the number of tests passed, failed, and skipped.
        """
        try:
            self.start_node()
            return_code = self.run_tests()
            self._get_tests_stat()
        except Exception as e:
            self.test_output = f"Error running tests: {e}"
            return_code = 1
        finally:
            self.kill_node()
            success = return_code == 0
            return (
                success,
                self.test_output,
                (self.tests_passed, self.tests_failed, self.tests_skipped),
            )
