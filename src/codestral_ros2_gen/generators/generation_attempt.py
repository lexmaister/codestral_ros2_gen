import enum
import time
from pathlib import Path
from functools import wraps
import logging
from dataclasses import dataclass, asdict
from typing import Any, Optional, Callable
import pandas as pd

from codestral_ros2_gen import logger_main
from codestral_ros2_gen.models.mistral_client import MistralClient, ModelUsage
from codestral_ros2_gen.utils.code_parser import ROS2CodeParser
from codestral_ros2_gen.utils.ros2_runner import ROS2Runner


logger = logging.getLogger(f"{logger_main}.{__name__.split('.')[-1]}")


@dataclass
class AttemptMetrics:
    """
    Dataclass representing metrics for a single generation attempt.

    This class stores and provides access to various performance metrics collected during
    a code generation attempt, including timing, success status, test results, and token usage.

    Attributes:
        attempt_time (float): Total time in seconds taken for this attempt.
        success (bool): Final state of the attempt (True for SUCCESS, False for FAILURE).
        final_state (str): Final state of the attempt: SUCCESS or other in case of FAILURE.
        tests_passed (int): Number of tests passed.
        tests_failed (int): Number of tests failed.
        tests_skipped (int): Number of tests skipped.
        prompt_tokens (int): Number of tokens used in the prompt.
        completion_tokens (int): Number of tokens used in the completion.
        total_tokens (int): Total number of tokens used.
        error (Optional[str]): Error message if any error occurred in case of FAILURE.

    Internal Attributes:
        _fields_order (list[str]): Internal list defining the order of fields for reports.
    """

    attempt_time: float
    success: bool
    final_state: str
    tests_passed: int
    tests_failed: int
    tests_skipped: int
    prompt_tokens: int
    completion_tokens: int
    total_tokens: int
    error: Optional[str] = None

    _fields_order = [
        "attempt_time",
        "success",
        "final_state",
        "tests_passed",
        "tests_failed",
        "tests_skipped",
        "prompt_tokens",
        "completion_tokens",
        "total_tokens",
        "error",
    ]

    @property
    def as_dict(self) -> dict[str, Any]:
        """
        Convert metrics to a dictionary.

        Returns:
            dict[str, Any]: Dictionary representation of all metrics.
        """
        return asdict(self)

    @property
    def as_series(self) -> pd.Series:
        """
        Convert metrics to a pandas Series with proper ordering.

        Returns:
            pd.Series: Series containing all metrics with field names as index.
        """
        s = pd.Series(
            {field: self.as_dict[field] for field in self._fields_order},
            name="value",
        )
        s.index.name = "metric"
        return s

    def __str__(self) -> str:
        """
        Return a formatted string with all attempt metrics in table format.

        Creates a markdown-formatted table of metrics, excluding the error message.

        Returns:
            str: Formatted metrics table excluding error message.
        """
        s = self.as_series[:-1].copy()
        s["attempt_time"] = round(s["attempt_time"], 1)
        s["success"] = str(s["success"])
        return s.to_markdown(tablefmt="simple")


class AttemptState(enum.Enum):
    """
    Enum representing valid states during a generation attempt.

    This enum defines the possible states a generation attempt can be in,
    from initialization through success or failure.

    Attributes:
        INITIALIZE (int): Starting state for the attempt.
        GENERATE (int): Code generation is in progress.
        PARSE (int): Parsing the generated code.
        SAVE (int): Saving the parsed code to disk.
        TEST (int): Running tests on the saved code.
        SUCCESS (int): Generation attempt succeeded.
        FAILURE (int): Generation attempt failed.
    """

    INITIALIZE = 1
    GENERATE = 2
    PARSE = 3
    SAVE = 4
    TEST = 5
    SUCCESS = 6
    FAILURE = 7


def attempt_state(func) -> Callable | None:
    """
    Decorator for methods representing a state in GenerationAttempt.

    This decorator wraps methods in the GenerationAttempt class to:
    1. Catch exceptions and set error state.
    2. Log errors when they occur.
    3. Transition to FAILURE state on error.

    When an exception occurs in the wrapped method, the decorator sets the error
    message, logs the error, transitions to FAILURE state, and returns None.

    Args:
        func (Callable): The method to wrap.

    Returns:
        Callable | None: Wrapped method with error handling and state transition logic, or None if exception occurs.

    Example:
        .. code-block:: python

            @attempt_state
            def _some_state_method(self):
                # Method implementation that might raise an exception
                # If exception occurs, state transitions to FAILURE
                # and the decorator returns None
    """

    @wraps(func)
    def wrapper(self, *args, **kwargs):
        try:
            return func(self, *args, **kwargs)
        except Exception as e:
            logger.error(
                f"Exception on stage {self.previous_state.name}:\n\n{str(e)}\n"
            )
            self.error = str(e)
            self.state = AttemptState.FAILURE
            return None

    return wrapper


class GenerationAttempt:
    """
    A state machine that orchestrates a single generation attempt using Mistral AI.

    This class handles the complete lifecycle of a code generation attempt, including:
    - Generating code by interacting with the Mistral client.
    - Parsing generated code with ROS2CodeParser.
    - Saving parsed code using a provided callback.
    - Running tests on the generated code using ROS2Runner.

    The class implements a state machine pattern to manage transitions between
    different phases of the generation process.

    Attributes:
        model (MistralClient): Instance of the Mistral client for code generation.
        config (dict): Configuration dictionary including generation timeout and test settings.
        state (AttemptState): Current state of the attempt.
        previous_state (Optional[AttemptState]): Previous state of the attempt.
        start_time (float): Timestamp when the attempt started.
        generated_code (Optional[str]): Raw code generated by the model.
        parsed_code (Optional[str]): Parsed and optionally formatted code.
        usage (Optional[ModelUsage]): Model usage information.
        test_counts (tuple[int, int, int]): Tuple of test counts (passed, failed, skipped).
        error (Optional[str]): Error description if the attempt failed.
    """

    def __init__(self, model: MistralClient, config: dict):
        """
        Initialize a GenerationAttempt.

        Sets up the state machine with the given model and configuration.

        Args:
            model (MistralClient): Mistral client instance for code generation.
            config (dict): Configuration settings including generation timeout and test settings.
        """
        self.model = model
        self.config = config
        self.state = AttemptState.INITIALIZE
        self.previous_state: AttemptState = None
        self.start_time = time.time()
        self.generated_code = None
        self.parsed_code = None
        self.usage: ModelUsage = None
        self.test_counts: tuple[int] = (0, 0, 0)
        self.error = None

    def run(
        self, output_path: Path, prompt: str, save_callback
    ) -> tuple[bool, AttemptMetrics]:
        """
        Run the complete generation attempt process.

        Executes the full generation cycle through all states (initialize, generate,
        parse, save, test) and returns success status and metrics.

        Args:
            output_path (Path): Destination file path for the generated code.
            prompt (str): The prompt to generate code from.
            save_callback (Callable[[str, Path], bool]): Function that saves the code to the specified path.

        Returns:
            tuple[bool, AttemptMetrics]: Tuple containing success status (bool) and metrics (AttemptMetrics).
        """
        while self.state not in (AttemptState.SUCCESS, AttemptState.FAILURE):
            self.previous_state = self.state
            logger.info(f"Entering state: {self.state.name}")
            if self.state == AttemptState.INITIALIZE:
                self._initialize()
            elif self.state == AttemptState.GENERATE:
                self._generate(prompt)
            elif self.state == AttemptState.PARSE:
                self._parse()
            elif self.state == AttemptState.SAVE:
                self._save(output_path, save_callback)
            elif self.state == AttemptState.TEST:
                self._test()
            # each stage should set new self.state itself
            logger.info(
                f"Exiting state: {self.previous_state.name} -> New state: {self.state.name}"
            )
            if self.state == self.previous_state:
                logger.error(
                    f"State did not change from {self.previous_state.name}. Terminating attempt."
                )
                self.error = f"State {self.previous_state.name} did not transition."
                self.state = AttemptState.FAILURE
                break

        overall_time = time.time() - self.start_time
        success = self.state == AttemptState.SUCCESS
        metrics = AttemptMetrics(
            attempt_time=overall_time,
            success=success,
            final_state=self.state.name if success else self.previous_state.name,
            tests_passed=self.test_counts[0],
            tests_failed=self.test_counts[1],
            tests_skipped=self.test_counts[2],
            prompt_tokens=self.usage.prompt_tokens if self.usage else 0,
            completion_tokens=self.usage.completion_tokens if self.usage else 0,
            total_tokens=self.usage.total_tokens if self.usage else 0,
            error=self.error,
        )
        return success, metrics

    @attempt_state
    def _initialize(self):
        """
        Initialize the attempt by clearing errors and preparing for generation.

        This method is the entry point for the state machine, resetting the error
        state and transitioning to the GENERATE state.

        Raises:
            RuntimeError: If initialization fails.
        """
        logger.info("Initializing...")
        self.error = None
        self.state = AttemptState.GENERATE

    @attempt_state
    def _generate(self, prompt: str):
        """
        Generate code using the Mistral client.

        Calls the Mistral client's complete() method with the provided prompt
        and records the generated code and model usage details.

        Args:
            prompt (str): The prompt to generate code from.

        Raises:
            RuntimeError: If code generation fails or times out.
        """
        logger.info("Generating code...")
        generated_code, usage = self.model.complete(prompt)
        logger.info("Code generated successfully.")
        self.generated_code = generated_code
        self.usage: ModelUsage = usage
        logger.info(
            f"Model usage: prompt_tokens={usage.prompt_tokens}, "
            f"completion_tokens={usage.completion_tokens}, total_tokens={usage.total_tokens}"
        )
        self.state = AttemptState.PARSE

    @attempt_state
    def _parse(self):
        """
        Parse the generated code.

        Uses ROS2CodeParser to extract and optionally format the generated code.
        Transitions to the SAVE state if parsing succeeds.

        Raises:
            RuntimeError: If parsing fails or no valid code is extracted.
        """
        logger.info("Parsing generated code...")
        code = ROS2CodeParser.parse(self.generated_code)
        if not code:
            raise RuntimeError("Parsing failed, no code returned.")
        else:
            self.parsed_code = code
            logger.info("Parsing succeeded.")
            self.state = AttemptState.SAVE

    @attempt_state
    def _save(self, output_path: Path, save_callback):
        """
        Save the parsed code to a file.

        Invokes the provided save_callback with the parsed code and output path.
        Transitions to the TEST state if saving succeeds.

        Args:
            output_path (Path): Destination file path for the generated code.
            save_callback (Callable[[str, Path], bool]): Function that saves the code to the specified path.

        Raises:
            RuntimeError: If the save operation fails.
        """
        logger.info("Saving code...")
        if not save_callback(self.parsed_code, output_path):
            raise RuntimeError("Saving failed.")
        else:
            logger.info("Code saved successfully.")
            self.state = AttemptState.TEST

    @attempt_state
    def _test(self):
        """
        Execute tests on the saved code using ROS2Runner.

        Retrieves test configuration from the config and runs tests on the generated code.
        Transitions to SUCCESS if tests pass, or FAILURE if they fail.

        Raises:
            RuntimeError: If test configuration is missing or tests fail.
        """
        logger.info("Running tests...")
        test_section = self.config.get("test")
        if (
            not test_section
            or "ws_setup" not in test_section
            or "node_command" not in test_section
            or "test_command" not in test_section
            or "test_timeout" not in test_section
        ):
            raise RuntimeError(
                "Missing required test configuration (ws_setup, node_command, test_command or test_timeout)."
            )

        ws_setup = test_section["ws_setup"]
        node_command = f"{ws_setup} && {test_section['node_command']}"
        test_command = f"{ws_setup} && {test_section['test_command']}"
        logger.info(f"Built node command: {node_command}")
        logger.info(f"Built test command: {test_command}")

        test_runner = ROS2Runner(
            node_command=node_command,
            test_command=test_command,
            test_timeout=test_section["test_timeout"],
        )
        success, test_logs, self.test_counts = test_runner.run()
        logger.debug(f"Test logs:\n{test_logs}")

        if success:
            logger.info("Tests passed successfully.")
            self.state = AttemptState.SUCCESS
        else:
            logger.warning("Tests failed.")
            self.error = f"Tests failed. Logs:\n{test_logs}"
            self.state = AttemptState.FAILURE
