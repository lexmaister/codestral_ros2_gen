import enum
import time
from pathlib import Path
from functools import wraps
import logging
from dataclasses import dataclass, asdict
from typing import Any, Optional
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
        return asdict(self)

    @property
    def to_series(self) -> pd.Series:
        # Create series with specific order
        return pd.Series({field: self.as_dict[field] for field in self._fields_order})

    def __str__(self) -> str:
        """
        Return a formatted string of the attempt metrics.

        Returns:
            str: Formatted string with final state, time (rounded to 0.1 sec), and error.
        """
        return (
            f"Success:\t\t{self.success}\n"
            + f"Time taken:\t\t{self.attempt_time:.1f} sec\n"
            + f"Final state:\t\t{self.final_state}\n"
            + f"Tests passed:\t\t{self.tests_passed}\n"
            + f"Tests failed:\t\t{self.tests_failed}\n"
            + f"Tests skipped:\t\t{self.tests_skipped}\n"
            + f"Prompt tokens:\t\t{self.prompt_tokens}\n"
            + f"Completion tokens:\t{self.completion_tokens}\n"
            + f"Total tokens:\t\t{self.total_tokens}\n"
        )


def attempt_state(func):
    """
    Decorator for methods representing a state in GenerationAttempt.

    Catches exceptions, logs the error, sets the error message in instance attribute 'error'
    and forces the attempt state to FAILURE.

    Args:
        func (Callable): The state method to execute.

    Returns:
        Callable: Wrapped state method.
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

    return wrapper


class AttemptState(enum.Enum):
    """
    Enum representing valid states during a generation attempt.

    Attributes:
        INITIALIZE: Starting state.
        GENERATE: Code generation in-progress.
        PARSE: Parsing the generated code.
        SAVE: Saving the parsed code to disk.
        TEST: Running tests on the saved code.
        SUCCESS: Generation attempt succeeded.
        FAILURE: Generation attempt failed.
    """

    INITIALIZE = 1
    GENERATE = 2
    PARSE = 3
    SAVE = 4
    TEST = 5
    SUCCESS = 6
    FAILURE = 7


class GenerationAttempt:
    """
    A state machine that orchestrates a single generation attempt using Mistral AI.

    This class handles:
      - Generating code by interacting with the Mistral client's complete() method.
      - Parsing generated code via the ROS2CodeParser.
      - Saving parsed code using a provided callback.
      - Running tests on the generated code using the ROS2Runner.

    The overall attempt result along with metrics is returned as an AttemptMetrics object.

    Attributes:
        model (MistralClient): Instance of the Mistral client for code generation.
        config (dict): Configuration dict including generation timeout and test settings.
        state (AttemptState): Current state of the attempt.
        previous_state (Optional[AttemptState]): Previous state of the attempt.
        start_time (float): Timestamp when the attempt started.
        generated_code (Optional[str]): Raw code generated by the model.
        parsed_code (Optional[str]): Parsed and optionally formatted code.
        usage (Optional[ModelUsage]): Model usage information.
        test_counts (tuple[int]): Tuple of test counts (passed, failed, skipped).
        error (Optional[str]): Error description if the attempt failed.
    """

    def __init__(self, model: MistralClient, config: dict):
        """
        Initialize a GenerationAttempt.

        Args:
            model (MistralClient): Mistral client instance.
            config (dict): Configuration settings (e.g., generation->timeout, test settings).
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
        Run all stages of the generation attempt sequentially.

        The state machine transitions through the states: INITIALIZE, GENERATE, PARSE, SAVE, and TEST.
        If a state fails to transition or an error occurs, the attempt is terminated and marked as FAILURE.

        Args:
            output_path (Path): Target file path to save the generated code.
            prompt (str): The input prompt for code generation.
            save_callback (callable): Function that accepts code and output_path; returns True on success.

        Returns:
            tuple[bool, AttemptMetrics]: A boolean indicating success (True if SUCCESS state)
                                         and an AttemptMetrics object containing performance data.
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
            prompt_tokens=self.usage.prompt_tokens,
            completion_tokens=self.usage.completion_tokens,
            total_tokens=self.usage.total_tokens,
            error=self.error,
        )
        return success, metrics

    @attempt_state
    def _initialize(self):
        """
        Initialize the attempt by clearing errors and setting the state to GENERATE.
        """
        logger.info("Initializing...")
        self.error = None
        self.state = AttemptState.GENERATE

    @attempt_state
    def _generate(self, prompt: str):
        """
        Generate code using the Mistral client.

        Calls the Mistral client's complete() method and records model usage details.
        Moves the state to PARSE upon successful code generation.

        Args:
            prompt (str): The prompt to generate code.
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

        Utilizes ROS2CodeParser to extract and format the code.
        On success, transitions the state to SAVE.
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

        Invokes the provided save_callback with the parsed code and output_path.
        On success, transitions the state to TEST.

        Args:
            output_path (Path): Destination file path for the generated code.
            save_callback (callable): Function that persists the code.
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

        Retrieves required test configuration (node_command, test_command, and ros2_version) from the config.
        Sources the ROS2 environment and runs tests. If tests pass, the attempt is marked SUCCESS;
        otherwise, it transitions to FAILURE.
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
