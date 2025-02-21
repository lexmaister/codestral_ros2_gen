import enum
import time
import pytest
import subprocess
from pathlib import Path
from functools import wraps
import logging

from codestral_ros2_gen import logger_main
from codestral_ros2_gen.models.mistral_client import MistralClient, ModelUsage
from codestral_ros2_gen.utils.code_parser import ROS2CodeParser

logger = logging.getLogger(f"{logger_main}.{__name__.split('.')[-1]}")


def attempt_state(func):
    """
    Decorator for state methods in GenerationAttempt specific to Mistral client.

    Wraps the execution of a state method in a try/except block.
    On exception, it logs the error, stores the error message in self.error,
    and sets the attempt state to FAILURE.
    """

    @wraps(func)
    def wrapper(self, *args, **kwargs):
        try:
            return func(self, *args, **kwargs)
        except Exception as e:
            logger.error(f"Exception in {func.__name__}: {str(e)}")
            self.error = str(e)
            self.state = AttemptState.FAILURE

    return wrapper


class AttemptState(enum.Enum):
    """
    Enum representing the states of a generation attempt.
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
    State machine representing a single generation attempt using Mistral client.

    This class handles:
      - Generating code via the model.complete() method.
      - Parsing the generated code.
      - Saving the parsed code using a provided callback.
      - Running tests on the saved code.

    It returns metrics including the attempt's duration, final state, and any error message.
    """

    def __init__(self, model: MistralClient, config: dict):
        """
        Initialize a GenerationAttempt instance.

        Args:
            model (MistralClient): The Mistral AI model client.
            config (dict): Configuration settings (e.g., generation timeout).
        """
        self.model = model
        self.config = config
        self.state = AttemptState.INITIALIZE
        self.start_time = time.time()
        self.generated_code = None
        self.parsed_code = None
        self.usage = None
        self.error = None

    def run(self, output_path: Path, prompt: str, save_callback) -> tuple[bool, dict]:
        """
        Run the state machine for a single generation attempt.

        Args:
            output_path (Path): The target path to save the generated code.
            prompt (str): The prompt for code generation.
            save_callback (callable): Function (code, output_path) -> bool for saving code.

        Returns:
            tuple[bool, dict]: A boolean indicating success (state SUCCESS) and a dictionary containing:
                - "attempt_time": Duration of the attempt.
                - "final_state": The final state name ("SUCCESS" or "FAILURE").
                - "error": Any error message (or None if no error).
        """
        while self.state not in (AttemptState.SUCCESS, AttemptState.FAILURE):
            current_state = self.state
            logger.info(f"Entering state: {current_state.name}")
            if current_state == AttemptState.INITIALIZE:
                self._initialize()
            elif current_state == AttemptState.GENERATE:
                self._generate(prompt)
            elif current_state == AttemptState.PARSE:
                self._parse()
            elif current_state == AttemptState.SAVE:
                self._save(output_path, save_callback)
            elif current_state == AttemptState.TEST:
                self._test(output_path)
            logger.info(
                f"Exiting state: {current_state.name} -> New state: {self.state.name}"
            )
            if self.state == current_state:
                logger.error(
                    f"State did not change from {current_state.name}. Terminating attempt."
                )
                self.error = f"State {current_state.name} did not transition."
                self.state = AttemptState.FAILURE
                break
        overall_time = time.time() - self.start_time
        return self.state == AttemptState.SUCCESS, {
            "attempt_time": overall_time,
            "final_state": self.state.name,
            "error": self.error,
        }

    @attempt_state
    def _initialize(self):
        """
        Attempt-specific initialization.

        Clears any previous error and sets the state to GENERATE.
        (No extra model initialization is necessary for Mistral.)
        """
        logger.info("Initializing...")
        self.error = None  # Clear previous errors.
        self.state = AttemptState.GENERATE

    @attempt_state
    def _generate(self, prompt: str):
        """
        Generate code using the Mistral model.

        Uses model.complete() with the given prompt. Assumes that usage is properly
        returned as a ModelUsage instance. Checks generation time against configured timeout.

        Args:
            prompt (str): The prompt for code generation.
        """
        logger.info("Generating code...")
        t0 = time.time()
        generated_code, usage = self.model.complete(prompt)
        elapsed = time.time() - t0
        logger.info(f"Generated code in {elapsed:.2f} sec")
        if elapsed > self.config["generation"]["timeout"]:
            logger.warning("Generation timed out")
            self.error = "Generation timed out"
            self.state = AttemptState.FAILURE
            return
        self.generated_code = generated_code
        self.usage = usage
        logger.info(
            f"Model usage: prompt_tokens={usage.prompt_tokens}, "
            f"completion_tokens={usage.completion_tokens}, total_tokens={usage.total_tokens}"
        )
        self.state = AttemptState.PARSE

    @attempt_state
    def _parse(self):
        """
        Parse the generated code.

        Uses ROS2CodeParser to process the generated code.
        If parsing fails, sets state to FAILURE; otherwise, transitions to SAVE.
        """
        logger.info("Parsing generated code...")
        code = ROS2CodeParser.parse(self.generated_code)
        if not code:
            logger.warning("Parsing failed")
            self.error = "Parsing failed"
            self.state = AttemptState.FAILURE
        else:
            self.parsed_code = code
            logger.info("Parsing succeeded")
            self.state = AttemptState.SAVE

    @attempt_state
    def _save(self, output_path: Path, save_callback):
        """
        Save the parsed code to disk.

        Uses the provided save_callback to handle file I/O.
        If saving fails, sets state to FAILURE; otherwise, transitions to TEST.

        Args:
            output_path (Path): The desired file path to save code.
            save_callback (callable): Function to persist code (code, output_path) -> bool.
        """
        logger.info("Saving code...")
        if not save_callback(self.parsed_code, output_path):
            logger.warning("Saving failed")
            self.error = "Saving failed"
            self.state = AttemptState.FAILURE
        else:
            logger.info("Code saved successfully")
            self.state = AttemptState.TEST

    @attempt_state
    def _test(self, output_path: Path):
        """
        Run tests on the saved code.

        Executes pytest on the parent directory of output_path.
        Successful tests transition the state to SUCCESS, otherwise set to FAILURE.

        Args:
            output_path (Path): Location of the saved code for test context.
        """
        logger.info("Running tests...")
        result = pytest.main([str(output_path.parent)])
        if result == pytest.ExitCode.OK:
            logger.info("Tests passed successfully")
            self.state = AttemptState.SUCCESS
        else:
            logger.warning("Tests failed")
            self.error = "Tests failed"
            self.state = AttemptState.FAILURE
