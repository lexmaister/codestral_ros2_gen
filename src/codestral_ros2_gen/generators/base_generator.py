import time
from pathlib import Path
from typing import Optional, Type
from abc import ABC, abstractmethod

from codestral_ros2_gen import get_config_path, load_config, logger
from codestral_ros2_gen.utils.metrics_handler import MetricsHandler
from codestral_ros2_gen.models.mistral_client import MistralClient
from codestral_ros2_gen.generators.generation_attempt import GenerationAttempt
from codestral_ros2_gen.utils.file_io import save_code
from codestral_ros2_gen.utils.config_utils import validate_config_keys


class BaseGenerator(ABC):
    """
    Abstract base class for code generation modules in the codestral_ros2_gen framework.

    This class implements the template method pattern with a three-phase generation process:
    1. Initialization: Set up the environment, load configuration, and prepare resources.
    2. Generation: Execute the actual code generation with multiple attempts and iterations.
    3. Reporting: Analyze and display metrics about the generation process.

    Derived classes must implement the abstract method `prepare_prompt()` to define
    the specific prompting strategy for their generation task.
    """

    # Default dependencies - can be overridden in tests
    GenerationAttemptClass = GenerationAttempt
    MetricsHandlerClass = MetricsHandler
    ModelClientClass = MistralClient

    def __init__(
        self,
        config_path: Optional[Path] = None,
        generation_attempt_class: Optional[Type] = None,
        metrics_handler_class: Optional[Type] = None,
        model_client_class: Optional[Type] = None,
    ):
        """
        Initialize the generator with configurable dependencies.

        Args:
            config_path (Optional[Path]): Path to the configuration file.
            generation_attempt_class (Optional[Type]): Class to use for generation attempts (for testing).
            metrics_handler_class (Optional[Type]): Class to use for metrics handling (for testing).
            model_client_class (Optional[Type]): Class to use for the AI model client (for testing).
        """
        self.config_path: Path = config_path if config_path else get_config_path()
        self.config: dict = None  # Will be loaded in _initialization_phase
        self.start_time: float = time.time()

        # Allow dependency injection for testing
        if generation_attempt_class:
            self.GenerationAttemptClass = generation_attempt_class
        if metrics_handler_class:
            self.MetricsHandlerClass = metrics_handler_class
        if model_client_class:
            self.ModelClientClass = model_client_class

    @abstractmethod
    def prepare_prompt(self, **kwargs) -> str:
        """
        Prepare the input prompt to be used for code generation.

        This abstract method must be implemented by subclasses to define the specific
        prompting strategy for their generation task.

        Args:
            **kwargs: Additional keyword arguments specific to the generation task (e.g., service_name, node_name for prompting).

        Returns:
            str: A formatted prompt string ready for model input.
        """
        pass

    def _validate_environment(self) -> None:
        """
        Validate that the configuration has a 'generation' section with required keys,
        and ensure that the output directory is accessible.

        Checks for required configuration sections and keys:
        - 'generation' section with 'max_attempts' and 'evaluation_iterations'
        - 'output' section with 'output_file'

        Also attempts to create the output directory if it doesn't exist.

        Raises:
            RuntimeError: If configuration is invalid or output directory is inaccessible.
        """
        # Check that "generation" section contains required keys.
        validate_config_keys(
            self.config,
            "generation",
            ("max_attempts", "evaluation_iterations"),
        )
        # Check that "output" section contains the output file key.
        validate_config_keys(
            self.config,
            "output",
            ("output_file",),
        )
        output_path: Path = Path(self.config["output"]["output_file"])
        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            raise RuntimeError(
                f"Configuration error: Cannot access output directory '{output_path.parent}': {e}"
            )

    def _check_ros2_workspace(self) -> None:
        """
        Check that the current working directory is a valid ROS2 workspace,
        i.e. contains either a 'src' or 'install' directory.

        Raises:
            RuntimeError: If the current directory is not a valid ROS2 workspace.
        """
        cwd: Path = Path.cwd()
        if not ((cwd / "src").exists() or (cwd / "install").exists()):
            raise RuntimeError(
                "Workspace error: Current directory is not a valid ROS2 workspace (missing 'src' or 'install')."
            )

    def _initialization_phase(self) -> None:
        """
        Execute the initialization phase of the generation process.

        This phase:
        1. Loads the configuration from the specified file.
        2. Validates the environment and configuration.
        3. Initializes the metrics handler and the model client.

        Raises:
            RuntimeError: If there's an issue with configuration or initialization.
        """
        logger.info("Phase: INITIALIZATION -> Loading configuration.")
        self.config = load_config(self.config_path)
        self._validate_environment()
        self._check_ros2_workspace()
        try:
            # Use the metrics_file value from the "metrics" section,
            # cleanup it first if it exists.
            metrics_file = self.config["metrics"]["metrics_file"]
            if Path(metrics_file).exists():
                Path(metrics_file).unlink()

            self.metrics_handler = self.MetricsHandlerClass(metrics_file=metrics_file)
            self.model = self.ModelClientClass(config=self.config)
            self.max_attempts = self.config["generation"]["max_attempts"]
            self.evaluation_iterations = self.config["generation"][
                "evaluation_iterations"
            ]
        except KeyError as e:
            raise RuntimeError(
                f"Configuration error: Missing required generation setting: {e}"
            )
        logger.info(
            "Phase: INITIALIZATION -> Environment OK, model and metrics handler initialized. "
            + "Moving to GENERATION phase."
        )

    def _generation_phase(self, **kwargs) -> None:
        """
        Execute the generation phase by performing multiple iterations and attempts.

        This phase:
        1. Prepares the prompt using the subclass-specific implementation.
        2. Runs the configured number of iterations.
        3. For each iteration, makes multiple generation attempts until success or max attempts.
        4. Records metrics for each attempt.

        The process stops early if a successful generation is achieved.

        Args:
            **kwargs: Additional keyword arguments to be passed to the prompt preparation method.

        Raises:
            Exception: Any exceptions that might occur during the generation process.
        """
        logger.info("Phase: GENERATION -> Starting generation process.")
        logger.info(f"Max attempts: {self.max_attempts!r}")
        logger.info(f"Evaluation iterations: {self.evaluation_iterations!r}")
        prompt: str = self.prepare_prompt(**kwargs)
        for iteration in range(1, self.evaluation_iterations + 1):
            logger.info(f" Iteration {iteration} started.")
            output_path: Path = Path(self.config["output"]["output_file"])
            for attempt in range(1, self.max_attempts + 1):
                logger.info(
                    f"--- Starting Attempt {attempt} (Iteration {iteration}) ---"
                )
                attempt_instance = self.GenerationAttemptClass(self.model, self.config)
                success, attempt_metrics = attempt_instance.run(
                    output_path, prompt, save_code
                )
                self.metrics_handler.record_attempt(
                    iteration_number=iteration,
                    attempt_number=attempt,
                    attempt_metrics=attempt_metrics,
                )
                logger.info(
                    f"Attempt {attempt} finished with metrics:\n{attempt_metrics}"
                )
                if success:
                    logger.info(f"Iteration {iteration}: Successful attempt achieved.")
                    break
                else:
                    logger.info(f"Iteration {iteration}: Attempt {attempt} failed.")

            logger.info(f"Iteration {iteration} finished.")

        logger.info(
            "Phase: GENERATION -> Generation process finished. Moving to REPORT phase."
        )

    def _report_phase(self) -> None:
        """
        Analyze the metrics collected during the generation process and display a summary report.

        This phase:
        1. Processes all collected metrics from generation attempts.
        2. Generates a comprehensive report on the generation performance.
        3. Logs the report at INFO level.

        This method is typically called in the finally block to ensure reports are generated
        even if exceptions occur during generation.
        """
        logger.info("Phase: REPORT -> Start analyzing metrics.")
        logger.debug(f"Collected attempt metrics:\n{self.metrics_handler.metrics_df}")
        logger.info(f"Generation report:\n{self.metrics_handler.get_report()}")
        logger.info("Phase: REPORT -> Metrics analysis finished.")

    def run(self, **kwargs) -> None:
        """
        Run the complete generation process (initialization, generation, reporting).

        This is the main entry point for executing the generation workflow:
        1. Runs the initialization phase.
        2. Executes the generation phase.
        3. Ensures the reporting phase is executed even if errors occur.
        4. Logs the total execution time.

        Args:
            **kwargs: Additional keyword arguments to prepare prompt (e.g., service_name, node_name) and other parameters for the generation process. These are passed to the generation phase.

        Raises:
            None: Logs but does not propagate exceptions from the generation process.
        """
        try:
            self._initialization_phase()
            self._generation_phase(**kwargs)
        except Exception as e:
            logger.error(f"Error during generation process:\n\n{e}\n")
        finally:
            if hasattr(self, "metrics_handler"):
                self._report_phase()
            else:
                logger.warning("No metrics handler found. Skipping REPORT phase.")

            logger.info(
                f"Generation process finished in {time.time() - self.start_time:.0f} seconds."
            )
