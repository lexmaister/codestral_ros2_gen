import time
from pathlib import Path
from typing import Optional, Dict, Any, Tuple
from abc import ABC, abstractmethod

from codestral_ros2_gen import get_config_path, load_config, logger
from codestral_ros2_gen.utils.metrics_handler import MetricsHandler
from codestral_ros2_gen.models.mistral_client import MistralClient
from codestral_ros2_gen.generators.generation_attempt import GenerationAttempt
from codestral_ros2_gen.utils.file_io import save_code


def _missing_keys(
    config: Dict[str, Any], section: str, required: Tuple[str, ...]
) -> None:
    """Raise error if any required keys are missing in the given section of config."""
    if section not in config:
        raise RuntimeError(
            f"Configuration error: Missing section '{section}' in configuration."
        )
    missing = [key for key in required if key not in config[section]]
    if missing:
        raise RuntimeError(
            f"Configuration error: Missing required keys in '{section}': {', '.join(missing)}"
        )


class BaseGenerator(ABC):
    def __init__(self, config_path: Optional[Path] = None) -> None:
        self.config_path: Path = config_path if config_path else get_config_path()
        self.config: dict = None  # Will be loaded in _initialization_phase
        self.start_time: float = time.time()

    @abstractmethod
    def prepare_prompt(self, **kwargs) -> str:
        """
        Prepare the input prompt to be used for code generation.

        Returns:
            str: The prepared prompt.
        """
        pass

    def _validate_environment(self) -> None:
        """
        Validate that the configuration has a 'generation' section with required keys,
        and ensure that the output directory is accessible.
        """
        # Check that "generation" section contains required keys.
        _missing_keys(
            self.config,
            "generation",
            ("max_attempts", "evaluation_iterations"),
        )
        # Check that "output" section contains the output file key.
        _missing_keys(
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
        """
        cwd: Path = Path.cwd()
        if not ((cwd / "src").exists() or (cwd / "install").exists()):
            raise RuntimeError(
                "Workspace error: Current directory is not a valid ROS2 workspace (missing 'src' or 'install')."
            )

    def _initialization_phase(self) -> None:
        """
        Load the configuration, validate required keys, check the workspace,
        and initialize internal components.
        """
        self.config = load_config(self.config_path)
        self._validate_environment()
        self._check_ros2_workspace()
        try:
            # Use the metrics_file value from the "metrics" section,
            # cleanup it first if it exists.
            metrics_file = self.config["metrics"]["metrics_file"]
            if Path(metrics_file).exists():
                Path(metrics_file).unlink()

            self.metrics_handler = MetricsHandler(
                config=self.config, metrics_file=metrics_file
            )
            self.model = MistralClient(config=self.config)
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

    def _generation_phase(self) -> None:
        """
        Execute the generation phase by performing multiple iterations and attempts. Update metrics.
        """
        logger.info("Phase: GENERATION -> Starting generation process.")
        logger.info(f"Max attempts: {self.max_attempts!r}")
        logger.info(f"Evaluation iterations: {self.evaluation_iterations!r}")
        prompt: str = self.prepare_prompt()
        for iteration in range(1, self.evaluation_iterations + 1):
            logger.info(f" Iteration {iteration} started.")
            output_path: Path = Path(self.config["output"]["output_file"])
            for attempt in range(1, self.max_attempts + 1):
                logger.info(
                    f"--- Starting Attempt {attempt} (Iteration {iteration}) ---"
                )
                attempt_instance = GenerationAttempt(self.model, self.config)
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

            logger.info(f" Iteration {iteration} finished.")

        logger.info(
            "Phase: GENERATION -> Generation process finished. Moving to REPORT phase."
        )

    def _report_phase(self) -> None:
        """
        Analyse the metrics collected during the generation process and display a summary report.
        """
        logger.info("Phase: REPORT -> Start analyzing metrics.")
        logger.info(f"Collected attempt metrics:\n{self.metrics_handler.metrics_df}")

        logger.info("Phase: REPORT -> Metrics analysis finished.")

    def run(self, **kwargs) -> None:
        """
        Run the complete generation process (initialization, generation, reporting)
        """
        try:
            self._initialization_phase()
            self._generation_phase()
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
