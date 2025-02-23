import time
from pathlib import Path
from typing import Optional, Dict, Any, Tuple
from abc import ABC, abstractmethod

from codestral_ros2_gen import get_config_path, load_config, logger
from codestral_ros2_gen.metrics.metrics_handler import MetricsHandler
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
        self.main_start_time: float = time.time()
        self.attempt_counter: int = 0

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
            # Use the metrics_file value from the "metrics" section.
            self.metrics_handler = MetricsHandler(
                config=self.config, metrics_file=self.config["metrics"]["metrics_file"]
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
            "Stage: INITIALIZATION -> Environment OK. Starting generation process."
        )

    def _generation_phase(self, **kwargs) -> Tuple[bool, Dict[str, Any]]:
        """
        Execute the generation phase by performing multiple iterations and attempts.

        Returns:
            Tuple[bool, Dict[str, Any]]: Overall success flag and aggregated iteration metrics.
        """
        overall_success: bool = False
        aggregated_main_metrics: Dict[str, list] = {"iterations": []}
        for main_iter in range(1, self.evaluation_iterations + 1):
            logger.info(f"Main Iteration {main_iter} started.")
            iteration_success: bool = False
            iteration_attempts: list = []
            # Use the output file from the "output" section.
            output_path: Path = Path(self.config["output"]["output_file"])
            for attempt in range(1, self.max_attempts + 1):
                self.attempt_counter += 1
                logger.info(
                    f"--- Starting Attempt {self.attempt_counter} (Main Iteration {main_iter}, Attempt {attempt}) ---"
                )
                attempt_instance = GenerationAttempt(self.model, self.config)
                prompt: str = self.prepare_prompt(**kwargs)
                success, attempt_metrics = attempt_instance.run(
                    output_path, prompt, save_code
                )
                self.metrics_handler.record_attempt(
                    self.attempt_counter, attempt_metrics
                )
                logger.info(
                    f"Attempt {self.attempt_counter} finished with metrics: {attempt_metrics}"
                )
                iteration_attempts.append(
                    {
                        "attempt_number": self.attempt_counter,
                        "metrics": attempt_metrics,
                        "success": success,
                    }
                )
                if success:
                    iteration_success = True
                    logger.info(
                        f"Main Iteration {main_iter}: Successful attempt achieved."
                    )
                    break
                else:
                    logger.info(
                        f"Main Iteration {main_iter}: Attempt {attempt} failed."
                    )
            aggregated_main_metrics["iterations"].append(
                {
                    "main_iteration": main_iter,
                    "attempts": iteration_attempts,
                    "iteration_result": "SUCCESS" if iteration_success else "FAILURE",
                }
            )
            if iteration_success:
                overall_success = True
                break
        return overall_success, aggregated_main_metrics

    def _report_phase(self, aggregated_main_metrics: Dict[str, Any]) -> Dict[str, Any]:
        """
        Compute and record overall metrics, then generate a summary report.

        Returns:
            Dict[str, Any]: Aggregated metrics including total run time and final result.
        """
        overall_time: float = time.time() - self.main_start_time
        aggregated_metrics: Dict[str, Any] = {
            "total_time": overall_time,
            "final_result": (
                "SUCCESS"
                if any(
                    iteration["iteration_result"] == "SUCCESS"
                    for iteration in aggregated_main_metrics["iterations"]
                )
                else "FAILURE"
            ),
            "details": aggregated_main_metrics,
        }
        logger.info("Stage: REPORT -> Generation process finished.")
        self.metrics_handler.record_overall(aggregated_metrics)
        report: Dict[str, Any] = self.metrics_handler.generate_report().to_dict()
        logger.info("Summary Report:\n" + str(report))
        return aggregated_metrics

    def run(self, **kwargs) -> Tuple[bool, Dict[str, Any]]:
        """
        Wrap the complete generation process (initialization, generation, reporting)
        in a try/except/finally block for unified error handling.

        Returns:
            Tuple[bool, Dict[str, Any]]: Overall success flag and aggregated metrics.
        """
        overall_success: bool = False
        gen_metrics: Dict[str, Any] = {}
        try:
            self._initialization_phase()
            overall_success, gen_metrics = self._generation_phase(**kwargs)
        except Exception as e:
            overall_success = False
            gen_metrics = {"error": str(e)}
            logger.error(f"Error during run phases: {e}")
        finally:
            if hasattr(self, "metrics_handler"):
                final_metrics: Dict[str, Any] = self._report_phase(gen_metrics)
            else:
                final_metrics = {
                    "total_time": time.time() - self.main_start_time,
                    "final_result": "FAILURE",
                    "error": "MetricsHandler not initialized: Check that required configuration sections (e.g. 'generation') are present.",
                }
            return overall_success, final_metrics
