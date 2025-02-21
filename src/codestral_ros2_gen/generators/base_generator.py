import time
from pathlib import Path
from typing import Optional, Dict, Any, Tuple
from abc import ABC, abstractmethod

from codestral_ros2_gen import get_config_path, load_config, logger
from codestral_ros2_gen.metrics.metrics_handler import MetricsHandler
from codestral_ros2_gen.models.mistral_client import MistralClient, ModelUsage
from codestral_ros2_gen.generators.generation_attempt import GenerationAttempt


class BaseGenerator(ABC):
    def __init__(
        self,
        config_path: Optional[Path] = None,
        metrics_file: Optional[str] = None,
        api_key: Optional[str] = None,
    ):
        self.config_path = config_path if config_path else get_config_path()
        self.config = load_config(self.config_path)
        self.metrics_handler = MetricsHandler(
            metrics_file=metrics_file, config=self.config
        )
        self.model = MistralClient(api_key=api_key, config=self.config)
        # Load high-level settings from config (e.g., number of iterations)
        self.max_attempts = self.config["generation"].get("max_attempts", 3)
        self.evaluation_iterations = self.config["generation"].get(
            "evaluation_iterations", 30
        )
        self.main_start_time = time.time()
        self.attempt_counter = 0

    @abstractmethod
    def prepare_prompt(self, **kwargs) -> str:
        # ...to be implemented by subclasses...
        pass

    @abstractmethod
    def save_output(self, code: str, output_path: Path) -> bool:
        # ...to be implemented by subclasses...
        pass

    def _validate_environment(self, output_path: Path) -> bool:
        """
        Validate the environment for generation attempts.

        Checks that the configuration contains required keys and that the output_path
        is writable (creating parent directories if necessary).

        Returns:
            bool: True if the environment is valid; otherwise, False.
        """
        # Check required generation config keys.
        if (
            "generation" not in self.config
            or "timeout" not in self.config["generation"]
        ):
            logger.error("Missing required 'generation' configuration or timeout.")
            return False
        # Ensure we can write to the output directory.
        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            logger.error(f"Cannot create or access output directory: {str(e)}")
            return False
        return True

    def run_generator(self, output_path: Path, **kwargs) -> Tuple[bool, Dict[str, Any]]:
        # Validate environment before starting generation attempts.
        if not self._validate_environment(output_path):
            logger.error("Environment validation failed. Aborting generation process.")
            return False, {"error": "Environment validation failed."}

        overall_success = False
        iteration = 0
        aggregated_metrics = {"attempts": []}

        logger.info("Starting high-level generation process")
        while iteration < self.evaluation_iterations and not overall_success:
            self.attempt_counter += 1
            logger.info(f"--- Starting Attempt {self.attempt_counter} ---")
            attempt_instance = GenerationAttempt(self.model, self.config)
            prompt = self.prepare_prompt(**kwargs)
            success, attempt_metrics = attempt_instance.run_attempt(
                output_path, prompt, self.save_output
            )
            aggregated_metrics["attempts"].append(attempt_metrics)
            logger.info(
                f"Attempt {self.attempt_counter} finished with state: {attempt_metrics['final_state']}"
            )
            if success:
                overall_success = True
                logger.info("High-level process: Attempt succeeded")
            else:
                logger.info(
                    "High-level process: Attempt failed, retrying if iterations remain"
                )
            iteration += 1

        aggregated_metrics["total_time"] = time.time() - self.main_start_time
        aggregated_metrics["final_result"] = "SUCCESS" if overall_success else "FAILURE"
        logger.info("High-level generation process finished")
        self.metrics_handler.add_metric(aggregated_metrics)
        return overall_success, aggregated_metrics
