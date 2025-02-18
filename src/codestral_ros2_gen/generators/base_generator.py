# src/codestral_ros2_gen/generators/base_generator.py

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional, Dict, Any, Tuple
import time
from functools import partial
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeoutError
import pytest
import subprocess

from codestral_ros2_gen import *
from codestral_ros2_gen.metrics.metrics_handler import MetricsHandler
from codestral_ros2_gen.models.mistral_client import MistralClient, ModelUsage
from codestral_ros2_gen.utils.code_parser import ROS2CodeParser


class BaseGenerator(ABC):
    """Abstract base class for ROS2 code generators that follows the flowchart:
    MainTimer -> Counter -> Timer -> GenCode -> SaveCode -> RunTests -> TestResult
    """

    def __init__(
        self,
        config_path: Optional[Path] = None,
        metrics_file: Optional[str] = None,
        api_key: Optional[str] = None,
    ):
        """
        Initialize the generator.

        Args:
            config_path: Path to configuration file. If None, uses default config.
            metrics_file: Path to metrics output file. If None, uses config value.
            api_key: Mistral API key. If None, attempts to get from env or config.
        """
        # Load configuration
        self.config_path = config_path if config_path else get_config_path()
        self.config = load_config(self.config_path)

        # Initialize metrics handler with loaded config
        self.metrics_handler = MetricsHandler(
            metrics_file=metrics_file, config=self.config
        )

        # Initialize AI model client
        self.model = MistralClient(api_key=api_key, config=self.config)

    @abstractmethod
    def prepare_prompt(self, **kwargs) -> str:
        """
        Prepare the generation prompt based on provided parameters.

        Args:
            **kwargs: Generator-specific parameters

        Returns:
            str: Formatted prompt string
        """
        pass

    @abstractmethod
    def save_output(self, code: str, output_path: Path) -> bool:
        """
        Save the generated code to file.

        Args:
            code: Generated code to save
            output_path: Path where to save the code

        Returns:
            bool: True if save successful, False otherwise
        """
        pass

    def run_tests(self, package_path: Path) -> Tuple[bool, str]:
        """Execute tests for generated code."""
        try:
            # Run colcon build
            build_cmd = f"cd {package_path.parent} && colcon build --packages-select {package_path.name}"
            subprocess.run(build_cmd, shell=True, check=True)

            # Run tests
            result = pytest.main([str(package_path)])
            return result == pytest.ExitCode.OK, ""
        except Exception as e:
            return False, str(e)

    def generate(
        self,
        output_path: Path,
        max_attempts: Optional[int] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[bool, Dict[str, Any]]:
        """Generate ROS2 code element following the flowchart structure."""
        # MainTimer start
        main_start_time = time.time()

        # Initialize counter
        if max_attempts is None:
            max_attempts = self.config["generation"]["max_attempts"]
        if timeout is None:
            timeout = self.config["generation"].get("timeout", 300)

        metrics = {
            "attempts": 0,
            "errors": [],
            "main_timer": 0,
            "token_usage": {
                "prompt_tokens": 0,
                "completion_tokens": 0,
                "total_tokens": 0,
            },
        }

        # Counter loop
        for attempt in range(1, max_attempts + 1):
            metrics["attempts"] = attempt
            try:
                # GenCode phase
                prompt = self.prepare_prompt(**kwargs)
                generated_code, usage = self.model.complete(prompt, timeout=timeout)

                # Update token usage metrics
                if isinstance(usage, ModelUsage):
                    metrics["token_usage"].update(
                        {
                            "prompt_tokens": usage.prompt_tokens,
                            "completion_tokens": usage.completion_tokens,
                            "total_tokens": usage.total_tokens,
                        }
                    )

                # Parse code
                code = ROS2CodeParser.parse(generated_code)
                if not code:
                    metrics["errors"].append("parsing_error")
                    continue

                # SaveCode phase
                if not self.save_output(code, output_path):
                    metrics["errors"].append("save_error")
                    continue

                # RunTests phase
                tests_passed, error = self.run_tests(output_path.parent)
                if not tests_passed:
                    metrics["errors"].append(f"test_failure: {error}")
                    continue

                # Success
                metrics["main_timer"] = time.time() - main_start_time
                self.metrics_handler.add_metric(metrics)
                return True, metrics

            except Exception as e:
                metrics["errors"].append(f"error: {str(e)}")

        # All attempts failed
        metrics["main_timer"] = time.time() - main_start_time
        self.metrics_handler.add_metric(metrics)
        return False, metrics

    def _record_error(
        self, metrics: Dict[str, Any], error_type: str, error_msg: str
    ) -> None:
        """Record error in metrics with timestamp."""
        logger.warning(error_msg)
        metrics["error_patterns"].append(error_type)
        metrics["error_details"].append(error_msg)
        metrics["timestamps"].append(time.time())

    def get_metrics(self) -> Dict[str, Any]:
        """Get collected metrics statistics."""
        return self.metrics_handler.generate_report()

    def plot_metrics(self, output_dir: Optional[Path] = None) -> Optional[Path]:
        """
        Create metrics visualization plots.

        Args:
            output_dir: Directory to save plots. If None, uses current directory.

        Returns:
            Optional[Path]: Path to saved plot file, or None if plotting failed
        """
        return self.metrics_handler.plot_metrics(output_dir=output_dir)
