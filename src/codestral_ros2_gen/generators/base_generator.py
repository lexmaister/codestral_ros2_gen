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
        """Generate ROS2 code following the block diagram flow."""
        metrics = self._init_metrics(max_attempts, timeout)

        for attempt in range(1, metrics["config_used"]["max_attempts"] + 1):
            metrics["attempts"] = attempt
            attempt_start = time.time()

            try:
                success, usage = self._execute_attempt(
                    output_path=output_path,
                    timeout=metrics["config_used"]["per_attempt_timeout"],
                    **kwargs,
                )

                if success:
                    self._record_success(metrics, attempt_start, usage)
                    return True, metrics

            except RuntimeError as e:
                error_msg = str(e)
                if "timeout" in error_msg.lower():
                    metrics["timeouts"]["attempts"] += 1
                metrics["errors"].append(error_msg)
                metrics["error_patterns"].append(error_msg)

            metrics["attempt_timers"].append(time.time() - attempt_start)

        self._record_final_metrics(metrics)
        return False, metrics

    def _init_metrics(
        self, max_attempts: Optional[int], timeout: Optional[float]
    ) -> Dict[str, Any]:
        """Initialize metrics dictionary with timing start."""
        max_attempts = max_attempts or self.config["generation"]["max_attempts"]
        timeout = timeout or self.config["generation"]["timeout"]
        per_attempt_timeout = max(timeout / max_attempts, 0.3)  # Minimum 300ms

        return {
            "attempts": 0,
            "errors": [],
            "attempt_timers": [],
            "timeouts": {"attempts": 0},
            "token_usage": {
                "prompt_tokens": 0,
                "completion_tokens": 0,
                "total_tokens": 0,
            },
            "main_timer": time.time(),  # Store start time directly
            "error_patterns": [],  # Add this for error tracking
            "config_used": {
                "timeout": timeout,
                "max_attempts": max_attempts,
                "per_attempt_timeout": per_attempt_timeout,
            },
        }

    def _execute_attempt(
        self, output_path: Path, timeout: float, **kwargs
    ) -> Tuple[bool, Optional[ModelUsage]]:
        """Execute single attempt with simple timeout checking between stages."""
        start = time.time()
        stage_time = timeout / 3

        try:
            # GenCode stage
            prompt = self.prepare_prompt(**kwargs)
            if time.time() - start > stage_time:
                raise RuntimeError("Timeout: Model generation timed out")
            generated_code, usage = self.model.complete(prompt)

            # Parse code
            if time.time() - start > stage_time * 2:
                raise RuntimeError("Timeout: Code parsing timed out")
            code = ROS2CodeParser.parse(generated_code)
            if not code:
                raise RuntimeError("Failed to parse generated code")

            # Save code
            if not self.save_output(code, output_path):
                raise RuntimeError("Failed to save code")

            # RunTests stage
            if time.time() - start > timeout:
                raise RuntimeError("Timeout: Test execution timed out")
            tests_passed, error = self.run_tests(output_path.parent)
            if not tests_passed:
                raise RuntimeError(f"Tests failed: {error}")

            return True, usage

        except Exception as e:
            # Ensure we propagate timeout errors with consistent messages
            error_msg = str(e)
            if "timeout" in error_msg.lower():
                logger.error(f"Timeout error: {error_msg}")
                raise RuntimeError(f"Timeout: {error_msg}")
            logger.error(f"Error in attempt: {error_msg}")
            raise RuntimeError(error_msg)

    def _record_success(
        self, metrics: Dict[str, Any], attempt_start: float, usage: Optional[ModelUsage]
    ) -> None:
        """Record successful attempt metrics."""
        metrics["attempt_timers"].append(time.time() - attempt_start)
        if usage:
            metrics["token_usage"].update(usage.__dict__)
        metrics["main_timer"] = time.time() - metrics["main_timer"]
        self.metrics_handler.add_metric(metrics)

    def _record_timeout(self, metrics: Dict[str, Any]) -> None:
        """Record timeout metrics."""
        metrics["timeouts"]["attempts"] += 1
        metrics["errors"].append("Attempt timed out")

    def _record_final_metrics(self, metrics: Dict[str, Any]) -> None:
        """Record final metrics for failed attempts."""
        metrics["main_timer"] = time.time() - metrics["main_timer"]
        self.metrics_handler.add_metric(metrics)

    def _safe_model_complete(self, prompt: str) -> Tuple[str, ModelUsage]:
        """Safely execute model completion with error handling."""
        try:
            return self.model.complete(prompt)
        except Exception as e:
            raise RuntimeError(f"Model generation failed: {str(e)}")

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
