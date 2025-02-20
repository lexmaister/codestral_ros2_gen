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
    """Abstract base class for ROS2 code generators.

    This class implements the generation flow:
    MainTimer -> Counter -> Timer -> GenCode -> SaveCode -> RunTests -> TestResult

    The generation process follows these steps:
    1. Initialize metrics and timers
    2. For each attempt:
        a. Generate code using AI model
        b. Save code to filesystem
        c. Optionally build workspace and run tests
    3. Record metrics and results

    Attributes:
        config_path (Path): Path to configuration file
        config (Dict): Loaded configuration
        metrics_handler (MetricsHandler): Metrics collection and analysis
        model (MistralClient): AI model client
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

    def generate(
        self,
        output_path: Path,
        max_attempts: Optional[int] = None,
        timeout: Optional[float] = None,
        build_workspace: bool = False,
        **kwargs,
    ) -> Tuple[bool, Dict[str, Any]]:
        """Generate ROS2 code with metrics collection and optional workspace building.

        Args:
            output_path: Where to save the generated code
            max_attempts: Maximum generation attempts (default: from config)
            timeout: Total timeout in seconds (default: from config)
            build_workspace: Whether to build ROS2 workspace and run tests
            **kwargs: Additional arguments passed to prepare_prompt()

        Returns:
            Tuple[bool, Dict[str, Any]]: (success, metrics_dict)

        Metrics collected:
            - main_timer: Total execution time
            - attempts: Number of attempts made
            - attempt_timers: List of per-attempt execution times
            - timeouts: Count of timeout occurrences
            - errors: List of error messages
            - token_usage: Model token consumption statistics
        """
        metrics = self._init_metrics(max_attempts, timeout)
        metrics["build_workspace"] = build_workspace
        self._current_metrics = metrics  # Store for stage tracking

        for attempt in range(1, metrics["config_used"]["max_attempts"] + 1):
            metrics["attempts"] = attempt
            attempt_start = time.time()

            try:
                success, usage = self._execute_attempt(
                    output_path=output_path,
                    timeout=metrics["config_used"]["per_attempt_timeout"],
                    build_workspace=build_workspace,
                    **kwargs,
                )

                if success:
                    self._record_success(metrics, attempt_start, usage)
                    return True, metrics

            except RuntimeError as e:
                self._record_error(metrics, str(e))

            metrics["attempt_timers"].append(time.time() - attempt_start)

        self._record_final_metrics(metrics)
        delattr(self, "_current_metrics")  # Cleanup
        return False, metrics

    def run_tests(self, package_path: Path) -> Tuple[bool, str]:
        """Execute ROS2 package tests."""
        try:
            # Check build_workspace attribute is set
            build_workspace = getattr(self, "build_workspace", False)

            # Only build if workspace building is enabled
            if build_workspace:
                logger.info(f"Building workspace for package: {package_path.name}")
                build_cmd = f"cd {package_path.parent} && colcon build --packages-select {package_path.name}"
                subprocess.run(build_cmd, shell=True, check=True)

            # Run tests
            result = pytest.main([str(package_path)])
            return result == pytest.ExitCode.OK, ""
        except Exception as e:
            return False, str(e)

    def _init_metrics(
        self, max_attempts: Optional[int], timeout: Optional[float]
    ) -> Dict[str, Any]:
        """Initialize metrics dictionary with timing start."""
        max_attempts = max_attempts or self.config["generation"]["max_attempts"]
        timeout = timeout or self.config["generation"]["timeout"]
        # Use timeout/max_attempts directly instead of enforcing a minimum of 0.3
        per_attempt_timeout = timeout / max_attempts

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
        self, output_path: Path, timeout: float, build_workspace: bool = False, **kwargs
    ) -> Tuple[bool, Optional[ModelUsage]]:
        """Execute a single generation attempt with a full timeout for model generation.

        If model.complete takes longer than 'timeout' seconds, add an error record to metrics
        and return (False, None). Parsing, saving, and test execution are considered instant.
        """
        self.build_workspace = build_workspace
        stage_results = []
        try:
            # GenCode stage with full timeout
            prompt = self.prepare_prompt(**kwargs)
            t0 = time.time()
            generated_code, usage = self.model.complete(prompt)
            t1 = time.time()
            if (t1 - t0) > timeout:
                error = "Model generation timed out"  # Updated error string exactly as expected by tests
                self._current_metrics["stage_results"] = stage_results
                self._current_metrics["errors"].append(error)
                self._current_metrics["error_patterns"].append(error)
                self._current_metrics["timeouts"]["attempts"] = (
                    self._current_metrics["timeouts"].get("attempts", 0) + 1
                )
                return False, None
            stage_results.append("GenCode")

            # Parse and save code (assumed instant operations)
            code = ROS2CodeParser.parse(generated_code)
            if not code:
                error = "Failed to parse generated code"
                self._current_metrics["stage_results"] = stage_results
                self._current_metrics["errors"].append(error)
                self._current_metrics["error_patterns"].append(error)
                return False, None
            if not self.save_output(code, output_path):
                error = "Failed to save code"
                self._current_metrics["stage_results"] = stage_results
                self._current_metrics["errors"].append(error)
                self._current_metrics["error_patterns"].append(error)
                return False, None
            stage_results.append("SaveCode")

            # RunTests stage (no timeout enforcement here)
            if build_workspace:
                tests_passed, error = self.run_tests(output_path.parent)
                if not tests_passed:
                    error = f"Tests failed: {error}"
                    self._current_metrics["stage_results"] = stage_results
                    self._current_metrics["errors"].append(error)
                    self._current_metrics["error_patterns"].append(error)
                    return False, None
                stage_results.append("RunTests")

            self._current_metrics["stage_results"] = stage_results
            return True, usage

        except Exception as e:
            err_str = str(e)
            self._current_metrics["stage_results"] = stage_results
            self._current_metrics["errors"].append(err_str)
            self._current_metrics["error_patterns"].append(err_str)
            if "timeout" in err_str.lower():
                self._current_metrics["timeouts"]["attempts"] = (
                    self._current_metrics["timeouts"].get("attempts", 0) + 1
                )
            return False, None

    def _record_success(
        self, metrics: Dict[str, Any], attempt_start: float, usage: Optional[ModelUsage]
    ) -> None:
        """Record successful attempt metrics."""
        metrics["attempt_timers"].append(time.time() - attempt_start)
        if usage:
            metrics["token_usage"].update(usage.__dict__)
        metrics["main_timer"] = time.time() - metrics["main_timer"]
        self.metrics_handler.add_metric(metrics)

    def _record_error(self, metrics: Dict[str, Any], error_msg: str) -> None:
        """Record error in metrics.

        Args:
            metrics: Metrics dictionary to update
            error_msg: Error message to record
        """
        logger.warning(error_msg)
        metrics["errors"].append(error_msg)
        metrics["error_patterns"].append(error_msg)
        if "timeout" in error_msg.lower():
            metrics["timeouts"]["attempts"] += 1

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
