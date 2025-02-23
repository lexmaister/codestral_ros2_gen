from typing import Tuple, Dict, Any
import pytest
from pathlib import Path
from unittest.mock import patch

from codestral_ros2_gen import get_project_root, get_config_path, load_config
from codestral_ros2_gen.generators.base_generator import BaseGenerator
from codestral_ros2_gen.metrics.metrics_handler import MetricsHandler


# Dummy MistralClient substitute for testing.
class DummyMistral:
    def complete(self, prompt: str) -> Tuple[str, Any]:
        DummyUsage = type(
            "DummyUsage",
            (),
            {"prompt_tokens": 10, "completion_tokens": 20, "total_tokens": 30},
        )
        return "dummy code", DummyUsage()


# Dummy subclass simulating a successful generation process.
class DummyGenerator(BaseGenerator):
    def prepare_prompt(self, **kwargs) -> str:
        return "dummy prompt"

    def _initialization_phase(self) -> None:
        # Here, self.config is already set externally.
        self._validate_environment()
        self._check_ros2_workspace()
        # Use the metrics_file key from the metrics section.
        self.metrics_handler = MetricsHandler(
            config=self.config, metrics_file=self.config["metrics"]["metrics_file"]
        )
        self.model = DummyMistral()
        self.max_attempts = self.config["generation"]["max_attempts"]
        self.evaluation_iterations = self.config["generation"]["evaluation_iterations"]

    def _generation_phase(self, **kwargs) -> Tuple[bool, Dict[str, Any]]:
        dummy_iteration = {
            "main_iteration": 1,
            "attempts": [
                {
                    "attempt_number": 1,
                    "metrics": {"dummy_metric": 1},
                    "success": True,
                }
            ],
            "iteration_result": "SUCCESS",
        }
        return True, {"iterations": [dummy_iteration]}


# Dummy subclass to force a configuration error (missing 'generation').
class DummyGeneratorError(BaseGenerator):
    def prepare_prompt(self, **kwargs) -> str:
        return "error prompt"

    def _initialization_phase(self) -> None:
        self.config = {"metrics": {"metrics_file": "dummy_metrics.jsonl"}}
        self._validate_environment()


# Fixture to simulate a valid ROS2 workspace.
@pytest.fixture(autouse=True)
def fake_workspace(tmp_path):
    fake_ws = tmp_path / "workspace"
    (fake_ws / "src").mkdir(parents=True, exist_ok=True)
    with patch("pathlib.Path.cwd", return_value=fake_ws):
        yield


# Fixture for a successful dummy generator.
@pytest.fixture
def dummy_gen_success(tmp_path) -> DummyGenerator:
    # Load configuration from project config.
    config = load_config(get_config_path())
    # Ensure the "output" section exists.
    if "output" not in config:
        config["output"] = {}
    # Override the metrics file to a temporary file in tmp_path.
    new_metrics_file = tmp_path / "dummy_metrics.jsonl"
    config["metrics"]["metrics_file"] = str(new_metrics_file)
    # Override the output file path in the "output" section.
    new_node_file = tmp_path / "dummy_node.py"
    config["output"]["output_file"] = str(new_node_file)
    # Create an instance of DummyGenerator and manually set its config.
    gen = DummyGenerator(config_path=get_config_path())
    gen.config = config
    # Perform initialization.
    gen._initialization_phase()
    # Ensure the MetricsHandler uses the new temporary metrics file.
    gen.metrics_handler.metrics_file = new_metrics_file
    return gen


@pytest.fixture
def dummy_gen_error() -> DummyGeneratorError:
    return DummyGeneratorError()


def test_run_success(dummy_gen_success: DummyGenerator):
    overall_success, final_metrics = dummy_gen_success.run()
    assert overall_success is True
    assert "final_result" in final_metrics
    assert final_metrics["final_result"] == "SUCCESS"
    assert final_metrics["total_time"] > 0


def test_run_error(dummy_gen_error: DummyGeneratorError):
    overall_success, final_metrics = dummy_gen_error.run()
    assert overall_success is False
    assert "error" in final_metrics
    assert (
        "MetricsHandler not initialized: Check that required configuration sections"
        in final_metrics["error"]
    )


def test_run_timing(dummy_gen_success: DummyGenerator):
    overall_success, final_metrics = dummy_gen_success.run()
    assert final_metrics["total_time"] >= 0.0


def test_metrics_collection(dummy_gen_success: DummyGenerator):
    _, gen_metrics = dummy_gen_success._generation_phase()
    assert "iterations" in gen_metrics
    assert isinstance(gen_metrics["iterations"], list)
    assert len(gen_metrics["iterations"]) == 1
    iteration = gen_metrics["iterations"][0]
    assert iteration["iteration_result"] == "SUCCESS"
    assert len(iteration["attempts"]) == 1


# ---- New tests for individual helper methods ----


def test_validate_environment_success(dummy_gen_success: DummyGenerator):
    """
    Test that _validate_environment() passes when configuration is complete.
    """
    try:
        dummy_gen_success._validate_environment()
    except Exception as e:
        pytest.fail(f"_validate_environment() raised Exception unexpectedly: {e}")


def test_validate_environment_failure(tmp_path):
    """
    Test that _validate_environment() fails if required keys in 'generation' are missing.
    """

    class IncompleteGenerator(BaseGenerator):
        def prepare_prompt(self, **kwargs) -> str:
            return "dummy prompt"

        def _initialization_phase(self) -> None:
            pass

    gen = IncompleteGenerator()
    # Missing keys inside "generation".
    gen.config = {"generation": {}, "output": {"output_file": "dummy_node.py"}}
    with pytest.raises(RuntimeError, match="Missing required keys in 'generation'"):
        gen._validate_environment()


def test_check_ros2_workspace_success(dummy_gen_success: DummyGenerator):
    """
    Test that _check_ros2_workspace() passes in a valid workspace.
    """
    try:
        dummy_gen_success._check_ros2_workspace()
    except Exception as e:
        pytest.fail(f"_check_ros2_workspace() raised Exception unexpectedly: {e}")


def test_check_ros2_workspace_failure(tmp_path):
    """
    Test that _check_ros2_workspace() fails when neither 'src' nor 'install' exist.
    """
    with patch("pathlib.Path.cwd", return_value=tmp_path / "empty_workspace"):
        gen = DummyGenerator(config_path=get_config_path())
        gen.config = load_config(get_config_path())
        with pytest.raises(
            RuntimeError, match="Current directory is not a valid ROS2 workspace"
        ):
            gen._check_ros2_workspace()
