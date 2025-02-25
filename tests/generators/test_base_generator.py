from typing import Tuple, Dict, Any
import pytest
from pathlib import Path
from unittest.mock import patch

from codestral_ros2_gen import get_project_root, get_config_path, load_config
from codestral_ros2_gen.generators.base_generator import BaseGenerator
from codestral_ros2_gen.utils.metrics_handler import MetricsHandler


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


# -------------------------------------------
# Additional Edge-Case Tests for Coverage
# -------------------------------------------


class DummyGeneratorMultiIteration(BaseGenerator):
    """
    This generator has 'evaluation_iterations' > 1 to ensure we test
    the multi-iteration logic. It fails all attempts in the first iteration,
    then succeeds immediately in the second iteration.
    """

    def prepare_prompt(self, **kwargs) -> str:
        return "multi-iteration prompt"

    def _initialization_phase(self) -> None:
        """
        Basic initialization that sets up a dummy model, ensures we fail
        first iteration fully, then succeed in the second iteration.
        """
        self._validate_environment()
        self._check_ros2_workspace()

        from codestral_ros2_gen.utils.metrics_handler import MetricsHandler

        self.metrics_handler = MetricsHandler(
            config=self.config, metrics_file=self.config["metrics"]["metrics_file"]
        )

        class FailFirstIterationMistral:
            def __init__(self):
                # This counter helps us determine which main iteration is active.
                # We'll increment externally from run() or _generation_phase().
                self.main_iteration_call = 0

            def complete(self, prompt: str):
                DummyUsage = type(
                    "DummyUsage",
                    (),
                    {"prompt_tokens": 10, "completion_tokens": 20, "total_tokens": 30},
                )
                # If main_iteration_call == 1, we fail every request.
                # If main_iteration_call == 2, we succeed on the first attempt.
                if self.main_iteration_call == 1:
                    return "failing code", DummyUsage()
                else:
                    return "successful code", DummyUsage()

        self.model = FailFirstIterationMistral()
        self.max_attempts = self.config["generation"]["max_attempts"]
        self.evaluation_iterations = self.config["generation"]["evaluation_iterations"]

    def _generation_phase(self, **kwargs) -> Tuple[bool, Dict[str, Any]]:
        aggregated_metrics: Dict[str, Any] = {"iterations": []}

        # We'll manually loop over the number of main iterations to reflect
        # how BaseGenerator normally does it.
        overall_success = False
        for main_iter in range(1, self.evaluation_iterations + 1):
            # Let the model know which main iteration we're on
            self.model.main_iteration_call = main_iter
            iteration_result = "FAILURE"
            attempts_data = []

            for attempt_i in range(1, self.max_attempts + 1):
                # If second iteration => succeed immediately
                success = main_iter == 2 and attempt_i == 1
                attempts_data.append(
                    {
                        "attempt_number": attempt_i,
                        "metrics": {"foo": "bar"},
                        "success": success,
                    }
                )
                if success:
                    iteration_result = "SUCCESS"
                    overall_success = True
                    break

            aggregated_metrics["iterations"].append(
                {
                    "main_iteration": main_iter,
                    "attempts": attempts_data,
                    "iteration_result": iteration_result,
                }
            )
            if iteration_result == "SUCCESS":
                break  # mimic the break in BaseGenerator for a successful iteration

        return overall_success, aggregated_metrics


@pytest.fixture
def dummy_gen_multi_iteration(tmp_path) -> DummyGeneratorMultiIteration:
    """
    Fixture that returns a DummyGeneratorMultiIteration with config set
    to do multiple main iterations, ensuring coverage of the multi-iteration loop.
    """
    config = load_config(get_config_path())
    if "output" not in config:
        config["output"] = {}

    # Adjust settings for multi-iteration testing
    config["generation"]["evaluation_iterations"] = 2
    config["generation"]["max_attempts"] = 2

    # Replace metrics file and output file with temp paths
    new_metrics_file = tmp_path / "dummy_metrics_multi_iteration.jsonl"
    config["metrics"]["metrics_file"] = str(new_metrics_file)
    new_node_file = tmp_path / "dummy_node_multi_iteration.py"
    config["output"]["output_file"] = str(new_node_file)

    gen = DummyGeneratorMultiIteration(config_path=get_config_path())
    gen.config = config
    gen._initialization_phase()
    gen.metrics_handler.metrics_file = new_metrics_file
    return gen


def test_multi_iteration_success(
    dummy_gen_multi_iteration: DummyGeneratorMultiIteration,
):
    """
    Test scenario where the first iteration fails completely,
    then the second iteration immediately succeeds.
    """
    overall_success, final_metrics = dummy_gen_multi_iteration.run()
    assert (
        overall_success is True
    ), "Should succeed on second iteration after first fails."
    details = final_metrics.get("details", {})
    iterations = details.get("iterations", [])
    assert len(iterations) <= 2, "We have at most 2 iterations."
    assert iterations[0]["iteration_result"] == "FAILURE"
    # If we have a second iteration, it should be marked SUCCESS
    if len(iterations) > 1:
        assert iterations[1]["iteration_result"] == "SUCCESS"


def test_report_phase_in_failure(dummy_gen_error: DummyGeneratorError):
    """
    Force a configuration failure so that run() triggers the error path,
    verifying that _report_phase is still called in the finally block.
    """
    overall_success, final_metrics = dummy_gen_error.run()
    # We expect an error was raised, so success is False
    assert not overall_success
    # check that final_metrics still has final_result and total_time
    assert "final_result" in final_metrics, "Expected a final_result key in metrics."
    assert "total_time" in final_metrics, "Expected total_time recorded."
    assert (
        final_metrics["final_result"] == "FAILURE"
    ), "Should be labeled FAILURE due to config error."


@patch("pathlib.Path.mkdir")
def test_output_dir_unwritable(mock_mkdir, dummy_gen_success: DummyGenerator):
    """
    Mock an unwritable output directory and confirm it raises the expected error,
    covering the code that handles directory creation failures.
    """
    mock_mkdir.side_effect = OSError("Cannot create directory.")
    # Force re-validation to see if error is thrown
    with pytest.raises(RuntimeError, match="Cannot access output directory"):
        dummy_gen_success._validate_environment()


@patch("pathlib.Path.unlink")
def test_missing_metrics_file(mock_unlink, dummy_gen_success: DummyGenerator):
    """
    Simulate a missing metrics file or an OSError while unlinking,
    ensuring that scenario doesn't kill the entire initialization.
    """
    # Make unlink raise an OSError.
    mock_unlink.side_effect = OSError("No such file or directory")
    # Re-run initialization phase to trigger the unlink attempt
    # (it should catch or handle the error without failing)
    dummy_gen_success._initialization_phase()
    # Verify it still sets up the metrics handler
    assert hasattr(
        dummy_gen_success, "metrics_handler"
    ), "MetricsHandler should be initialized."
