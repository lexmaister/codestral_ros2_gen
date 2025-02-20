import pytest
import time
from pathlib import Path
from unittest.mock import patch, Mock, MagicMock
import subprocess  # Add this import

from codestral_ros2_gen.generators.base_generator import BaseGenerator
from codestral_ros2_gen.models.mistral_client import ModelUsage


class MockGenerator(BaseGenerator):
    """Simple mock generator for testing."""

    def prepare_prompt(self, **kwargs) -> str:
        return "mock prompt"

    def save_output(self, code: str, output_path: Path) -> bool:
        # Simple validation without filesystem operations
        return bool(code and not "error" in code)

    def run_tests(self, package_path: Path) -> tuple[bool, str]:
        """Mock test execution with proper build handling."""
        if getattr(self, "build_workspace", False):
            # Actually call subprocess.run to trigger the mock
            subprocess.run(["colcon", "build"], check=True)
        return True, ""


@pytest.fixture
def mock_config():
    """Provide test configuration."""
    return {
        "generation": {
            "max_attempts": 2,
            "timeout": 30,
        },
        "metrics": {
            "output_file": "test_metrics.jsonl",
            "collect": {
                "generation_time": True,
                "attempts_until_success": True,
                "token_usage": True,
            },
        },
        "model": {
            "type": "test-model",
            "parameters": {"temperature": 0.1},
        },
    }


@pytest.fixture
def mock_subprocess():
    """Mock all subprocess calls."""
    with patch("subprocess.run") as mock_run:
        mock_run.return_value = Mock(returncode=0)
        yield mock_run


@pytest.fixture
def mock_pytest_main():
    """Mock pytest.main calls."""
    with patch("pytest.main") as mock_main:
        mock_main.return_value = 0  # ExitCode.OK
        yield mock_main


@pytest.fixture
def generator(tmp_path, mock_config, mock_subprocess, mock_pytest_main):
    """Create mock generator that doesn't need ROS2."""
    metrics_file = tmp_path / "metrics" / "test_metrics.jsonl"

    # Mock all external dependencies
    with (
        patch(
            "codestral_ros2_gen.generators.base_generator.load_config",
            return_value=mock_config,
        ),
        patch(
            "codestral_ros2_gen.generators.base_generator.MistralClient"
        ) as mock_client,
    ):
        mock_client.return_value.complete.return_value = (
            "def test(): pass",
            ModelUsage(10, 20, 30),
        )
        gen = MockGenerator(metrics_file=str(metrics_file), api_key="test_key")
        return gen


def test_successful_generation_with_timeout(generator, tmp_path):
    """Test successful generation with proper timeout handling."""
    mock_usage = ModelUsage(10, 20, 30)

    with (
        patch.object(
            generator.model,
            "complete",
            return_value=("def test(): pass", mock_usage),
        ) as mock_complete,
        patch.object(generator, "run_tests", return_value=(True, "")),
    ):
        success, metrics = generator.generate(
            tmp_path / "test.py", timeout=10.0, max_attempts=2
        )

        assert success is True
        assert metrics["attempts"] == 1
        assert not metrics["errors"]
        assert metrics["token_usage"]["total_tokens"] == 30
        assert metrics["config_used"]["timeout"] == 10.0
        assert metrics["config_used"]["max_attempts"] == 2
        assert (
            metrics["config_used"]["per_attempt_timeout"] >= 5.0
        )  # minimum 10s per attempt
        assert len(metrics["attempt_timers"]) == 1


def test_timeout_handling_per_stage(generator, tmp_path, mock_subprocess):
    """Test timeout handling for each stage."""

    def mock_complete(*args, **kwargs):
        return "def test(): pass", ModelUsage(10, 20, 30)

    with (
        patch.object(generator.model, "complete", side_effect=mock_complete),
        patch.object(generator, "run_tests", wraps=generator.run_tests),
        patch(
            "codestral_ros2_gen.utils.code_parser.ROS2CodeParser.parse",
            return_value="def test(): pass",
        ),
    ):
        success, metrics = generator.generate(
            tmp_path / "test.py", timeout=3.0, build_workspace=True
        )

        assert success is True
        assert metrics.get("stage_results") == ["GenCode", "SaveCode", "RunTests"]
        assert metrics["timeouts"]["attempts"] == 0
        assert mock_subprocess.called


def test_stage_timeout_failure(generator, tmp_path):
    """Test handling of timeout in different stages (model stage timeout)."""
    call_count = 0

    def mock_slow_complete(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        time.sleep(0.5)  # For total timeout=0.4, 0.5 > 0.4 triggers timeout
        return "def test(): pass", ModelUsage(10, 20, 30)

    with patch.object(generator.model, "complete", side_effect=mock_slow_complete):
        success, metrics = generator.generate(
            tmp_path / "test.py",
            timeout=0.4,  # full timeout = 0.4s
            max_attempts=1,
        )
        assert call_count > 0, "Slow mock function was never called"
        assert not success, "Should fail due to timeout"
        assert metrics["timeouts"]["attempts"] >= 1, "Should record timeout attempt"
        assert len(metrics["errors"]) >= 1, "Should have error recorded"
        assert any(
            "Model generation timed out" in err.lower() for err in metrics["errors"]
        ), "Should have timeout error"
        assert metrics["attempts"] == 1, "Should only make one attempt"
        assert (
            len(metrics["attempt_timers"]) == 1
        ), "Should record timing for the attempt"


def test_stage_timeout_failure(generator, tmp_path):
    """Test handling of timeout in different stages."""

    def mock_slow_complete(*args, **kwargs):
        time.sleep(0.5)  # Sleep just long enough to trigger timeout
        return "def test(): pass", ModelUsage(10, 20, 30)

    with patch.object(generator.model, "complete", side_effect=mock_slow_complete):
        success, metrics = generator.generate(
            tmp_path / "test.py",
            timeout=0.3,  # Total timeout for all attempts
            max_attempts=2,
        )

        assert success is False
        assert metrics["timeouts"]["attempts"] >= 1, "Should record timeout attempt"
        assert "timed out" in metrics["errors"][0].lower(), "Should have timeout error"
        assert len(metrics["attempt_timers"]) > 0, "Should record attempt timing"


def test_model_stage_timeout(generator, tmp_path):
    """Test timeout specifically in the model generation stage."""

    def mock_slow_model(*args, **kwargs):
        time.sleep(0.6)  # For total timeout=0.5, 0.6 > 0.5 triggers timeout
        return "def test(): pass", ModelUsage(10, 20, 30)

    with patch.object(generator.model, "complete", side_effect=mock_slow_model):
        success, metrics = generator.generate(
            tmp_path / "test.py",
            timeout=0.5,  # full timeout = 0.5s
            max_attempts=1,
        )
        assert not success
        assert metrics["timeouts"]["attempts"] >= 1
        assert any("Model generation timed out" in err for err in metrics["errors"])
        assert len(metrics["attempt_timers"]) == 1


def test_timeout_simple(generator, tmp_path):
    """Test basic timeout functionality."""

    def mock_slow_complete(*args, **kwargs):
        time.sleep(0.5)  # For total timeout=0.4, 0.5 > 0.4 triggers timeout
        return "def test(): pass", ModelUsage(10, 20, 30)

    with patch.object(generator.model, "complete", side_effect=mock_slow_complete):
        success, metrics = generator.generate(
            tmp_path / "test.py",
            timeout=0.4,  # full timeout = 0.4s
            max_attempts=1,
        )
        assert not success
        assert metrics["timeouts"]["attempts"] == 1
        assert any("Model generation timed out" in err for err in metrics["errors"])


def test_multiple_timeouts(generator, tmp_path):
    """Test multiple timeout attempts."""

    def mock_timeout(*args, **kwargs):
        time.sleep(0.5)  # For total timeout=0.4, 0.5 > 0.4 triggers timeout per attempt
        return "def test(): pass", ModelUsage(10, 20, 30)

    with patch.object(generator.model, "complete", side_effect=mock_timeout):
        success, metrics = generator.generate(
            tmp_path / "test.py",
            timeout=0.4,  # full timeout = 0.4s per attempt
            max_attempts=2,
        )
        assert not success
        assert metrics["timeouts"]["attempts"] == 2
        assert len(metrics["errors"]) == 2
        assert all("Model generation timed out" in err for err in metrics["errors"])


def test_multiple_timeout_attempts(generator, tmp_path):
    """Test timeout handling across multiple attempts."""
    call_count = 0

    def mock_timeout_model(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        time.sleep(0.5)  # For total timeout=0.4, 0.5 > 0.4 triggers timeout
        return "def test(): pass", ModelUsage(10, 20, 30)

    with patch.object(generator.model, "complete", side_effect=mock_timeout_model):
        success, metrics = generator.generate(
            tmp_path / "test.py",
            timeout=0.4,  # full timeout = 0.4s per attempt
            max_attempts=2,
        )
        assert call_count > 0
        assert not success
        assert metrics["timeouts"]["attempts"] >= 1
        assert metrics["attempts"] <= 2


def test_metrics_collection(generator, tmp_path):
    """Test comprehensive metrics collection."""
    mock_usage = ModelUsage(10, 20, 30)

    with (
        patch.object(
            generator.model,
            "complete",
            return_value=("def test(): pass", mock_usage),
        ),
        patch.object(generator, "run_tests", return_value=(True, "")),
    ):
        success, metrics = generator.generate(tmp_path / "test.py")

        # Check all required metrics are present
        assert "main_timer" in metrics
        assert "attempt_timers" in metrics
        assert "timeouts" in metrics
        assert "token_usage" in metrics
        assert "config_used" in metrics

        # Verify metrics values
        assert isinstance(metrics["main_timer"], float)
        assert len(metrics["attempt_timers"]) > 0
        assert all(isinstance(t, float) for t in metrics["attempt_timers"])
        assert metrics["token_usage"]["prompt_tokens"] == 10
        assert metrics["timeouts"]["attempts"] == 0


def test_error_recording(generator, tmp_path):
    """Test error recording in metrics."""
    error_msg = "API error during generation"

    def mock_failed_complete(*args, **kwargs):
        raise RuntimeError(error_msg)

    with patch.object(generator.model, "complete", side_effect=mock_failed_complete):
        success, metrics = generator.generate(tmp_path / "test.py", max_attempts=1)

        assert not success
        assert len(metrics["errors"]) >= 1
        assert error_msg in metrics["error_patterns"][0]
        assert metrics["attempts"] == 1


def test_metrics_file_creation(generator, tmp_path):
    """Test metrics file creation and format."""
    mock_usage = ModelUsage(10, 20, 30)

    with (
        patch.object(
            generator.model,
            "complete",
            return_value=("def test(): pass", mock_usage),
        ),
        patch.object(generator, "run_tests", return_value=(True, "")),
    ):
        generator.generate(tmp_path / "test.py")

        metrics_file = Path(generator.metrics_handler.metrics_file)
        assert metrics_file.exists()
        assert metrics_file.stat().st_size > 0


def test_build_operation_mocked(generator, tmp_path, mock_subprocess):
    """Verify that colcon build is properly mocked."""
    mock_usage = ModelUsage(10, 20, 30)

    with patch.object(
        generator.model,
        "complete",
        return_value=("def test(): pass", mock_usage),
    ):
        generator.generate(tmp_path / "test.py")

        # Verify subprocess.run was not actually called for colcon build
        assert not mock_subprocess.called, "Colcon build should be mocked"


def test_timeout_simple(generator, tmp_path):
    """Test basic timeout functionality without ROS dependencies."""

    def mock_slow_complete(*args, **kwargs):
        time.sleep(0.2)  # Short sleep that won't impact CI
        return "def test(): pass", ModelUsage(10, 20, 30)

    with patch.object(generator.model, "complete", side_effect=mock_slow_complete):
        success, metrics = generator.generate(
            tmp_path / "test.py", timeout=0.1, max_attempts=1  # Total timeout
        )

        assert success is False
        assert metrics["timeouts"]["attempts"] == 1
        assert any("timed out" in err.lower() for err in metrics["errors"])


def test_multiple_timeouts(generator, tmp_path):
    """Test handling of multiple timeout attempts."""

    def mock_timeout(*args, **kwargs):
        time.sleep(0.5)  # Longer than stage timeout
        return "def test(): pass", ModelUsage(10, 20, 30)

    with patch.object(generator.model, "complete", side_effect=mock_timeout):
        success, metrics = generator.generate(
            tmp_path / "test.py",
            timeout=0.3,  # Makes stage_timeout = 0.1s
            max_attempts=2,
        )

        assert not success
        assert metrics["timeouts"]["attempts"] == 2
        assert len(metrics["errors"]) == 2
        assert all("timed out" in err.lower() for err in metrics["errors"])


def test_no_ros_dependency(generator, tmp_path):
    """Verify that tests run without ROS2 environment."""
    success, metrics = generator.generate(tmp_path / "test.py")
    assert success is True  # Should work with mocked dependencies


def test_subprocess_not_called(generator, tmp_path, mock_subprocess):
    """Verify that no real subprocess calls are made."""
    generator.generate(tmp_path / "test.py")
    assert not mock_subprocess.called, "No real subprocess calls should be made"


def test_generate_without_build(generator, tmp_path):
    """Test generation without workspace building."""
    mock_usage = ModelUsage(10, 20, 30)

    with patch.object(
        generator.model,
        "complete",
        return_value=("def test(): pass", mock_usage),
    ):
        success, metrics = generator.generate(
            tmp_path / "test.py", build_workspace=False
        )

        assert success is True
        assert not metrics.get("build_workspace")
        assert not any("test" in err.lower() for err in metrics.get("errors", []))


def test_generate_with_build(generator, tmp_path, mock_subprocess):
    """Test generation with workspace building."""
    mock_usage = ModelUsage(10, 20, 30)

    with (
        patch.object(
            generator.model, "complete", return_value=("def test(): pass", mock_usage)
        ),
        patch(
            "codestral_ros2_gen.utils.code_parser.ROS2CodeParser.parse",
            return_value="def test(): pass",
        ),
    ):
        success, metrics = generator.generate(
            tmp_path / "test.py", build_workspace=True, timeout=5.0
        )

        assert success is True, f"Generation failed with metrics: {metrics}"
        assert metrics["build_workspace"] is True
        assert metrics.get("stage_results") == ["GenCode", "SaveCode", "RunTests"]
        assert (
            mock_subprocess.called
        ), "Colcon build should be called when build_workspace=True"
        mock_subprocess.assert_called_once()

        # Verify the subprocess call details
        args, kwargs = mock_subprocess.call_args
        assert kwargs.get(
            "check", False
        ), "subprocess.run should be called with check=True"
