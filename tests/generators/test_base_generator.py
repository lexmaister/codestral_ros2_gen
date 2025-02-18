import pytest
from pathlib import Path
from unittest.mock import patch
from codestral_ros2_gen.generators.base_generator import BaseGenerator
from codestral_ros2_gen.models.mistral_client import ModelUsage


class MockGenerator(BaseGenerator):
    """Simple mock generator for testing."""

    def prepare_prompt(self, **kwargs) -> str:
        return "mock prompt"

    def save_output(self, code: str, output_path: Path) -> bool:
        return "error" not in code


@pytest.fixture
def generator(tmp_path):
    """Create mock generator with basic config using temporary directory."""
    metrics_file = tmp_path / "metrics" / "test_metrics.jsonl"
    with patch(
        "codestral_ros2_gen.generators.base_generator.load_config"
    ) as mock_config:
        mock_config.return_value = {
            "generation": {"max_attempts": 2},
            "metrics": {"output_file": str(metrics_file)},
        }
        return MockGenerator(metrics_file=str(metrics_file))


def test_successful_generation(generator, tmp_path):
    """Test successful code generation flow."""
    with (
        patch.object(
            generator.model,
            "complete",
            return_value=("def test(): pass", ModelUsage(10, 20, 30)),
        ),
        patch.object(generator, "run_tests", return_value=(True, "")),
    ):
        success, metrics = generator.generate(tmp_path / "test.py")

        assert success is True
        assert metrics["attempts"] == 1
        assert not metrics["errors"]
        assert metrics["token_usage"]["prompt_tokens"] == 10
        assert metrics["token_usage"]["completion_tokens"] == 20
        assert metrics["token_usage"]["total_tokens"] == 30


def test_failed_tests(generator, tmp_path):
    """Test generation with failing tests."""
    with (
        patch.object(
            generator.model,
            "complete",
            return_value=("def test(): pass", ModelUsage(10, 20, 30)),
        ),
        patch.object(generator, "run_tests", return_value=(False, "Test error")),
    ):

        success, metrics = generator.generate(tmp_path / "test.py")

        assert success is False
        assert "test_failure" in metrics["errors"][0]
        assert metrics["attempts"] > 0


def test_max_attempts_reached(generator, tmp_path):
    """Test reaching maximum generation attempts."""
    with patch.object(
        generator.model,
        "complete",
        return_value=("def error(): pass", ModelUsage(10, 20, 30)),
    ):

        success, metrics = generator.generate(tmp_path / "test.py")

        assert success is False
        assert metrics["attempts"] == 2  # max_attempts from config
        assert len(metrics["errors"]) > 0


def test_metrics_file_location(generator, tmp_path):
    """Verify metrics are saved to the temporary directory."""
    metrics_file = Path(generator.metrics_handler.metrics_file)
    assert tmp_path in metrics_file.parents
    assert metrics_file.parent.exists()
