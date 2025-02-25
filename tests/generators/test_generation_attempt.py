import pytest
from pathlib import Path
import pandas as pd
from codestral_ros2_gen.generators.generation_attempt import (
    GenerationAttempt,
    AttemptState,
    AttemptMetrics,
)
from codestral_ros2_gen.models.mistral_client import ModelUsage


# Dummy MistralClient for testing failure in parsing.
class FailingDummyMistralClient:
    def __init__(self, api_key, config):
        self.api_key = api_key
        self.config = config

    def complete(self, prompt: str):
        # Return invalid code that does not conform to expected formatting.
        return "print('invalid code", ModelUsage(
            prompt_tokens=5, completion_tokens=10, total_tokens=15
        )


def dummy_save_callback(code: str, output_path: Path) -> bool:
    return True


@pytest.fixture
def sample_config():
    return {
        "generation": {"max_attempts": 10, "evaluation_iterations": 10},
        "test": {
            "ws_setup": "echo 'Setting up workspace'",
            "node_command": "echo 'Running node'",
            "test_command": "echo 'Running tests'",
        },
    }


def test_generation_attempt_success(sample_config):
    # Using a dummy client that returns valid code (assuming ROS2CodeParser can parse it)
    class DummyMistralClient:
        def __init__(self, api_key, config):
            self.api_key = api_key
            self.config = config

        def complete(self, prompt: str):
            dummy_code = "```python\ndef hello():\n    print('Hello')\n```"
            usage = ModelUsage(prompt_tokens=5, completion_tokens=10, total_tokens=15)
            return dummy_code, usage

    dummy_client = DummyMistralClient(api_key="dummy", config=sample_config)
    print(sample_config)

    class TestsGenerationAttempt(GenerationAttempt):
        def __init__(self, client, config):
            super().__init__(client, config)

        # skip running tests for this test due unable run ros2 commands
        def _test(self):
            self.state = AttemptState.SUCCESS

    attempt_instance = TestsGenerationAttempt(dummy_client, sample_config)
    success, metrics = attempt_instance.run(
        Path("dummy_output.py"), "dummy prompt", dummy_save_callback
    )
    assert success is True
    # Use attribute access instead of dictionary indexing.
    assert metrics.final_state == AttemptState.SUCCESS.name


def test_generation_attempt_failure_in_parsing(sample_config):
    dummy_client = FailingDummyMistralClient(api_key="dummy", config=sample_config)
    attempt_instance = GenerationAttempt(dummy_client, sample_config)
    success, metrics = attempt_instance.run(
        Path("dummy_output.py"), "dummy prompt", dummy_save_callback
    )
    assert success is False
    # Use attribute access for metric (for example, final_state attribute)
    assert metrics.final_state == AttemptState.PARSE.name
    assert "Parsing failed" in attempt_instance.error


# TESTS FOR ATTEMPTS METRICS


def test_attempt_metrics_creation():
    """Test basic creation of AttemptMetrics instance"""
    metrics = AttemptMetrics(
        attempt_time=1.5,
        success=True,
        final_state="SUCCESS",
        tests_passed=10,
        tests_failed=0,
        tests_skipped=2,
        prompt_tokens=100,
        completion_tokens=50,
        total_tokens=150,
    )

    assert metrics.attempt_time == 1.5
    assert metrics.success is True
    assert metrics.final_state == "SUCCESS"
    assert metrics.error is None


def test_as_dict_property():
    """Test the as_dict property returns correct dictionary"""
    metrics = AttemptMetrics(
        attempt_time=1.5,
        success=True,
        final_state="SUCCESS",
        tests_passed=10,
        tests_failed=0,
        tests_skipped=2,
        prompt_tokens=100,
        completion_tokens=50,
        total_tokens=150,
    )

    result = metrics.as_dict
    assert isinstance(result, dict)
    assert result["attempt_time"] == 1.5
    assert result["success"] is True
    assert result["error"] is None


def test_as_series_property():
    """Test the as_series property returns correct pandas Series"""
    metrics = AttemptMetrics(
        attempt_time=1.5,
        success=True,
        final_state="SUCCESS",
        tests_passed=10,
        tests_failed=0,
        tests_skipped=2,
        prompt_tokens=100,
        completion_tokens=50,
        total_tokens=150,
    )

    result = metrics.as_series
    assert isinstance(result, pd.Series)
    assert list(result.index) == metrics._fields_order  # Check order
    assert result["attempt_time"] == 1.5


def test_error_state():
    """Test metrics with error state"""
    metrics = AttemptMetrics(
        attempt_time=1.5,
        success=False,
        final_state="FAILURE",
        tests_passed=5,
        tests_failed=5,
        tests_skipped=0,
        prompt_tokens=100,
        completion_tokens=50,
        total_tokens=150,
        error="Test error message",
    )

    assert metrics.success is False
    assert metrics.error == "Test error message"


@pytest.fixture
def sample_metrics():
    """Fixture providing a sample metrics instance"""
    return AttemptMetrics(
        attempt_time=1.5,
        success=True,
        final_state="SUCCESS",
        tests_passed=10,
        tests_failed=0,
        tests_skipped=2,
        prompt_tokens=100,
        completion_tokens=50,
        total_tokens=150,
    )


def test_series_order(sample_metrics):
    """Test that Series maintains the defined field order"""
    series = sample_metrics.as_series
    assert list(series.index) == AttemptMetrics._fields_order


def test_dict_values(sample_metrics):
    """Test dictionary values match original inputs"""
    d = sample_metrics.as_dict
    assert d["attempt_time"] == 1.5
    assert d["tests_passed"] == 10
    assert d["total_tokens"] == 150


@pytest.mark.parametrize("field", AttemptMetrics._fields_order)
def test_field_presence(sample_metrics, field):
    """Test all fields are present in both dict and series"""
    assert field in sample_metrics.as_dict
    assert field in sample_metrics.as_series


def test_dataframe_creation():
    """Test creation of DataFrame from multiple metrics"""
    metrics_list = []
    for i in range(3):
        metric = AttemptMetrics(
            attempt_time=1.5 + i,
            success=True,
            final_state="SUCCESS",
            tests_passed=10,
            tests_failed=0,
            tests_skipped=2,
            prompt_tokens=100,
            completion_tokens=50,
            total_tokens=150,
        )
        metrics_list.append(metric.as_series)

    df = pd.DataFrame(metrics_list)
    assert len(df) == 3
    assert list(df.columns) == AttemptMetrics._fields_order
    assert df["attempt_time"].tolist() == [1.5, 2.5, 3.5]
