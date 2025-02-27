import pytest
import time
from pathlib import Path
from unittest.mock import patch, MagicMock

from codestral_ros2_gen.generators.generation_attempt import (
    GenerationAttempt,
    AttemptState,
    AttemptMetrics,
)
from codestral_ros2_gen.models.mistral_client import ModelUsage


def dummy_save_callback(code: str, output_path: Path) -> bool:
    return True


@pytest.fixture
def success_mistral_client():
    """Fixture providing a client that returns valid, parsable code."""

    class SuccessMistralClient:
        def __init__(self, api_key, config):
            self.api_key = api_key
            self.config = config

        def complete(self, prompt: str):
            dummy_code = "```python\ndef hello():\n    print('Hello')\n```"
            usage = ModelUsage(
                prompt_tokens=100, completion_tokens=200, total_tokens=300
            )
            return dummy_code, usage

    return SuccessMistralClient


@pytest.fixture
def failing_mistral_client():
    """Fixture providing a client that returns unparsable code."""

    class FailingMistralClient:
        def __init__(self, api_key, config):
            self.api_key = api_key
            self.config = config

        def complete(self, prompt: str):
            # Return invalid code that does not conform to expected formatting
            return "print('invalid code", ModelUsage(
                prompt_tokens=50, completion_tokens=75, total_tokens=125
            )

    return FailingMistralClient


@pytest.fixture
def error_mistral_client():
    """Fixture providing a client that raises an exception."""

    class ErrorMistralClient:
        def __init__(self, api_key, config):
            self.api_key = api_key
            self.config = config

        def complete(self, prompt: str):
            raise RuntimeError("Forced error for testing")

    return ErrorMistralClient


@pytest.fixture
def test_generation_attempt():
    """Fixture providing a test-specific subclass of GenerationAttempt."""

    class TestGenerationAttempt(GenerationAttempt):
        def __init__(self, client, config):
            super().__init__(client, config)

        # Override _test method to avoid running actual ROS2 commands
        def _test(self):
            self.test_counts = (5, 0, 0)  # passed, failed, skipped
            self.state = AttemptState.SUCCESS

    return TestGenerationAttempt


def test_generation_attempt_success(
    sample_config, success_mistral_client, test_generation_attempt
):
    """Test successful generation attempt."""
    client = success_mistral_client(api_key="dummy", config=sample_config)
    attempt = test_generation_attempt(client, sample_config)

    success, metrics = attempt.run(
        Path("dummy_output.py"), "dummy prompt", dummy_save_callback
    )

    assert success is True
    assert metrics.final_state == AttemptState.SUCCESS.name
    assert metrics.success is True
    assert metrics.prompt_tokens == 100
    assert metrics.completion_tokens == 200
    assert metrics.total_tokens == 300
    assert metrics.tests_passed == 5
    assert metrics.tests_failed == 0
    assert metrics.tests_skipped == 0


def test_generation_attempt_failure_in_parsing(sample_config, failing_mistral_client):
    """Test generation attempt with parsing failure."""
    client = failing_mistral_client(api_key="dummy", config=sample_config)
    attempt = GenerationAttempt(client, sample_config)

    success, metrics = attempt.run(
        Path("dummy_output.py"), "dummy prompt", dummy_save_callback
    )

    assert success is False
    assert metrics.final_state == AttemptState.PARSE.name
    assert metrics.success is False
    assert "Parsing failed" in attempt.error
    assert metrics.prompt_tokens == 50
    assert metrics.completion_tokens == 75
    assert metrics.total_tokens == 125


def test_generation_attempt_client_error(sample_config, error_mistral_client):
    """Test generation attempt with client error."""
    client = error_mistral_client(api_key="dummy", config=sample_config)
    attempt = GenerationAttempt(client, sample_config)

    success, metrics = attempt.run(
        Path("dummy_output.py"), "dummy prompt", dummy_save_callback
    )

    assert success is False
    assert metrics.final_state == AttemptState.GENERATE.name
    assert metrics.success is False
    assert "Forced error for testing" in attempt.error


@pytest.mark.parametrize(
    "save_result,expected_state",
    [
        (True, AttemptState.TEST),  # Save succeeds, moves to TEST
        (False, AttemptState.FAILURE),  # Save fails, moves to FAILURE
    ],
)
def test_save_callback_behavior(
    sample_config, success_mistral_client, save_result, expected_state
):
    """Test behavior with different save callback results."""

    def mock_save_callback(code, path):
        return save_result

    # Create GenerationAttempt with mock client
    client = success_mistral_client(api_key="dummy", config=sample_config)
    attempt = GenerationAttempt(client, sample_config)

    # Set up the test to mimic a proper parse state
    attempt.state = AttemptState.PARSE  # Pretend we're in the PARSE state
    attempt.previous_state = AttemptState.GENERATE  # The state before PARSE
    attempt.generated_code = "def test(): pass"  # Mock generated code
    attempt.parsed_code = "def test(): pass"  # Mock parsed code

    # Call _save with the mock save callback and patch _test to prevent transition
    with patch.object(
        attempt, "_test", return_value=None
    ):  # Prevent actual _test from running
        attempt._save(Path("test.py"), mock_save_callback)  # Call the save method

    # Assert the state after execution
    assert attempt.state == expected_state
    if not save_result:
        assert "Saving failed" in attempt.error


@pytest.mark.parametrize(
    "method_name,expected_final_state",
    [
        ("_generate", AttemptState.PARSE),
        ("_parse", AttemptState.SAVE),
        ("_save", AttemptState.TEST),
    ],
)
def test_state_transitions(
    sample_config,
    success_mistral_client,
    method_name,
    expected_final_state,
):
    """Test state transitions between different phases."""
    client = success_mistral_client(api_key="dummy", config=sample_config)
    attempt = GenerationAttempt(client, sample_config)

    attempt.generated_code = "```python\ndef test(): pass```"
    attempt.parsed_code = "def test(): pass"

    # Call the method under test
    if method_name == "_generate":
        attempt._generate("test prompt")
    elif method_name == "_parse":
        attempt._parse()
    elif method_name == "_save":
        attempt._save(Path("test.py"), dummy_save_callback)
    elif method_name == "_test":
        attempt._test()

    # Assert final state
    assert attempt.state == expected_final_state


def test_attempt_metrics_to_dict():
    """Test that AttemptMetrics properly converts to dictionary."""
    metrics = AttemptMetrics(
        attempt_time=2.5,
        success=True,
        final_state="SUCCESS",
        tests_passed=10,
        tests_failed=0,
        tests_skipped=2,
        prompt_tokens=150,
        completion_tokens=250,
        total_tokens=400,
    )

    metrics_dict = metrics.as_dict

    assert isinstance(metrics_dict, dict)
    assert metrics_dict["attempt_time"] == 2.5
    assert metrics_dict["success"] is True
    assert metrics_dict["final_state"] == "SUCCESS"
    assert metrics_dict["tests_passed"] == 10
    assert metrics_dict["tests_failed"] == 0
    assert metrics_dict["tests_skipped"] == 2
    assert metrics_dict["prompt_tokens"] == 150
    assert metrics_dict["completion_tokens"] == 250
    assert metrics_dict["total_tokens"] == 400
