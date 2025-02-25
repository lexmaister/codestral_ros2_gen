import pytest
from pathlib import Path
from codestral_ros2_gen.generators.generation_attempt import (
    GenerationAttempt,
    AttemptState,
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
