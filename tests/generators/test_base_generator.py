import pytest
from unittest.mock import MagicMock, patch

from codestral_ros2_gen.generators.base_generator import BaseGenerator


# Test implementation of BaseGenerator for testing
class TestGenerator(BaseGenerator):
    """Concrete implementation of BaseGenerator for testing."""

    def prepare_prompt(self, **kwargs):
        """Implementation of abstract method for testing."""
        return "Test prompt" + (f" with {kwargs}" if kwargs else "")


@pytest.fixture
def mock_generation_attempt_success(sample_attempt_metrics):
    """Mock GenerationAttempt class run method that returns success."""
    mock = MagicMock()
    mock.return_value.run.return_value = (True, sample_attempt_metrics)
    return mock


@pytest.fixture
def mock_generation_attempt_failure():
    """Mock GenerationAttempt class run method that raises RuntimeError."""
    mock = MagicMock()
    mock.side_effect = RuntimeError("Generation failed.")
    return mock


@pytest.fixture
def mock_metrics_handler():
    """Mock MetricsHandler class."""
    mock = MagicMock()
    mock.return_value.record_attempt.return_value = None
    return mock


@pytest.fixture
def mock_model_client():
    """Mock MistralClient class."""
    mock = MagicMock()
    mock.return_value.complete.return_value = "Generated code"
    return mock


def test_initialization(sample_config_path):
    """Test basic initialization of BaseGenerator."""
    generator = TestGenerator(config_path=sample_config_path)
    assert generator.config_path == sample_config_path
    assert generator.config is None
    assert isinstance(generator.start_time, float)


def test_initialization_with_dependency_injection(
    sample_config_path,
    mock_generation_attempt_success,
    mock_metrics_handler,
    mock_model_client,
):
    """Test initialization with dependency injection."""
    generator = TestGenerator(
        config_path=sample_config_path,
        generation_attempt_class=mock_generation_attempt_success,
        metrics_handler_class=mock_metrics_handler,
        model_client_class=mock_model_client,
    )

    assert generator.GenerationAttemptClass == mock_generation_attempt_success
    assert generator.MetricsHandlerClass == mock_metrics_handler
    assert generator.ModelClientClass == mock_model_client


def test_prepare_prompt():
    """Test the prepare_prompt implementation."""
    generator = TestGenerator()
    assert generator.prepare_prompt() == "Test prompt"
    assert generator.prepare_prompt(key="value") == "Test prompt with {'key': 'value'}"


def test_validate_environment(sample_config, sample_config_path):
    """Test environment validation with valid configuration."""
    generator = TestGenerator(config_path=sample_config_path)
    generator._initialization_phase()
    assert generator.config == sample_config


def test_validate_environment_missing_section(sample_config_path):
    """Test environment validation with missing required section."""
    invalid_config = {"generation": {"max_attempts": 3}}  # Missing output section

    generator = TestGenerator(config_path=sample_config_path)
    generator._initialization_phase()
    generator.config = invalid_config
    with pytest.raises(RuntimeError, match="Missing required keys"):
        generator._validate_environment()


def test_initialization_phase(
    sample_config,
    sample_config_path,
    tmp_path,
    mock_metrics_handler,
    mock_model_client,
):
    """Test the initialization phase."""
    # Create a temporary config with all required sections
    temp_config = sample_config.copy()
    metrics_file = str(tmp_path / "metrics.jsonl")
    temp_config["metrics"] = {"metrics_file": metrics_file}
    temp_config["output"] = {"output_file": str(tmp_path / "output.py")}
    temp_config["generation"] = {"max_attempts": 3, "evaluation_iterations": 2}

    with patch(
        "codestral_ros2_gen.generators.base_generator.load_config",
        return_value=temp_config,
    ):
        generator = TestGenerator(
            config_path=sample_config_path,
            metrics_handler_class=mock_metrics_handler,
            model_client_class=mock_model_client,
        )
        generator._initialization_phase()

        assert generator.max_attempts == 3
        assert generator.evaluation_iterations == 2
        mock_metrics_handler.assert_called_once_with(metrics_file=metrics_file)
        mock_model_client.assert_called_once_with(config=temp_config)


def test_generation_phase(
    sample_config_path,
    mock_metrics_handler,
    mock_model_client,
    mock_generation_attempt_success,
    sample_attempt_metrics,
):
    """Test the generation phase with successful generation."""
    generator = TestGenerator(
        config_path=sample_config_path,
        metrics_handler_class=mock_metrics_handler,
        model_client_class=mock_model_client,
        generation_attempt_class=mock_generation_attempt_success,
    )

    # Set up the generator with configuration
    generator._initialization_phase()
    # change the max attempts to 1 and evaluation iterations to 1
    generator.max_attempts = 1
    generator.evaluation_iterations = 1

    # Test generation phase
    generator._generation_phase()

    # Check that generation_attempt was called
    mock_generation_attempt_success.assert_called()
    # Check that the metrics handler was called with the correct metrics
    mock_metrics_handler.return_value.record_attempt.assert_called_once_with(
        iteration_number=1,
        attempt_number=1,
        attempt_metrics=sample_attempt_metrics,
    )


def test_generation_phase_with_failures(
    sample_config_path,
    mock_metrics_handler,
    mock_model_client,
    mock_generation_attempt_failure,
):
    """Test the generation phase with failure attempt."""
    generator = TestGenerator(
        config_path=sample_config_path,
        metrics_handler_class=mock_metrics_handler,
        model_client_class=mock_model_client,
        generation_attempt_class=mock_generation_attempt_failure,
    )
    # There shouldn't be any exception
    generator.run(prompt="Hello!")
    # Check that generation_attempt was called
    mock_generation_attempt_failure.assert_called_once()
