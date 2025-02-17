import pytest
from unittest.mock import Mock, patch
from mistralai import Mistral
from codestral_ros2_gen.models.mistral_client import MistralClient, ModelUsage


@pytest.fixture
def mock_response():
    """Create a mock Mistral API response"""
    mock = Mock()
    mock.choices = [Mock(message=Mock(content="Generated test response"))]
    mock.usage = Mock(prompt_tokens=10, completion_tokens=20, total_tokens=30)
    return mock


@pytest.fixture
def mock_mistral():
    """Create a mock for the Mistral class"""
    with patch(
        "codestral_ros2_gen.models.mistral_client.Mistral", autospec=True
    ) as mock:
        instance = mock.return_value
        chat = Mock()
        instance.chat = chat
        instance.chat.complete = Mock()
        yield mock


def test_mistral_client_initialization(mock_mistral):
    """Test MistralClient initialization with config"""
    config = {
        "model": {
            "type": "test-model",
            "api_key": "test-key",
            "parameters": {"temperature": 0.5},
        }
    }

    client = MistralClient(api_key="test-key", config=config)
    mock_mistral.assert_called_once_with(api_key="test-key")
    assert client.config == config["model"]


@patch.dict("os.environ", {}, clear=True)
def test_mistral_client_missing_api_key():
    """Test MistralClient initialization without API key"""
    with pytest.raises(RuntimeError) as exc_info:
        MistralClient(config={"model": {}})
    assert "Mistral API key not found" in str(exc_info.value)


def test_complete_basic(mock_mistral, mock_response):
    """Test basic completion functionality"""
    # Setup mock
    instance = mock_mistral.return_value
    instance.chat.complete.return_value = mock_response

    client = MistralClient(api_key="test-key")

    response, usage = client.complete(
        prompt="Test prompt", system_prompt="Test system prompt"
    )

    assert response == "Generated test response"
    assert isinstance(usage, ModelUsage)
    assert usage.prompt_tokens == 10
    assert usage.completion_tokens == 20
    assert usage.total_tokens == 30

    # Verify correct message structure was passed to API
    instance.chat.complete.assert_called_once()

    call_kwargs = instance.chat.complete.call_args.kwargs
    assert call_kwargs["model"] == "codestral-latest"
    assert len(call_kwargs["messages"]) == 2
    assert call_kwargs["messages"][0]["role"] == "system"
    assert call_kwargs["messages"][0]["content"] == "Test system prompt"
    assert call_kwargs["messages"][1]["role"] == "user"
    assert call_kwargs["messages"][1]["content"] == "Test prompt"
    assert call_kwargs["temperature"] == 0.2


def test_complete_with_connection_error(mock_mistral):
    """Test handling of connection errors"""
    instance = mock_mistral.return_value
    instance.chat.complete.side_effect = ConnectionError("Connection failed")

    client = MistralClient(api_key="test-key")

    with pytest.raises(ConnectionError) as exc_info:
        client.complete("Test prompt")
    assert "Failed to connect to Mistral API" in str(exc_info.value)


def test_complete_with_invalid_response(mock_mistral):
    """Test handling of invalid API response"""
    instance = mock_mistral.return_value
    mock_invalid_response = Mock(choices=None)
    instance.chat.complete.return_value = mock_invalid_response

    client = MistralClient(api_key="test-key")

    with pytest.raises(RuntimeError) as exc_info:
        client.complete("Test prompt")
    assert "Invalid response format from API" in str(exc_info.value)


def test_complete_with_custom_model_type(mock_mistral, mock_response):
    """Test completion with custom model type override"""
    instance = mock_mistral.return_value
    instance.chat.complete.return_value = mock_response

    client = MistralClient(api_key="test-key")
    custom_model = "mistral-custom-model"

    response, usage = client.complete(prompt="Test prompt", model_type=custom_model)

    call_kwargs = instance.chat.complete.call_args.kwargs
    assert call_kwargs["model"] == custom_model


def test_complete_with_custom_temperature(mock_mistral, mock_response):
    """Test completion with custom temperature override"""
    instance = mock_mistral.return_value
    instance.chat.complete.return_value = mock_response

    client = MistralClient(api_key="test-key")
    custom_temp = 0.8

    response, usage = client.complete(prompt="Test prompt", temperature=custom_temp)

    call_kwargs = instance.chat.complete.call_args.kwargs
    assert call_kwargs["temperature"] == custom_temp


def test_complete_with_config_temperature(mock_mistral, mock_response):
    """Test completion using temperature from config"""
    instance = mock_mistral.return_value
    instance.chat.complete.return_value = mock_response

    config = {"model": {"parameters": {"temperature": 0.7}}}

    client = MistralClient(api_key="test-key", config=config)
    response, usage = client.complete(prompt="Test prompt")

    call_kwargs = instance.chat.complete.call_args.kwargs
    assert call_kwargs["temperature"] == 0.7


def test_api_key_precedence(mock_mistral):
    """Test API key precedence (direct > env > config)"""
    # Test direct API key
    direct_key = "direct-key"
    env_key = "env-key"
    config_key = "config-key"

    with patch.dict("os.environ", {"MISTRAL_API_KEY": env_key}):
        config = {"model": {"api_key": config_key}}

        # Direct key should take precedence
        client = MistralClient(api_key=direct_key, config=config)
        assert client.api_key == direct_key

        # Env key should take precedence over config
        client = MistralClient(config=config)
        assert client.api_key == env_key

        # Config key should be used if no other keys available
        with patch.dict("os.environ", {}, clear=True):
            client = MistralClient(config=config)
            assert client.api_key == config_key


def test_empty_system_prompt(mock_mistral, mock_response):
    """Test completion without system prompt"""
    instance = mock_mistral.return_value
    instance.chat.complete.return_value = mock_response

    client = MistralClient(api_key="test-key")
    response, usage = client.complete(prompt="Test prompt")

    call_kwargs = instance.chat.complete.call_args.kwargs
    assert len(call_kwargs["messages"]) == 1
    assert call_kwargs["messages"][0]["role"] == "user"


@pytest.mark.parametrize(
    "error_response",
    [
        {"error": "Rate limit exceeded"},
        {"error": "Invalid request"},
        {"error": "Server error"},
    ],
)
def test_api_error_handling(mock_mistral, error_response):
    """Test handling of various API errors"""
    instance = mock_mistral.return_value
    instance.chat.complete.side_effect = Exception(str(error_response))

    client = MistralClient(api_key="test-key")

    with pytest.raises(RuntimeError) as exc_info:
        client.complete("Test prompt")
    assert "API error" in str(exc_info.value)
    assert str(error_response) in str(exc_info.value)


def test_long_prompt_handling(mock_mistral, mock_response):
    """Test handling of very long prompts"""
    instance = mock_mistral.return_value
    instance.chat.complete.return_value = mock_response

    client = MistralClient(api_key="test-key")
    long_prompt = "test " * 1000  # Very long prompt

    response, usage = client.complete(prompt=long_prompt)

    # Verify the call was made with the full prompt
    call_kwargs = instance.chat.complete.call_args.kwargs
    assert call_kwargs["messages"][0]["content"] == long_prompt


def test_token_usage_validation(mock_mistral):
    """Test validation of token usage statistics"""
    # Create a mock response with invalid token counts
    mock_response = Mock()
    mock_response.choices = [Mock(message=Mock(content="Test response"))]
    mock_response.usage = Mock(
        prompt_tokens=-1,  # Invalid token count
        completion_tokens=-5,  # Invalid token count
        total_tokens=-10,  # Invalid total
    )

    instance = mock_mistral.return_value
    instance.chat.complete.return_value = mock_response

    client = MistralClient(api_key="test-key")
    response, usage = client.complete("Test prompt")

    # Verify usage statistics are corrected
    assert usage.prompt_tokens == 0  # Negative values should be converted to 0
    assert usage.completion_tokens == 0  # Negative values should be converted to 0
    assert usage.total_tokens == 0  # Should be 0 since both components are 0


def test_token_usage_consistency(mock_mistral):
    """Test token usage consistency validation"""
    # Test cases with inconsistent token counts
    test_cases = [
        # (prompt_tokens, completion_tokens, reported_total, expected_total)
        (10, 20, 15, 30),  # reported_total less than sum
        (10, 20, 40, 40),  # reported_total more than sum
        (-1, 20, 30, 20),  # negative prompt_tokens
        (10, -5, 30, 10),  # negative completion_tokens
        (10, 20, -10, 30),  # negative total_tokens
    ]

    client = MistralClient(api_key="test-key")

    for prompt, completion, reported_total, expected_total in test_cases:
        mock_response = Mock()
        mock_response.choices = [Mock(message=Mock(content="Test response"))]
        mock_response.usage = Mock(
            prompt_tokens=prompt,
            completion_tokens=completion,
            total_tokens=reported_total,
        )

        instance = mock_mistral.return_value
        instance.chat.complete.return_value = mock_response

        _, usage = client.complete("Test prompt")

        # Verify token count corrections
        assert usage.prompt_tokens >= 0
        assert usage.completion_tokens >= 0
        assert usage.total_tokens >= 0
        assert usage.total_tokens >= usage.prompt_tokens + usage.completion_tokens


def test_empty_prompt_handling(mock_mistral):
    """Test handling of empty prompts"""
    instance = mock_mistral.return_value
    # We don't need to set up the complete response since it should raise before API call

    client = MistralClient(api_key="test-key")

    with pytest.raises(ValueError) as exc_info:
        client.complete("")
    assert "Empty prompt provided" in str(exc_info.value)

    with pytest.raises(ValueError) as exc_info:
        client.complete("   ")
    assert "Empty prompt provided" in str(exc_info.value)
