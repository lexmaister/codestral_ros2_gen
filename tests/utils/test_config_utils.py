import pytest
from codestral_ros2_gen.utils.config_utils import validate_config_keys


def test_validate_config_keys_valid():
    """Test validate_config_keys with valid configuration."""
    config = {
        "section1": {"key1": "value1", "key2": "value2", "extra_key": "extra_value"}
    }

    # Should not raise an error when all required keys are present
    validate_config_keys(config, "section1", ("key1", "key2"))
    # Should also work with a single key
    validate_config_keys(config, "section1", ("key1",))
    # Should not care about extra keys
    validate_config_keys(config, "section1", tuple())


def test_validate_config_keys_missing_section():
    """Test validate_config_keys with missing section."""
    config = {"section1": {"key1": "value1"}}

    # Should raise RuntimeError for missing section
    with pytest.raises(RuntimeError, match="Missing section 'missing_section'"):
        validate_config_keys(config, "missing_section", ("key1",))


def test_validate_config_keys_missing_single_key():
    """Test validate_config_keys with a single missing key."""
    config = {"section1": {"key1": "value1"}}

    # Should raise RuntimeError for missing key
    with pytest.raises(RuntimeError, match="Missing required keys in 'section1': key2"):
        validate_config_keys(config, "section1", ("key1", "key2"))


def test_validate_config_keys_missing_multiple_keys():
    """Test validate_config_keys with multiple missing keys."""
    config = {"section1": {"key1": "value1"}}

    # Should raise RuntimeError for missing keys with proper formatting
    with pytest.raises(
        RuntimeError, match="Missing required keys in 'section1': key2, key3, key4"
    ):
        validate_config_keys(config, "section1", ("key1", "key2", "key3", "key4"))


def test_validate_config_keys_empty_section():
    """Test validate_config_keys with empty section."""
    config = {"empty_section": {}}

    # Should raise error when required keys are specified but section is empty
    with pytest.raises(
        RuntimeError, match="Missing required keys in 'empty_section': key1"
    ):
        validate_config_keys(config, "empty_section", ("key1",))

    # Should pass when no required keys are specified
    validate_config_keys(config, "empty_section", tuple())
