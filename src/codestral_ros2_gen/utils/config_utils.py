from typing import Dict, Any, Tuple


def validate_config_keys(
    config: Dict[str, Any], section: str, required: Tuple[str, ...]
) -> None:
    """Raise error if any required keys are missing in the given section of config.

    :param config: Configuration dictionary to inspect
    :type config: Dict[str, Any]
    :param section: The section name within the config to check
    :type section: str
    :param required: Tuple of required keys that must exist in the section
    :type required: Tuple[str, ...]
    :raises RuntimeError: If section is missing or any required keys are not found
    """
    if section not in config:
        raise RuntimeError(
            f"Configuration error: Missing section '{section}' in configuration."
        )
    missing = [key for key in required if key not in config[section]]
    if missing:
        raise RuntimeError(
            f"Configuration error: Missing required keys in '{section}': {', '.join(missing)}"
        )
