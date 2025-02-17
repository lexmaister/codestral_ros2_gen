import pytest
from pathlib import Path
import sys

# Get project root directory
PROJECT_ROOT = Path(__file__).parent.parent.resolve()

# Add project root to Python path
sys.path.insert(0, str(PROJECT_ROOT))


@pytest.fixture
def project_root():
    """Fixture providing project root path."""
    return PROJECT_ROOT


@pytest.fixture
def project_config(project_root):
    """Fixture to load project config.yaml."""
    import yaml

    config_path = project_root / "config" / "config.yaml"
    if not config_path.exists():
        pytest.skip(f"Config file not found at {config_path}")

    with open(config_path, "r") as f:
        return yaml.safe_load(f)
