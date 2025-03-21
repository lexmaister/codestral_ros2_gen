import sys
import pytest
from codestral_ros2_gen.network_scan import nscan


# Create a dummy scanner to override the actual scanning behavior
class DummyScanner:
    def __init__(self, timeout, logger):
        self.timeout = timeout
        self.logger = logger

    def scan(self, target):
        # Return a dummy result for testing
        return {"dummy": True}

    def format_results(self, results, show_all=False):
        # Return a fixed output for testing
        return "formatted dummy results"


def test_main(monkeypatch, capsys):
    import logging

    # Reconfigure logger so that its output is captured by capsys
    logger = logging.getLogger("nscan")
    logger.handlers = []  # remove previously added handlers
    handler = logging.StreamHandler(sys.stdout)
    logger.addHandler(handler)

    monkeypatch.setattr(nscan, "NetworkScanner", DummyScanner)
    testargs = ["nscan.py", "192.168.1.0/24", "--show-all", "-t", "3"]
    monkeypatch.setattr(sys, "argv", testargs)

    nscan.main()
    captured = capsys.readouterr().out
    assert "formatted dummy results" in captured


def test_missing_target(monkeypatch):
    # Test that missing required target argument causes a SystemExit
    testargs = ["nscan.py"]
    monkeypatch.setattr(sys, "argv", testargs)
    with pytest.raises(SystemExit):
        nscan.main()


def test_save_json(monkeypatch, tmp_path):
    import importlib

    module = importlib.import_module("codestral_ros2_gen.network_scan.nscan")
    monkeypatch.setattr(module, "NetworkScanner", DummyScanner)

    # Prepare dummy command-line arguments with out_path set to tmp_path directory
    test_dir = str(tmp_path)
    testargs = ["nscan.py", "192.168.1.0/24", "-t", "3", "-o", test_dir]
    monkeypatch.setattr(sys, "argv", testargs)

    # Run main from the module
    module.main()

    # Check that the JSON file exists and contains the expected dummy result
    result_file = tmp_path / "nscan_results.json"
    assert result_file.exists(), f"Expected JSON file {result_file} not found"
    import json

    with result_file.open() as f:
        data = json.load(f)
    assert data == {"dummy": True}
