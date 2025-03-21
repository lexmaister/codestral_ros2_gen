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
    # Override NetworkScanner with DummyScanner
    monkeypatch.setattr(nscan, "NetworkScanner", DummyScanner)

    # Prepare dummy command-line arguments
    testargs = ["nscan.py", "192.168.1.0/24", "--show-all", "-t", "3"]
    monkeypatch.setattr(sys, "argv", testargs)

    # Run main
    ret = nscan.main()

    # Capture standard output and assert the expected dummy output
    captured = capsys.readouterr().out
    assert "formatted dummy results" in captured


def test_missing_target(monkeypatch):
    # Test that missing required target argument causes a SystemExit
    testargs = ["nscan.py"]
    monkeypatch.setattr(sys, "argv", testargs)
    with pytest.raises(SystemExit):
        nscan.main()
