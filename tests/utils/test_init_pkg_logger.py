import pytest
from codestral_ros2_gen.utils import init_pkg_logger


class DummyLogger:
    pass


def test_init_pkg_logger_success(monkeypatch):
    dummy_logger = DummyLogger()
    monkeypatch.setattr(init_pkg_logger, "setup_logger", lambda x: dummy_logger)
    monkeypatch.setattr(init_pkg_logger, "logger_main", "dummy_main")
    result = init_pkg_logger.init_pkg_logger()
    assert result is dummy_logger


@pytest.mark.parametrize("exc", [FileNotFoundError, ValueError])
def test_init_pkg_logger_error(monkeypatch, exc):
    def raise_exception(x):
        raise exc("Test error")

    monkeypatch.setattr(init_pkg_logger, "setup_logger", raise_exception)
    monkeypatch.setattr(init_pkg_logger, "logger_main", "dummy_main")
    with pytest.raises(SystemExit):
        init_pkg_logger.init_pkg_logger()


def test_init_pkg_logger_unexpected_exception(monkeypatch):
    def raise_exception(x):
        raise Exception("Unexpected error")

    monkeypatch.setattr(init_pkg_logger, "setup_logger", raise_exception)
    monkeypatch.setattr(init_pkg_logger, "logger_main", "dummy_main")
    with pytest.raises(SystemExit):
        init_pkg_logger.init_pkg_logger()
