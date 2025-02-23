import os
from pathlib import Path
import stat

from codestral_ros2_gen.utils.file_io import save_code


def test_save_code_success(tmp_path):
    """
    Test that save_code successfully writes the file and sets executable permissions.
    """
    file_path = tmp_path / "subdir" / "output.py"
    code_content = "print('Hello world!')"
    result = save_code(code_content, file_path)
    assert result, "save_code should return True on successful save"
    # Verify file content.
    with open(file_path, "r") as f:
        assert f.read() == code_content
    # Verify file is executable.
    mode = os.stat(file_path).st_mode
    assert mode & stat.S_IXUSR, "File should be executable by the user"


def test_save_code_failure(monkeypatch, tmp_path):
    """
    Test that save_code returns False on failure.
    Simulate failure by monkeypatching Path.mkdir to raise an Exception.
    """
    file_path = tmp_path / "invalid" / "output.py"

    def fake_mkdir(*args, **kwargs):
        raise Exception("Simulated mkdir error")

    monkeypatch.setattr(Path, "mkdir", fake_mkdir)
    code_content = "print('Hello')"
    result = save_code(code_content, file_path)
    assert not result, "save_code should return False when saving fails"
