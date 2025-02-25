import pytest
import pandas as pd
import numpy as np
from pathlib import Path

from codestral_ros2_gen import get_config_path, load_config
from codestral_ros2_gen.utils.metrics_handler import MetricsHandler
from codestral_ros2_gen.generators.generation_attempt import AttemptMetrics


# Use a simple project_config fixture if "collect" is not used in new version.
@pytest.fixture
def project_config():
    # Provide minimal config for tests; metrics key must have an output_file entry.
    return {"metrics": {"output_file": "temp_metrics.jsonl"}}


@pytest.fixture
def temp_metrics_file(tmp_path, project_config):
    """Fixture to create temporary metrics file using project config."""
    return tmp_path / project_config["metrics"]["output_file"]


@pytest.fixture
def metrics_handler(temp_metrics_file, project_config) -> MetricsHandler:
    """Fixture to create MetricsHandler instance with project config."""
    handler = MetricsHandler(config=project_config, metrics_file=str(temp_metrics_file))
    return handler


def test_initialization(temp_metrics_file, project_config):
    """Test MetricsHandler initialization."""
    handler = MetricsHandler(config=project_config, metrics_file=str(temp_metrics_file))
    assert handler.metrics_file == temp_metrics_file
    assert isinstance(handler.metrics_df, pd.DataFrame)
    assert handler.metrics_df.empty


def test_add_metric(metrics_handler):
    """Test add_metric method."""
    sample_metrics = {"generation_time": 1.23, "attempts_until_success": 2}
    metrics_handler.add_metric(sample_metrics)
    assert not metrics_handler.metrics_df.empty
    assert "timestamp" in metrics_handler.metrics_df.columns
    assert len(metrics_handler.metrics_df) == 1


def test_add_invalid_metric(metrics_handler):
    """Test adding invalid metrics."""
    with pytest.raises(RuntimeError):
        metrics_handler.add_metric("invalid")


def test_save_and_load_metrics(metrics_handler, project_config):
    """Test saving and loading metrics."""
    sample_metrics = {"generation_time": 1.23, "attempts_until_success": 2}
    metrics_handler.add_metric(sample_metrics)
    new_handler = MetricsHandler(
        metrics_file=str(metrics_handler.metrics_file), config=project_config
    )
    new_handler.load_metrics()
    assert not new_handler.metrics_df.empty
    assert len(new_handler.metrics_df) == 1


def test_record_attempt(metrics_handler):
    """Test record_attempt function using a dummy AttemptMetrics."""
    # Create a dummy AttemptMetrics instance.
    dummy = AttemptMetrics(attempt_time=1.5, final_state="SUCCESS", error=None)
    metrics_handler.record_attempt(1, dummy)
    # Check that a record for attempt_number=1 exists.
    df = metrics_handler.metrics_df
    assert "attempt_number" in df.columns
    # There should be an entry with attempt_number == 1.
    assert any(df["attempt_number"] == 1)


def test_record_overall(metrics_handler):
    """Test record_overall function records overall metrics."""
    overall = {"total_time": 10.0, "final_result": "SUCCESS"}
    metrics_handler.record_overall(overall)
    df = metrics_handler.metrics_df
    # Check that overall metrics have a timestamp.
    assert "timestamp" in df.columns
    # Check overall record contains final_result.
    assert any(df["final_result"] == "SUCCESS")


def test_get_summary_stats(metrics_handler):
    """Test summary statistics is generated."""
    sample = {"generation_time": 1.23, "attempts_until_success": 2}
    metrics_handler.add_metric(sample)
    stats = metrics_handler.get_summary_stats()
    assert isinstance(stats, pd.DataFrame)
    assert not stats.empty


def test_analyze_errors(metrics_handler):
    """Test analyze_errors returns a pandas Series."""
    metrics_handler.add_metric({"error_patterns": ["timeout", "api_error"]})
    metrics_handler.add_metric({"error_patterns": ["timeout"]})
    errors = metrics_handler.analyze_errors()
    assert isinstance(errors, pd.Series)
    assert not errors.empty
    # Expect "timeout" to have count 2 and "api_error" count 1.
    assert errors.get("timeout", 0) == 2
    assert errors.get("api_error", 0) == 1


@pytest.mark.parametrize("plot_type", ["hist", "box", "line"])
def test_plot_metrics(metrics_handler, plot_type, tmp_path):
    """Test plotting with different plot types."""
    # Ensure numeric data exists.
    for i in range(3):
        metrics_handler.add_metric({"generation_time": 1.0 + i * 0.1})
    plot_path = metrics_handler.plot_metrics(plot_type=plot_type, output_dir=tmp_path)
    assert plot_path is not None
    assert plot_path.exists()
    assert plot_path.suffix == ".png"


def test_plot_invalid_metrics(metrics_handler, tmp_path):
    """Test plotting with an invalid metric key."""
    plot_path = metrics_handler.plot_metrics(
        metrics=["nonexistent"], plot_type="hist", output_dir=tmp_path
    )
    assert plot_path is None


def test_generate_report(metrics_handler):
    """Test generate_report produces non-empty DataFrame when metrics exist."""
    metrics_handler.add_metric({"generation_time": 1.23, "attempts_until_success": 2})
    report = metrics_handler.generate_report()
    assert isinstance(report, pd.DataFrame)
    assert not report.empty


def test_empty_report(metrics_handler):
    """Test generate_report returns empty DataFrame when no metrics exist."""
    report = metrics_handler.generate_report()
    assert isinstance(report, pd.DataFrame)
    assert report.empty


def test_multiple_metrics(metrics_handler):
    """Test handling multiple metrics entries."""
    metrics_handler.add_metric({"generation_time": 1.23})
    metrics_handler.add_metric({"generation_time": 1.45})
    assert len(metrics_handler.metrics_df) == 2


def test_metrics_with_nan(metrics_handler):
    """Test handling NaN values in metrics."""
    metrics_handler.add_metric({"generation_time": np.nan, "attempts_until_success": 1})
    stats = metrics_handler.get_summary_stats()
    assert "generation_time" in stats.index
    assert pd.isna(stats.loc["generation_time", "mean"])
    assert stats.loc["generation_time", "missing"] == 1
