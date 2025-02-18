import pytest
import pandas as pd
import numpy as np

from codestral_ros2_gen import get_config_path, load_config
from codestral_ros2_gen.metrics.metrics_handler import MetricsHandler


@pytest.fixture
def temp_metrics_file(tmp_path, project_config):
    """Fixture to create temporary metrics file using project config."""
    metrics_file = project_config["metrics"]["output_file"]
    return tmp_path / metrics_file


@pytest.fixture
def metrics_handler(temp_metrics_file, project_config) -> MetricsHandler:
    """Fixture to create MetricsHandler instance with project config."""
    handler = MetricsHandler(config=project_config, metrics_file=str(temp_metrics_file))
    return handler


@pytest.fixture
def sample_metrics(project_config):
    """Fixture with sample metrics based on project config."""
    metrics_to_collect = project_config["metrics"]["collect"]

    base_metrics = {
        "generation_time": 1.23,  # float
        "attempts_until_success": 2,  # int
        "test_execution_time": 0.5,  # float
        "response_size": 1024,  # int
        "token_usage": {  # dict
            "prompt_tokens": 100,
            "completion_tokens": 50,
            "total_tokens": 150,
        },
        "error_patterns": ["timeout"],  # list of strings
    }

    return {
        k: v
        for k, v in base_metrics.items()
        if k in metrics_to_collect and metrics_to_collect[k]
    }


def test_initialization(temp_metrics_file, project_config):
    """Test MetricsHandler initialization."""
    handler = MetricsHandler(config=project_config, metrics_file=str(temp_metrics_file))
    assert handler.metrics_file == temp_metrics_file
    assert isinstance(handler.metrics_df, pd.DataFrame)
    assert handler.metrics_df.empty


def test_add_metric(metrics_handler, sample_metrics):
    """Test adding metrics."""
    metrics_handler.add_metric(sample_metrics)

    assert not metrics_handler.metrics_df.empty
    assert "timestamp" in metrics_handler.metrics_df.columns
    assert len(metrics_handler.metrics_df) == 1


def test_add_invalid_metric(metrics_handler):
    """Test adding invalid metrics."""
    with pytest.raises(RuntimeError):
        metrics_handler.add_metric("invalid")


def test_save_and_load_metrics(metrics_handler, sample_metrics, project_config):
    """Test saving and loading metrics."""
    metrics_handler.add_metric(sample_metrics)

    # Create new handler to test loading
    new_handler = MetricsHandler(
        metrics_file=str(metrics_handler.metrics_file), config=project_config
    )
    new_handler.load_metrics()

    assert not new_handler.metrics_df.empty
    assert len(new_handler.metrics_df) == 1


def test_metrics_collection_config(metrics_handler, sample_metrics, project_config):
    """Test metrics collection according to config."""
    metrics_handler.add_metric(sample_metrics)

    metrics_to_collect = project_config["metrics"]["collect"]
    for metric, should_collect in metrics_to_collect.items():
        if should_collect and metric in sample_metrics:
            assert metric in metrics_handler.metrics_df.columns


def test_available_metrics(metrics_handler, sample_metrics, project_config):
    """Test available_metrics property."""
    metrics_handler.add_metric(sample_metrics)

    available = metrics_handler.available_metrics
    metrics_to_collect = project_config["metrics"]["collect"]

    # Define which metrics should be numeric
    numeric_metrics = {
        "generation_time",
        "attempts_until_success",
        "test_execution_time",
        "response_size",
    }

    for metric in metrics_to_collect:
        if (
            metrics_to_collect[metric]
            and metric in sample_metrics
            and metric in numeric_metrics
        ):  # Only check numeric metrics
            assert metric in available


@pytest.mark.parametrize("plot_type", ["hist", "box", "line"])
def test_plot_metrics(metrics_handler, sample_metrics, plot_type, tmp_path):
    """Test plotting with different plot types."""
    # Ensure we have numeric data
    test_metrics = {
        "generation_time": 1.23,
        "attempts_until_success": 2,
        "test_execution_time": 0.5,
        "response_size": 1024,
    }

    # Add multiple data points with some variation
    for i in range(5):
        metrics = test_metrics.copy()
        # Add some variation to the data
        metrics = {k: v * (1 + i * 0.1) for k, v in metrics.items()}
        metrics_handler.add_metric(metrics)

    # Debug output
    print(f"\nAvailable metrics: {metrics_handler.available_metrics}")
    print(f"DataFrame columns: {metrics_handler.metrics_df.columns}")
    print(f"DataFrame head:\n{metrics_handler.metrics_df.head()}")

    plot_path = metrics_handler.plot_metrics(plot_type=plot_type, output_dir=tmp_path)

    assert plot_path is not None
    assert plot_path.exists()
    assert plot_path.suffix == ".png"
    assert plot_path.parent == tmp_path


def test_plot_custom_name(metrics_handler, tmp_path):
    """Test plotting with custom name."""
    # Ensure we have numeric data
    test_metrics = {"generation_time": 1.23, "attempts_until_success": 2}

    # Add multiple data points
    for i in range(3):
        metrics = test_metrics.copy()
        metrics = {k: v * (1 + i * 0.1) for k, v in metrics.items()}
        metrics_handler.add_metric(metrics)

    custom_name = "custom_plot"

    plot_path = metrics_handler.plot_metrics(plot_name=custom_name, output_dir=tmp_path)

    assert plot_path is not None
    assert plot_path.exists()
    assert custom_name in plot_path.name


def test_plot_invalid_metrics(metrics_handler, tmp_path):
    """Test plotting with invalid metrics."""
    plot_path = metrics_handler.plot_metrics(
        metrics=["invalid_metric"], plot_type="hist", output_dir=tmp_path
    )
    assert plot_path is None


def test_plot_invalid_type(metrics_handler, sample_metrics, tmp_path):
    """Test plotting with invalid plot type."""
    metrics_handler.add_metric(sample_metrics)
    plot_path = metrics_handler.plot_metrics(
        metrics=["generation_time"], plot_type="invalid", output_dir=tmp_path
    )
    assert plot_path is None


def test_plot_empty_data(metrics_handler, tmp_path):
    """Test plotting with empty data."""
    plot_path = metrics_handler.plot_metrics(output_dir=tmp_path)
    assert plot_path is None


def test_generate_report(metrics_handler, sample_metrics):
    """Test report generation."""
    metrics_handler.add_metric(sample_metrics)

    report = metrics_handler.generate_report()
    assert isinstance(report, pd.DataFrame)
    assert not report.empty
    assert report.index.name.startswith("Report generated at:")


def test_empty_report(metrics_handler):
    """Test report generation with empty data."""
    report = metrics_handler.generate_report()
    assert isinstance(report, pd.DataFrame)
    assert report.empty


def test_multiple_metrics(metrics_handler, sample_metrics):
    """Test handling multiple metrics entries."""
    metrics_handler.add_metric(sample_metrics)
    metrics_handler.add_metric(sample_metrics)

    assert len(metrics_handler.metrics_df) == 2


def test_metrics_with_nan(metrics_handler, project_config):
    """Test handling NaN values in metrics."""
    metrics_to_collect = project_config["metrics"]["collect"]
    numeric_metric = next(
        k for k, v in metrics_to_collect.items() if v and k != "error_patterns"
    )

    metrics = {numeric_metric: np.nan, "attempts_until_success": 1}
    metrics_handler.add_metric(metrics)

    stats = metrics_handler.get_summary_stats()
    assert numeric_metric in stats.index
    assert pd.isna(stats.loc[numeric_metric, "mean"])
    assert stats.loc[numeric_metric, "missing"] == 1


def test_error_analysis(metrics_handler, sample_metrics, project_config):
    """Test error pattern analysis."""
    if not project_config["metrics"]["collect"].get("error_patterns"):
        pytest.skip("Error pattern collection not enabled in config")

    # Add multiple metrics with different error patterns
    sample_metrics["error_patterns"] = ["timeout"]
    metrics_handler.add_metric(sample_metrics)

    sample_metrics["error_patterns"] = ["timeout", "api_error"]
    metrics_handler.add_metric(sample_metrics)

    sample_metrics["error_patterns"] = ["api_error"]
    metrics_handler.add_metric(sample_metrics)

    error_counts = metrics_handler.analyze_errors()

    print(f"Error counts: {error_counts}")  # Debug output

    assert isinstance(error_counts, pd.Series)
    assert not error_counts.empty
    assert error_counts["timeout"] == 2
    assert error_counts["api_error"] == 2
