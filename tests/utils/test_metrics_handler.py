import pytest
from unittest.mock import patch
import pandas as pd
from codestral_ros2_gen.utils.metrics_handler import MetricsHandler
from codestral_ros2_gen.generators.generation_attempt import AttemptMetrics


def test_record_attempt(sample_metrics, tmp_path):
    metrics_file = tmp_path / "test_metrics.jsonl"
    handler = MetricsHandler(metrics_file)
    handler.record_attempt(1, 1, sample_metrics)

    handler.load_metrics()
    assert not handler.metrics_df.empty
    assert handler.metrics_df.iloc[0]["attempt_time"] == 1.0
    assert handler.metrics_df.iloc[0]["success"]
    assert handler.metrics_df.iloc[0]["tests_passed"] == 10


def test_save_and_load_metrics(sample_metrics, tmp_path):
    metrics_file = tmp_path / "test_metrics.jsonl"
    handler = MetricsHandler(metrics_file)
    handler.record_attempt(1, 1, sample_metrics)
    handler._save_metrics()

    handler2 = MetricsHandler(metrics_file)
    handler2.load_metrics()
    assert not handler2.metrics_df.empty
    assert handler2.metrics_df.iloc[0]["attempt_time"] == 1.0
    assert handler2.metrics_df.iloc[0]["success"]
    assert handler2.metrics_df.iloc[0]["tests_passed"] == 10


def test_get_report(sample_metrics, tmp_path):
    metrics_file = tmp_path / "test_metrics.jsonl"
    handler = MetricsHandler(metrics_file)
    handler.record_attempt(1, 1, sample_metrics)
    handler.record_attempt(1, 2, sample_metrics)

    report = handler.get_report()
    assert "iteration" in report
    assert "success" in report
    assert "attempts" in report
    assert "tests_passed" in report
    assert "avg attempt time, s" in report
    assert "avg total tokens" in report


def test_load_nonexistent_metrics(tmp_path):
    metrics_file = tmp_path / "nonexistent_metrics.jsonl"
    handler = MetricsHandler(metrics_file)

    with pytest.raises(RuntimeError, match="Metrics file .* not found."):
        handler.load_metrics()


def test_record_multiple_attempts(sample_metrics, tmp_path):
    metrics_file = tmp_path / "test_metrics.jsonl"
    handler = MetricsHandler(metrics_file)
    handler.record_attempt(1, 1, sample_metrics)
    handler.record_attempt(1, 2, sample_metrics)
    handler.record_attempt(2, 1, sample_metrics)

    handler.load_metrics()
    assert len(handler.metrics_df) == 3
    assert handler.metrics_df.iloc[0]["iteration"] == 1
    assert handler.metrics_df.iloc[0]["attempt"] == 1
    assert handler.metrics_df.iloc[1]["iteration"] == 1
    assert handler.metrics_df.iloc[1]["attempt"] == 2
    assert handler.metrics_df.iloc[2]["iteration"] == 2
    assert handler.metrics_df.iloc[2]["attempt"] == 1


def test_empty_report(tmp_path):
    metrics_file = tmp_path / "test_metrics.jsonl"
    handler = MetricsHandler(metrics_file)

    report = handler.get_report()
    assert report == "No metrics collected."


@patch("pandas.DataFrame.to_json", side_effect=RuntimeError("Simulated error"))
def test_save_metrics_error(mock_to_json, tmp_path):
    metrics_file = tmp_path / "test_metrics.jsonl"
    handler = MetricsHandler(metrics_file)
    handler.metrics_df = pd.DataFrame({"invalid_column": [1, 2, 3]})

    with pytest.raises(RuntimeError, match="Error saving metrics: Simulated error"):
        handler._save_metrics()
