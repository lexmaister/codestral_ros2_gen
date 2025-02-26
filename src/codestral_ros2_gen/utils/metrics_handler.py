import pandas as pd
from pathlib import Path
import logging
import json
from io import StringIO

from codestral_ros2_gen import logger_main
from codestral_ros2_gen.generators.generation_attempt import AttemptMetrics


logger = logging.getLogger(f"{logger_main}.{__name__.split('.')[-1]}")


class MetricsHandler:
    """
    Handler for collecting, recording, and analyzing code generation metrics.

    This class manages per-attempt metrics and overall aggregated metrics for
    the code generation process. It provides functionalities to record metrics,
    save and load metrics from a JSONL file, generate summary statistics,
    analyze error patterns, and produce plots.
    """

    def __init__(self, metrics_file: str | Path):
        """
        Initialize the MetricsHandler.

        Args:
        """
        self.metrics_file = Path(metrics_file).absolute().resolve()
        logger.info(f"Metrics file: {self.metrics_file}")
        self.metrics_df = pd.DataFrame()

        # Ensure the metrics directory exists.
        self.metrics_file.parent.mkdir(parents=True, exist_ok=True)

    def record_attempt(
        self,
        iteration_number: int,
        attempt_number: int,
        attempt_metrics: AttemptMetrics,
    ) -> None:
        """
        Record the metrics for a single generation attempt.

        Args:
            iteration_number (int): The sequential number of the iteration.
            attempt_number (int): The sequential number of the attempt.
            attempt_metrics: An AttemptMetrics instance containing metrics for the attempt.
        """
        logger.debug(
            f"Recording Attempt #{attempt_number}, Iteration #{iteration_number}:\n"
            + f"{json.dumps(attempt_metrics.as_dict, indent=2)}"
        )
        record = attempt_metrics.as_series
        record["iteration"] = iteration_number
        record["attempt"] = attempt_number
        self.metrics_df = pd.concat(
            [self.metrics_df, record.to_frame().T], ignore_index=True
        )
        self._save_metrics()

    def _save_metrics(self) -> None:
        """
        Save the current metrics DataFrame to the JSONL file.

        Raises:
            RuntimeError: If saving fails.
        """
        try:
            self.metrics_df.to_json(
                self.metrics_file, orient="records", lines=True, date_format="iso"
            )
            logger.debug(f"Metrics saved to {self.metrics_file}")
        except Exception as e:
            raise RuntimeError(f"Error saving metrics: {str(e)}")

    def load_metrics(self) -> None:
        """
        Load metrics from an existing JSONL file into the internal DataFrame.

        If the file is not found, an error
        """
        try:
            data = self.metrics_file.read_text(encoding="utf-8")
            self.metrics_df = pd.read_json(
                StringIO(data),
                orient="records",
                lines=True,
            )
            logger.info(f"Metrics loaded from {self.metrics_file}")
            logger.debug(f"Loaded metrics: {self.metrics_df}")
        except FileNotFoundError:
            raise RuntimeError(f"Metrics file {self.metrics_file} not found.")
        except Exception as e:
            raise RuntimeError(f"Error loading metrics: {str(e)}")

    def get_report(self) -> str:
        """
        Generate a report based on the collected metrics.
        Returns:
            str: The generated report.
        """
        if self.metrics_df.empty:
            return "No metrics collected."

        rep_1 = self.metrics_df.groupby("iteration")[
            ["success", "attempt", "tests_passed"]
        ].apply("max")
        rep_1["success"] = rep_1["success"].map({True: "✅", False: "🚫"})
        rep_1.columns = ("success", "attempts", "tests_passed")

        rep2 = (
            self.metrics_df.groupby("iteration")[["attempt_time", "total_tokens"]]
            .apply("mean")
            .astype(int)
        )
        rep2.columns = ("avg attempt time, s", "avg total tokens")

        return pd.concat([rep_1, rep2], axis=1).to_markdown(tablefmt="pretty")
