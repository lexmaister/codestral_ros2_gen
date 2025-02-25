import pandas as pd
import numpy as np
from typing import Dict, List, Optional, Literal
from pathlib import Path
import matplotlib.pyplot as plt
from datetime import datetime
import logging
import json

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

    PLOT_TYPES = Literal["hist", "box", "line"]

    def __init__(self, config: Dict, metrics_file: Optional[str] = None):
        """
        Initialize the MetricsHandler.

        Args:
            config (Dict): The configuration dictionary.
            metrics_file (Optional[str]): If provided, overrides the 'metrics_file' specified in the config.
                Otherwise, uses self.config["metrics"]["metrics_file"].
        """
        self.config = config
        self.metrics_file = (
            Path(metrics_file)
            if metrics_file
            else Path(self.config["metrics"]["metrics_file"])
        )
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

        If the file is not found, an empty DataFrame is created.
        """
        try:
            self.metrics_df = pd.read_json(
                self.metrics_file, orient="records", lines=True
            )
            logger.info(f"Metrics loaded from {self.metrics_file}")
            logger.debug(f"Loaded metrics: {self.metrics_df}")
        except FileNotFoundError:
            logger.warning(
                f"Metrics file {self.metrics_file} not found. Starting with an empty DataFrame."
            )
            self.metrics_df = pd.DataFrame()

    @property
    def available_metrics(self) -> List[str]:
        """
        Get a list of available numeric metric keys from the stored metrics.

        Returns:
            List[str]: List of column names corresponding to numeric metrics.
        """
        return list(self.metrics_df.select_dtypes(include=[np.number]).columns)

    def get_summary_stats(self) -> pd.DataFrame:
        """
        Calculate summary statistics (count, mean, median, std, min, max, missing)
        for all numeric metrics.

        Returns:
            pd.DataFrame: A DataFrame containing the summary statistics.
        """
        if self.metrics_df.empty:
            return pd.DataFrame()

        numeric_cols = self.available_metrics

        return pd.DataFrame(
            {
                "count": self.metrics_df[numeric_cols].count(),
                "mean": self.metrics_df[numeric_cols].mean(),
                "median": self.metrics_df[numeric_cols].median(),
                "std": self.metrics_df[numeric_cols].std(),
                "min": self.metrics_df[numeric_cols].min(),
                "max": self.metrics_df[numeric_cols].max(),
                "missing": self.metrics_df[numeric_cols].isna().sum(),
            }
        )

    def analyze_errors(self) -> pd.Series:
        """
        Analyze and count the frequency of error messages across all stored metrics.

        Returns:
            pd.Series: A Series with the counts of each error message.
        """
        if self.metrics_df.empty or "error_patterns" not in self.metrics_df.columns:
            return pd.Series(dtype=object)

        all_errors = []
        for errors in self.metrics_df["error_patterns"].dropna():
            if isinstance(errors, list):
                all_errors.extend(errors)
            elif isinstance(errors, str):
                all_errors.append(errors)

        error_counts = pd.Series(all_errors).value_counts()
        return error_counts

    def plot_metrics(
        self,
        metrics: Optional[List[str]] = None,
        plot_type: str = "hist",
        plot_name: str = "metrics_plot",
        output_dir: Optional[Path] = None,
    ) -> Optional[Path]:
        """
        Create a plot for the specified metrics using the chosen plot type.

        Args:
            metrics (Optional[List[str]]): A list of metric keys to plot.
                If None, all available numeric metrics (except timestamp) are plotted.
            plot_type (str): The type of plot to create ("hist", "box", or "line").
            plot_name (str): Base name for the output plot file.
            output_dir (Optional[Path]): Directory where the plot will be saved.
                Defaults to a 'plots' directory in the current working directory.

        Returns:
            Optional[Path]: The Path to the saved plot file, or None if plotting fails.
        """
        if self.metrics_df.empty:
            logger.error("No metrics data available for plotting")
            return None

        if plot_type not in ["hist", "box", "line"]:
            logger.error(f"Unsupported plot type: {plot_type}")
            return None

        numeric_cols = self.metrics_df.select_dtypes(include=[np.number]).columns
        available = [col for col in numeric_cols if col != "timestamp"]
        if not available:
            logger.error("No numeric metrics available for plotting")
            return None

        metrics_to_plot = metrics if metrics else available
        valid_metrics = [m for m in metrics_to_plot if m in available]
        if not valid_metrics:
            logger.error(f"No valid metrics found. Available metrics: {available}")
            return None

        try:
            n_metrics = len(valid_metrics)
            fig, axes = plt.subplots(n_metrics, 1, figsize=(10, 5 * n_metrics))
            axes = [axes] if n_metrics == 1 else axes.flatten()

            for ax, metric in zip(axes, valid_metrics):
                data = self.metrics_df[metric].dropna()
                if len(data) < 1:
                    logger.warning(f"No data available for metric: {metric}")
                    continue
                if plot_type == "hist":
                    data.hist(ax=ax, bins="auto")
                    ax.set_ylabel("Frequency")
                elif plot_type == "box":
                    data.plot(kind="box", ax=ax)
                elif plot_type == "line":
                    data.plot(kind="line", ax=ax)
                ax.set_title(f"{metric}")
                ax.set_xlabel(metric)
                ax.grid(True)

            plt.tight_layout()
            if output_dir is None:
                output_dir = Path.cwd() / "plots"
            output_dir = Path(output_dir)
            output_dir.mkdir(exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plot_file = output_dir / f"{plot_name}_{timestamp}.png"
            fig.savefig(plot_file, dpi=300, bbox_inches="tight")
            plt.close(fig)
            logger.info(f"Plot saved as: {plot_file}")
            return plot_file

        except Exception as e:
            logger.error(f"Error creating plot: {str(e)}")
            plt.close()
            return None

    def generate_report(self) -> pd.DataFrame:
        """
        Generate a summary report as a DataFrame with statistics and error analysis.

        Returns:
            pd.DataFrame: A DataFrame containing summary statistics and error count details.
        """
        if self.metrics_df.empty:
            logger.error("No metrics data available")
            return pd.DataFrame()

        stats_df = self.get_summary_stats()
        error_counts = self.analyze_errors()
        if not error_counts.empty:
            stats_df.loc["error_count"] = [
                error_counts.sum() if i == "count" else np.nan for i in stats_df.columns
            ]
        stats_df.index.name = f"Report generated at: {datetime.now().isoformat()}"
        return stats_df
