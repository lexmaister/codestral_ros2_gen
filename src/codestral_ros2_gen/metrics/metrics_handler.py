import pandas as pd
import numpy as np
from typing import Dict, List, Optional, Literal, Any
from pathlib import Path
import matplotlib.pyplot as plt
from datetime import datetime
import yaml
import logging

from codestral_ros2_gen import logger_main

logger = logging.getLogger(f"{logger_main}.{__name__.split('.')[-1]}")


class MetricsHandler:
    """Handler for collecting and analyzing code generation metrics."""

    PLOT_TYPES = Literal["hist", "box", "line"]

    def __init__(self, config: Dict, metrics_file: Optional[str] = None):
        """
        Initialize metrics handler.

        Args:
            config: Configuration dictionary
            metrics_file: Path to metrics file (optional)
        """
        self.config = config
        self.metrics_file = (
            Path(metrics_file)
            if metrics_file
            else Path(self.config["metrics"]["output_file"])
        )
        self.metrics_df = pd.DataFrame()

        # Load existing metrics if file exists
        if self.metrics_file.exists():
            self.load_metrics()

        # Ensure metrics directory exists
        self.metrics_file.parent.mkdir(parents=True, exist_ok=True)

    def _load_config(self, config_file: str) -> Dict:
        """
        Load configuration from yaml file.

        Args:
            config_file: Path to config file

        Returns:
            Dict: Configuration dictionary

        Raises:
            RuntimeError: If config file cannot be loaded
        """
        try:
            with open(config_file, "r") as f:
                return yaml.safe_load(f)
        except Exception as e:
            raise RuntimeError(f"Error loading config: {str(e)}")

    def format_attempt_metrics(self, metrics: Dict[str, Any]) -> str:
        """Format per-attempt metrics for logging."""
        sections = [
            f"Time: {metrics['time']:.2f}s",
            f"Success: {metrics['success']}",
            f"Stages: {', '.join(metrics['stage_results'])}",
        ]
        if metrics["token_usage"]:
            sections.extend(
                [
                    "Token Usage:",
                    f"  Prompt: {metrics['token_usage'].get('prompt_tokens', 0)}",
                    f"  Completion: {metrics['token_usage'].get('completion_tokens', 0)}",
                    f"  Total: {metrics['token_usage'].get('total_tokens', 0)}",
                ]
            )
        if metrics["errors"]:
            sections.extend(["Errors:", *[f"  - {err}" for err in metrics["errors"]]])
        return "\n".join(sections)

    def format_generation_summary(self, metrics: Dict) -> str:
        """Generate metrics summary in a detailed, original style."""

        def format_number(n: float) -> str:
            return f"{n:.2f}" if isinstance(n, float) else str(n)

        def safe_get(d: Dict, *keys: str, default: None):
            try:
                result = d
                for key in keys:
                    result = result[key]
                return result
            except (KeyError, TypeError):
                return default

        sections = []
        sections.append("=== Generation Summary ===")
        sections.append(
            f"Total Time: {format_number(safe_get(metrics, 'main_timer', default=0.0))}s"
        )
        attempts = safe_get(metrics, "attempts", default=0)
        max_attempts = safe_get(metrics, "config_used", "max_attempts", default="?")
        sections.append(f"Attempts: {attempts}/{max_attempts}")
        timeouts = safe_get(metrics, "timeouts", "attempts", default=0)
        sections.append(f"Timeouts: {timeouts}")

        # Show aggregated errors via error_summary
        error_summary = safe_get(metrics, "error_summary", default="")
        if error_summary:
            sections.append("\n=== Aggregated Errors ===")
            sections.append(error_summary)

        attempt_timers = safe_get(metrics, "attempt_timers", default=[])
        if attempt_timers:
            sections.append("\n=== Timing Details ===")
            avg_attempt = sum(attempt_timers) / len(attempt_timers)
            sections.append(f"Average per attempt: {format_number(avg_attempt)}s")
            per_attempt_timeout = safe_get(
                metrics, "config_used", "per_attempt_timeout", default=0.0
            )
            sections.append(
                f"Per-attempt timeout: {format_number(per_attempt_timeout)}s"
            )

        token_usage = safe_get(metrics, "token_usage", default={})
        if token_usage and any(token_usage.values()):
            sections.append("\n=== Token Usage ===")
            for key, value in token_usage.items():
                sections.append(f"{key}: {value}")

        return "\n".join(sections)

    def add_metric(self, metrics: Dict) -> None:
        """Add new metrics and save to file."""
        if not isinstance(metrics, dict):
            raise RuntimeError("Metrics must be a dictionary")

        # Compute error_summary from cumulative error_counts
        error_counts = metrics.get("error_counts", {})
        metrics["error_summary"] = ", ".join(
            [f"{err}: {count}" for err, count in error_counts.items()]
        )

        # Filter and prepare metrics
        metrics_to_collect = self.config["metrics"].get("collect", {})
        filtered_metrics = {
            k: v for k, v in metrics.items() if metrics_to_collect.get(k, True)
        }
        filtered_metrics["timestamp"] = datetime.now().isoformat()

        logger.info(
            f"\nMetrics report:\n\n{self.format_generation_summary(filtered_metrics)}\n"
        )

        new_row = pd.DataFrame([filtered_metrics])
        for col in new_row.columns:
            if isinstance(new_row[col].iloc[0], (int, float)):
                new_row[col] = pd.to_numeric(new_row[col])
        self.metrics_df = pd.concat([self.metrics_df, new_row], ignore_index=True)
        self._save_metrics()

    def _save_metrics(self) -> None:
        """Save metrics to JSONL file."""
        try:
            self.metrics_df.to_json(
                self.metrics_file, orient="records", lines=True, date_format="iso"
            )
            logger.debug(f"Metrics saved to {self.metrics_file}")
        except Exception as e:
            raise RuntimeError(f"Error saving metrics: {str(e)}")

    def load_metrics(self) -> None:
        """Load metrics from existing JSONL file."""
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
        """Get list of available numeric metrics."""
        return list(self.metrics_df.select_dtypes(include=[np.number]).columns)

    def get_summary_stats(self) -> pd.DataFrame:
        """
        Calculate summary statistics for numeric metrics.

        Returns:
            pd.DataFrame: Summary statistics
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
        """Analyze error patterns from collected metrics."""
        if self.metrics_df.empty or "error_patterns" not in self.metrics_df.columns:
            return pd.Series(dtype=object)

        # Flatten list of error patterns into single list
        all_errors = []
        for errors in self.metrics_df["error_patterns"].dropna():
            if isinstance(errors, list):
                all_errors.extend(errors)
            elif isinstance(errors, str):
                all_errors.append(errors)

        # Count occurrences of each error
        error_counts = pd.Series(all_errors).value_counts()
        return error_counts

    def plot_metrics(
        self,
        metrics: Optional[List[str]] = None,
        plot_type: str = "hist",
        plot_name: str = "metrics_plot",
        output_dir: Optional[Path] = None,
    ) -> Optional[Path]:
        """Create plot for specified metrics."""
        if self.metrics_df.empty:
            logger.error("No metrics data available for plotting")
            return None

        if plot_type not in ["hist", "box", "line"]:
            logger.error(f"Unsupported plot type: {plot_type}")
            return None

        # Get numeric columns
        numeric_cols = self.metrics_df.select_dtypes(include=[np.number]).columns
        available = [col for col in numeric_cols if col != "timestamp"]

        if not available:
            logger.error("No numeric metrics available for plotting")
            return None

        # Use provided metrics or all available numeric metrics
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

            # Use provided output directory or create 'plots' in it
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
        Generate report as pandas DataFrame with statistics and error analysis.

        Returns:
            pd.DataFrame: Report containing statistics for all metrics
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
