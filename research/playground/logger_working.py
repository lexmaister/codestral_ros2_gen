import codestral_ros2_gen.metrics.metrics_handler as mh
import codestral_ros2_gen.models.mistral_client as mc
from codestral_ros2_gen.utils.code_parser import ROS2CodeParser as parser
from codestral_ros2_gen import logger, get_config_path


test_metrics = {
    "generation_time": 1.23,
    "attempts_until_success": 2,
    "test_execution_time": 0.5,
    "response_size": 1024,
}


if __name__ == "__main__":
    logger.info("This is an info message")

    # Initialize the metrics handler
    metrics_handler = mh.MetricsHandler(
        metrics_file="tst.jsonl", config_file=get_config_path()
    )
    metrics_handler.add_metric(test_metrics)
    mh.logger.info("This is an info message from module")
    mh.logger.debug("This is a debug message from module")

    logger.info("This is an info message from root")
    logger.debug("This is a debug message from root")

    client = mc.MistralClient()
    try:
        resp, usage = client.complete(
            "Create a simple python function to calculate the factorial of a number"
        )
    except Exception as e:
        mc.logger.error(f"Error: {e}")

    code = parser.parse(resp)
    if code:
        logger.info(f"Parsed code:\n{code}")
    else:
        logger.error("None code in response")

    logger.info("App is closed")
