import os
from typing import Dict, Any, Tuple, Optional, List
from dataclasses import dataclass
from mistralai import Mistral
import logging

from codestral_ros2_gen import logger_main


logger = logging.getLogger(f"{logger_main}.{__name__.split('.')[-1]}")


@dataclass
class ModelUsage:
    """
    Dataclass to represent token usage statistics.

    Attributes:
        prompt_tokens (int): Number of tokens used in the prompt.
        completion_tokens (int): Number of tokens used in the completion.
        total_tokens (int): Total number of tokens used.
    """

    prompt_tokens: int
    completion_tokens: int
    total_tokens: int

    def __str__(self):
        return (
            f"Model usage (tokens):\n"
            f"prompt:\t\t{self.prompt_tokens}\n"
            f"completion:\t{self.completion_tokens}\n"
            f"total:\t\t{self.total_tokens}"
        )


class MistralClient:
    """
    Client for interacting with Mistral AI API.

    Attributes:
        DEFAULT_CONFIG (Dict[str, Any]): Default configuration for the Mistral client.
        api_key (str): API key for authenticating with the Mistral AI API.
        client (Mistral): Mistral client instance.
        config (Dict[str, Any]): Configuration for the Mistral client.
    """

    DEFAULT_CONFIG = {
        "model": {"type": "codestral-latest", "parameters": {"temperature": 0.2}}
    }

    def __init__(
        self, api_key: Optional[str] = None, config: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize the MistralClient.

        Args:
            api_key (Optional[str]): API key for authenticating with the Mistral AI API.
            config (Optional[Dict[str, Any]]): Configuration for the Mistral client.
        """
        model_config = config.get("model", {}) if config else {}
        self.api_key = self._get_api_key(api_key, model_config)
        self.client = Mistral(api_key=self.api_key)
        self.config = model_config

    def _get_api_key(self, api_key: Optional[str], config: Dict[str, Any]) -> str:
        """
        Get API key from provided sources in order of precedence.

        Args:
            api_key (Optional[str]): API key provided directly.
            config (Dict[str, Any]): Configuration dictionary.

        Returns:
            str: API key.

        Raises:
            RuntimeError: If API key is not found in any of the provided sources.
        """
        msg = "Create Mistral client with provided API key"
        if api_key:
            logger.info(msg)
            return api_key

        env_key = os.getenv("MISTRAL_API_KEY")
        if env_key:
            logger.info(f"{msg} from the environment variable")
            return env_key

        config_key = config.get("api_key")
        if config_key and config_key != "YOUR_API_KEY_HERE":
            logger.info(f"{msg} from config")
            return config_key

        raise RuntimeError(
            "Mistral API key not found. Please provide it through one of:\n"
            "1. Direct api_key parameter in MistralClient initialization\n"
            "2. MISTRAL_API_KEY environment variable\n"
            "3. 'api_key' field in the model config\n"
            "Current config template value 'YOUR_API_KEY_HERE' is not valid."
        )

    def complete(
        self,
        prompt: str,
        system_prompt: Optional[str] = None,
        model_type: Optional[str] = None,
        temperature: Optional[float] = None,
    ) -> Tuple[str, ModelUsage]:
        """
        Get completion from the model.

        Args:
            prompt (str): Main prompt text.
            system_prompt (Optional[str]): Optional override for system prompt.
            model_type (Optional[str]): Optional override for model type.
            temperature (Optional[float]): Optional override for temperature.

        Returns:
            Tuple[str, ModelUsage]: Tuple of (generated_text, usage_stats).

        Raises:
            ValueError: If the prompt is empty.
            ConnectionError: If there is a connection error with the Mistral API.
            RuntimeError: If there is an API error.
        """
        logger.info("Start generating completion from Mistral AI")
        logger.debug(f"Prompt:\n{'<'*3}\n{prompt.strip()}\n{'<'*3}")
        if not prompt or not prompt.strip():
            raise ValueError("Empty prompt provided")

        try:
            messages = self._prepare_messages(
                prompt, system_prompt or self.config.get("system_prompt")
            )
            logger.debug(f"Prepared messages:\n{messages}\n")

            response = self.client.chat.complete(
                model=model_type
                or self.config.get("type", self.DEFAULT_CONFIG["model"]["type"]),
                messages=messages,
                temperature=temperature
                or self.config.get("parameters", {}).get(
                    "temperature",
                    self.DEFAULT_CONFIG["model"]["parameters"]["temperature"],
                ),
            )

            if not hasattr(response, "choices") or not response.choices:
                raise RuntimeError("Invalid response format from API")

            completion = response.choices[0].message.content
            if not completion:
                raise RuntimeError("Empty completion response from API")

            logger.info("Completion generated successfully")
            logger.debug(f"Response:\n{'>'*3}\n{completion}\n{'>'*3}")

            # Update usage statistics
            prompt_tokens = max(0, getattr(response.usage, "prompt_tokens", 0))
            completion_tokens = max(0, getattr(response.usage, "completion_tokens", 0))
            total_tokens = prompt_tokens + completion_tokens

            usage = ModelUsage(
                prompt_tokens=prompt_tokens,
                completion_tokens=completion_tokens,
                total_tokens=total_tokens,
            )
            logger.debug(str(usage))

            return completion, usage

        except ConnectionError as e:
            raise ConnectionError(f"Failed to connect to Mistral API: {str(e)}")
        except Exception as e:
            error_msg = str(e)
            raise RuntimeError(f"API error: {error_msg}")

    def _prepare_messages(
        self, prompt: str, system_prompt: Optional[str] = None
    ) -> List[Dict[str, str]]:
        """
        Prepare messages list for the API call.

        Args:
            prompt (str): Main prompt text.
            system_prompt (Optional[str]): Optional system prompt.

        Returns:
            List[Dict[str, str]]: List of messages for the API call.
        """
        messages = []

        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})

        messages.append({"role": "user", "content": prompt})

        return messages
