import os
from mistralai import Mistral


def chat_with_files(srv_file, test_file):
    api_key = os.environ.get("MISTRAL_API_KEY")
    if not api_key:
        raise ValueError("MISTRAL_API_KEY environment variable is not set")

    client = Mistral(api_key=api_key)

    # Read both files
    with open(srv_file, "rb") as file:
        srv_content = file.read().decode("utf-8")

    with open(test_file, "rb") as file:
        test_content = file.read().decode("utf-8")

    prompt = f"""I have a ROS2 service definition and a test file. Please generate a complete service node implementation that will pass these tests.

Service definition (ObjectHeight.srv):

{srv_content}

Test file (test_service_node.py):

{test_content}

Please generate a complete ROS2 service node implementation in Python that:
1. Implements the service defined in the .srv file
2. Will pass the provided test cases
3. Follows ROS2 best practices
4. Includes proper error handling
5. Has clear comments explaining the implementation
6. Uses proper ROS2 node structure

Please provide the complete code with explanations."""

    model = "codestral-latest"
    temperature = 0.2
    message = [
        {
            "role": "user",
            "content": prompt,
        }
    ]

    return client.chat.complete(model=model, messages=message, temperature=temperature)


if __name__ == "__main__":
    srv_file = input("Enter the path to the service definition file: ")
    test_file = input("Enter the path to the test file: ")
    response = chat_with_files(srv_file, test_file)
    print(
        response.choices[0].message.content,
        response.usage.total_tokens,
        response.usage.prompt_tokens,
        response.usage.completion_tokens,
        sep="\n========================\n",
    )
