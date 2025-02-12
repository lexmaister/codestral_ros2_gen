import os
from mistralai import Mistral


def chat_with_file(file_path):
    api_key = os.environ.get("MISTRAL_API_KEY")
    if not api_key:
        raise ValueError("MISTRAL_API_KEY environment variable is not set")

    client = Mistral(api_key=api_key)

    # Read file content
    with open(file_path, "rb") as file:
        file_content = file.read().decode("utf-8")

    model = "codestral-latest"
    message = [
        {
            "role": "user",
            "content": f"Please analyze this file content:\n\n{file_content}",
        }
    ]
    response = client.chat.complete(model=model, messages=message)

    return response.choices[0].message.content


if __name__ == "__main__":
    file_path = input("Enter the path to the file: ")
    response = chat_with_file(file_path=file_path)
    print(response)
