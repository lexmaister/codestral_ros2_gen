import os
from mistralai import Mistral

api_key = os.environ["MISTRAL_API_KEY"]
client = Mistral(api_key=api_key)

model = "codestral-latest"
message = [{"role": "user", "content": "Write a function for fibonacci"}]
chat_response = client.chat.complete(model=model, messages=message)
print(chat_response.choices[0].message.content)
