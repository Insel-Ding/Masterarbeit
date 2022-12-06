import os
import openai
openai.organization = "org-MdrSm8iIoPx6bYxvYDxUP7Qy"
openai.api_key = os.getenv("OPENAI_API_KEY")
openai.Model.list()