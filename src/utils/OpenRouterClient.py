import requests, time
from dataclasses import dataclass, asdict
import json, os

from openai import OpenAI


api_key = None
try:
    api_key = os.getenv("OPENROUTER_API_KEY")
except Exception as e:
    print(f"Error getting OpenRouter API key: {e}")
#MODEL = 'google/gemini-2.0-flash-001'
#MODEL = 'meta-llama/llama-4-maverick'
MODEL = 'google/gemma-3-27b-it'


class OpenRouterClient():
    DefaultEndpoint = 'https://openrouter.ai'
    UserAgent = 'Owl'
    def __init__(self, api_key=None):
        self.api_key = api_key
        
    def executeRequest(self, prompt, temperature=0.4, top_p=1.0, max_tokens=400, stops=[], model=None):
        startTime = time.time()
        if model is None:
            model = 'meta-llama/llama-4-maverick'
        max_retries = 3
        for attempt in range(max_retries):
            try:
                response = requests.post(
                    url="https://openrouter.ai/api/v1/chat/completions",
                    headers={
                        "Authorization": f"Bearer {self.api_key}",
                        "HTTP-Referer": "https://tuuyi.com",  # Optional site URL for rankings
                        "X-Title": "Tuuyi",  # Optional site title
                    },
                    data=json.dumps({
                        "model": MODEL,
                        "messages": prompt,
                        "max_tokens": max_tokens,
                        "temperature": temperature,
                        "top_p": top_p,
                        "stop": stops,
                        "stream": False,
                    }),
                    timeout=30.0,
                )

                response.raise_for_status()
                item = response.content
                item = json.loads(item.decode("utf-8"))
                item = item["choices"][0]["message"]["content"]
                return item

            except Exception as e:
                print(f"OpenRouter request error (attempt {attempt + 1}/{max_retries}): {e}")
                if attempt == max_retries - 1:
                    return {"status": "error", "message": "retries exceeded"}
                # Exponential backoff: 1s, 2s, 4s
                time.sleep(2 ** attempt)

        # If loop exits without return, treat as error
        return {"status": "error", "message": "Request failed after all retries"}
            

