import logging
import requests, time
from dataclasses import dataclass, asdict
import json, os, requests

from openai import OpenAI

logger = logging.getLogger('simulation_core')
#logger.setLevel(logging.DEBUG)

api_key = None
try:
    api_key = os.getenv("OPENROUTER_API_KEY")
except Exception as e:
    print(f"Error getting OpenRouter API key: {e}")

MODEL = ''
url = "https://openrouter.ai/api/v1/chat/completions"
headers = {
    "Authorization": f"Bearer {api_key}",
    "Content-Type": "application/json"
}


class OpenRouterClient():
    DefaultEndpoint = url
    def __init__(self, api_key=None):
        self.api_key = api_key
        self.client = OpenAI(
            base_url=url,
            api_key=api_key,
        )
        
    def executeRequest(self, prompt, temperature=0.4, top_p=1.0, max_tokens=400, stops=[], model=None):
        startTime = time.time()
        if model is None:
            model = 'meta-llama/llama-4-maverick'
        max_retries = 3
        for attempt in range(max_retries):
            try:
                reasoning_buffer = 500
                payload = {
                    "model": model,
                    "messages": prompt,
                    "reasoning": {"effort": "low"},
                    "max_tokens": max_tokens + reasoning_buffer,
                    "temperature": temperature,
                    "top_p": top_p,
                    "stop": stops,
                    "stream": False,
                }

                response = requests.post(url, headers=headers, data=json.dumps(payload))
                if response.status_code == 200:
                    return response

            except Exception as e:
                print(f"OpenRouter request error (attempt {attempt + 1}/{max_retries}): {e}")
                if attempt == max_retries - 1:
                    return {"status": "error", "message": "retries exceeded"}
                # Exponential backoff: 1s, 2s, 4s
                time.sleep(2 ** attempt)

        # If loop exits without return, treat as error
        return {"status": "error", "message": "Request failed after all retries"}
            

