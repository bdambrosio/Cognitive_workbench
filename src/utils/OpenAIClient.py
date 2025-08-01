import requests, time, copy
from typing import Optional, Dict, Any, Union
from dataclasses import dataclass, asdict
import json, os,sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Messages import SystemMessage

from openai import OpenAI

client = None

openai_api_key = None

try:
    openai_api_key = os.getenv("OPENAI_API_KEY")
except Exception as e:
    print(f"Error getting OpenAI API key: {e}")
    openai_api_key = None

class OpenAIClient:
    def __init__(self, model_name = 'gpt-4.1-mini', api_key=None):
        global openai_api_key
        self.DefaultEndpoint = 'https://api.openai.com'
        self.openai_api_base = self.DefaultEndpoint
        self.UserAgent = 'Owl'
        self.model_name = model_name
        if self.model_name.startswith('Qwen3'):
            self.openai_api_key = "EMPTY"
            self.openai_api_base = "http://localhost:5000/v1"

            self.client = OpenAI(
                api_key=api_key,
                base_url=self.openai_api_base,
                timeout=300.0,  # 60 second timeout
                max_retries=2   # Retry up to 3 times
            )
        else:
            self.client = OpenAI(
                api_key=openai_api_key,
                timeout=45.0,  # 60 second timeout
                max_retries=2   # Retry up to 3 times
            )

        
    def executeRequest(self, prompt, temperature=0.4, top_p=1.0, max_tokens=400, stops=[], model=None):
        startTime = time.time()

        if self.model_name.startswith('Qwen3'):
            try:
                response = self.client.chat.completions.create(
                    model='Qwen/Qwen3-14B',
                    messages=prompt,
                    max_tokens=max_tokens,
                    temperature=temperature,
                    top_p=top_p,
                    stop=stops,
                    stream=False,
                    timeout=600.0,
                    extra_body={"top_k": 20, "chat_template_kwargs": {"enable_thinking": False}}
                )
                item = response.choices[0].message
                return item.content
            except Exception as e:
                print(f"VLLM {model} request error: {e}")
                return {"status":'error', "message":str(e)} 
        else:
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    response = self.client.chat.completions.create(
                            model=self.model_name, 
                            messages=prompt,
                            max_tokens=max_tokens, 
                            temperature=temperature, 
                            top_p=top_p,
                            stop=stops, 
                            stream=False,
                            timeout=30.0
                        )
                    item = response.choices[0].message
                    return item.content
                except Exception as e:
                    print(f"OpenAI request error (attempt {attempt + 1}/{max_retries}): {e}")
            if attempt == max_retries - 1:
                return {"status":'error', "message":"retries exceeded"}
            # Exponential backoff: 1s, 2s, 4s
            time.sleep(2 ** attempt)
        
        return {"status":'error', "message":"Request failed after all retries"}
            

