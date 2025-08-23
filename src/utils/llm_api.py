#from matplotlib.hatch import Stars
import os, sys, re, traceback, requests, json
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import random
import socket
import time
import logging
from pathlib import Path
from sympy import Ordinal
from Messages import SystemMessage, UserMessage, AssistantMessage
from utils.OpenAIClient import OpenAIClient
import utils.OpenRouterClient as OpenRouterClient

MODEL = '' # set by simulation from config.py

# Create logger that inherits from parent app's logging configuration
logger = logging.getLogger('llm_api')

api_key = None
try:
    api_key = os.getenv("OPENAI_API_KEY")
except Exception as e:
    pass


openrouter_client = None
try:
    openrouter_client = OpenRouterClient.OpenRouterClient(api_key=os.getenv("OPENROUTER_API_KEY"))
except Exception as e:
    pass

response_prime_needed = False
headers = {}
IMAGE_PATH = Path.home() / '.local/share/AllTheWorld/images'
IMAGE_PATH.mkdir(parents=True, exist_ok=True)
vllm_model = '/home/bruce/Downloads/models/Qwen2.5-32B-Instruct'

elapsed_times = {}
iteration_count = 0

pattern = r'\{\$[^}]*\}'

# options include 'local', 'Claude', 'OpenAI', 'deepseek-chat',
class LLM():

    def __init__(self, server_name='local', model_name=None):
        global vllm_model
        self.server_name = server_name
        print(f'will use {self.server_name} as llm')
        self.context_size = 16384  # conservative local mis/mixtral default
        if model_name is not None:
            self.model = model_name
        else:
            self.model = MODEL
        if self.server_name == 'openai':
            self.openai_client = OpenAIClient(model_name=self.model)
        if self.server_name == 'vllm':
            try:
                response = requests.get('http://localhost:5000/v1/models', headers=headers)
                response.raise_for_status()  # Raises an HTTPError for bad responses (4xx, 5xx)
                data = response.json()
                if data.get('data') and len(data['data']) > 0:
                    self.model = data['data'][0]['id']
                else:
                    raise ValueError("No models found in the response")
            except (requests.RequestException, KeyError, ValueError) as e:
                # Handle the error appropriately
                raise Exception(f"Failed to get model information: {str(e)}")

        if not IMAGE_PATH.exists():
            IMAGE_PATH.mkdir(parents=True, exist_ok=True)
            print(f"Directory '{IMAGE_PATH}' created.")

    def substitute_bindings(self, prompt, bindings):
        if bindings is None or len(bindings) == 0:
            substituted_prompt = [{'role':message.role, 'content':message.content} for message in prompt]
        else:
            substituted_prompt = []
            for message in prompt:
                if type(message.content) is not str: # substitutions only in strings
                    substituted_prompt.append({'role':message.role, 'content':message.content})
                    continue
                matches = re.findall(pattern, message.content)
                new_content = message.content
                for match in matches:
                    var = match[2:-1]
                    if var in bindings.keys():
                        new_content = new_content.replace('{{$'+var+'}}', str(bindings[var]))
                    else:
                        print(f' var not in bindings {var} {bindings}')
                        raise ValueError(f'unbound prompt variable {var}')
                substituted_prompt.append({'role':message.role, 'content':new_content})

        return substituted_prompt

    def run_request(self, substituted_prompt, top_p=1.0, temperature=0.7, max_tokens=400, stops=[], log=False, trace=True):
        global vllm_model
        #
        ### first substitute for {{$var-name}} in prompt
        #
        global pattern
        #print(f'run_request {bindings}\n{prompt}\n')
        if trace:
            print(f'\n{json.dumps(substituted_prompt)}\n')      
        if log:
            logger.info(f'Prompt: {substituted_prompt}\n')
        if 'openai' in self.server_name:
            response = self.openai_client.executeRequest(prompt=substituted_prompt, temperature=temperature, top_p=top_p, max_tokens=max_tokens, stops=stops, model=self.model)
            return response
        elif 'openrouter' in self.server_name.lower():
            response= openrouter_client.executeRequest(prompt=substituted_prompt, temperature=temperature, top_p=top_p, max_tokens=max_tokens, stops=stops, model=self.model)
        elif 'vllm' in self.server_name:
            headers = {"Content-Type": "application/json"}
            url = 'http://localhost:5000/v1/completions'
            content = '\n'.join([msg['content'] for msg in substituted_prompt])
            response =  requests.post(url, headers= headers,
                                          json={"model":self.model, 
                                                "prompt":content, "temperature":0.0,
                                               "top_p":top_p, "max_tokens":max_tokens, "stop":stops})
        elif 'llama.cpp' in self.server_name:
            url = 'http://localhost:5000/v1/chat/completions'
            response =  requests.post(url, headers={"Content-Type":"application/json"},
                            json={"messages":substituted_prompt, "temperature":temperature,
                                    "top_p":top_p, "max_tokens":max_tokens, "stop":stops, "reasoning_effort": "low"})
        if response.status_code == 200:
            if 'vllm' in self.server_name:
                text = response.json()['choices'][0]['text']
                if (index := text.find('</think>')) > -1:
                    text = text[index+8:].strip()
                return text
            elif 'local' in self.server_name:
                try:
                    jsonr = response.json()
                    text = jsonr['choices'][0]['message']['content']
                    return text
                except Exception as e:
                    return response.content.decode('utf-8')
            else:
                # Default case for other server types
                try:
                    jsonr = response.json()
                    text = jsonr['choices'][0]['message']['content']
                    if text.startswith('{'):
                        try:
                            jsonr = json.loads(text)
                            return jsonr
                        except Exception as e:
                            traceback.print_exc()
                            return response.content.decode('utf-8')
                    return text
                except Exception as e:
                    return response.content.decode('utf-8')
        else:
            traceback.print_exc()
            raise Exception(response)

    def ask(self, input, prompt_msgs, template=None, tag='', temp=None, max_tokens=None, top_p=None, stops=None, is_json=False, model=None, log=True, trace=False):
        global elapsed_times, iteration_count
        if max_tokens is None: max_tokens = 400
        if temp is None: temp = 0.7
        if top_p is None: top_p = 1.0
          
        start = time.time()
        try:
            if response_prime_needed and type(prompt_msgs[-1]) != AssistantMessage:
                prompt_msgs = prompt_msgs + [AssistantMessage(content='')]
            print(f'{tag}...', end='')
            substituted_prompt = self.substitute_bindings(prompt_msgs, input)
            response = self.run_request(substituted_prompt, top_p=top_p, temperature=temp, max_tokens=max_tokens, stops=stops, log=log, trace=trace)
            #response = response.replace('<|im_end|>', '')
            elapsed = time.time()-start
            print(f'{elapsed:.2f}')
            if not tag or tag == '':
                tag = 'default'
            if tag != '':
                if tag not in elapsed_times.keys():
                    elapsed_times[tag] = 0
                elapsed_times[tag] += elapsed
                iteration_count += 1
                if iteration_count % 100 == 0:
                    print(f'iteration {iteration_count}')
                    for k,v in elapsed_times.items():
                        print(f'{k}: {v:.2f}')
                    print(f'total: {sum(elapsed_times.values()):.2f}')
                    print()
            if elapsed > 4.0:
                print(f'llm excessive time: {elapsed:.2f}')
            if stops is not None and type(response) is str: # claude returns eos
                if type(stops) is str:
                    stops = [stops]
                    for stop in stops:
                        eos_index=response.rfind(stop)
                        if eos_index > -1:
                            response=response[:eos_index]
            if is_json:
                if type(response) == dict:
                    return response
                try:
                    if type(response) == str:
                        response = response.strip()
                    response = json.loads(response)
                except Exception as e:
                    response = self.repair_json(substituted_prompt, response, e)
            if log:
                logger.info(f'Response:\n{response}\n')
            return response
        except Exception as e:
            traceback.print_exc()
            return None
       
    def repair_json(self, prompt,response, error):
        """Repair JSON if it is invalid"""

        if not response.startswith('{')and '{' in response:
            start = response.find('{')
            end = response.rfind('}')
            response = response[start:end+1]

        # Remove newlines that are outside of string values
        in_string = False
        result = []
        i = 0
        while i < len(response):
            if response[i] == '"' and (i == 0 or response[i-1] != '\\'):
                in_string = not in_string
            if not in_string and response[i] == '\n':
                i += 1
                continue
            result.append(response[i])
            i += 1
        response = ''.join(result)

        # Find first complete JSON form
        brace_count = 0
        json_end = 0
        for i, char in enumerate(response):
            if char == '{':
                brace_count += 1
            elif char == '}':
                brace_count -= 1
                if brace_count == 0:
                    json_end = i + 1
                    break
        if json_end > 0:
            response = response[:json_end]

        try:
            return json.loads(response)
        except Exception as e:
            error = e

        # Ok, ask llm
        prompt = [UserMessage(content="""You are a JSON repair tool.
An LLM received the following prompt and returned invalid JSON. Your task is to repair the JSON.

The prompt was:
{{$prompt}}

The returned JSON was:
{{$json}}

The reported error was:
{{$error}}

If it seems the JSON was truncated, it may have exceeded the max_tokens limit. In that case, try shortening some string values and completing the JSON according to the prompt.
Respond with the repaired JSON string. Make sure the string is in a format that can be parsed by the json.loads function. No commentary, no code fences.
""")]

        response = self.ask({"json": response, "error": error, "prompt": prompt}, prompt, tag='repair_json', temp=0.2, max_tokens=3500)
        try:
            return json.loads(response.replace("```json", "").replace("```", "").strip())
        except Exception as e:
            print(f'Error parsing JSON: {e}')
            return None
