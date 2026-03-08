#!/usr/bin/env python3
"""
Zenoh LLM Client

This module provides a simple client interface for making LLM requests via Zenoh.
Replaces ROS2 client complexity with direct Zenoh pub/sub.
"""

import traceback
import zenoh
import json
import time
import threading
import uuid
import logging
import sys
from datetime import datetime
from typing import Dict, List, Any, Optional
from Messages import SystemMessage, UserMessage
from utils.llm_api import LLM
from concurrent.futures import Future, ThreadPoolExecutor

# Create logger that inherits from parent app's logging configuration
logger = logging.getLogger('llm_client')


class LLMResponse:
    """Simple LLM response structure."""
    def __init__(self, text: str, success: bool = True, error: str = "", request_id: str = ""):
        self.text = text
        self.success = success
        self.error = error
        self.request_id = request_id
        self.timestamp = datetime.now().isoformat()


class ZenohLLMClient:
    """
    Zenoh client for making LLM requests.
    
    Features:
    - Simple request/response interface
    - Automatic request ID generation
    - Response correlation and timeout handling
    - Thread-safe operations
    """
    
    def __init__(self, server_name='openai', model_name='gpt-4.1', service_timeout: float = 600.0):
        # Initialize Zenoh session (localhost only)
        from utils.zenoh_utils import make_localhost_config
        self.session = zenoh.open(make_localhost_config())
        self.llm = LLM(server_name=server_name, model_name=model_name)
        
        # Publisher for LLM requests
        self.request_publisher = self.session.declare_publisher("cognitive/llm_request")
        
        # Subscriber for LLM responses
        self.response_subscriber = self.session.declare_subscriber(
            "cognitive/llm_response",
            self._handle_response
        )
        
        # Response tracking
        self.pending_requests = {}
        self.response_callbacks = {}
        self.service_timeout = service_timeout
        
        # Thread safety
        self._lock = threading.Lock()
        
        logger.info('🤖 Zenoh LLM Client initialized')

    def substitute_bindings(self, prompt, bindings):
        return self.llm.substitute_bindings(prompt, bindings)

    def ask(self, bindings: Dict[str, Any] = None, prompt: str = None, max_tokens: int = 150, temp: float = 0.7, stops: List[str] = ['</end>'], is_json: bool = False, log: bool = False, trace: bool = False, timeout: float = 120.0) -> LLMResponse:
        with ThreadPoolExecutor(max_workers=1) as executor:
            # First attempt
            future = executor.submit(self.llm.ask, bindings, prompt, max_tokens=max_tokens, temp=temp, stops=stops, is_json=is_json, log=log, trace=trace)
            try:
                response = future.result(timeout=max(timeout, 120.0))
                return response
            except TimeoutError:
                logger.warning(f'⏱️ LLM request timeout after {timeout}s, retrying once...')
                future.cancel()
                
                # Brief backoff to let server clear
                time.sleep(1.0)
                
                # Single retry with same timeout
                future2 = executor.submit(self.llm.ask, bindings, prompt, max_tokens=max_tokens, temp=temp, stops=stops, is_json=is_json, log=log, trace=trace)
                try:
                    response = future2.result(timeout=max(timeout, 60.0))
                    logger.info(f'✅ LLM request succeeded on retry')
                    return response
                except TimeoutError:
                    logger.error(f'⏱️ LLM request timeout after retry, giving up')
                    logger.error(traceback.format_exc())
                    future2.cancel()
                    raise
    
    def generate(self, 
                messages: List[str], 
                bindings: Dict[str, Any] = None,
                max_tokens: int = 150, 
                temperature: float = 0.7, 
                stops: List[str] = ['</end>'],
                is_json: bool = False,
                timeout: float = None) -> LLMResponse:
        """
        Generate a response from the LLM service.
        
        Args:
            messages: List of message strings (first is system prompt)
            bindings: Optional bindings for the LLM
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature
            stops: Stop sequences
            timeout: Request timeout (uses service_timeout if None)
            
        Returns:
            LLMResponse object with the result
        """
        try:
            message_objects = []
            for n, message in enumerate(messages):
                if n == 0:
                    message_objects.append(SystemMessage(content=message))
                else:
                    message_objects.append(UserMessage(content=message))
 
            result = self.ask(bindings, message_objects, max_tokens=max_tokens, temp=temperature, stops=stops, is_json=is_json, log=True, trace=False)
            
            # Check if result is None, empty string, or empty dict - retry once if so
            if result is None or result == "" or result == {}:
                logger.error(f'❌ LLM returned empty result (None/empty string/empty dict), retrying once...')
                time.sleep(1.0)  # Brief backoff
                result = self.ask(bindings, message_objects, max_tokens=max_tokens, temp=temperature, stops=stops, is_json=is_json, log=True, trace=False)
                
                # Check retry result
                if result is None or result == "" or result == {}:
                    logger.error(f'❌ LLM returned empty result on retry, giving up')
                else:
                    logger.info(f'✅ LLM request succeeded on retry after empty result')
            
            """future = self.generate_async(messages, bindings, max_tokens, temperature, stops, is_json))
            timeout_value = timeout or self.service_timeout
            result = future.result(timeout=max(timeout_value, 200.0))
        """
            return LLMResponse(
                text=result,
                success=True,
                error=None,
                request_id=-1
            )
        except Exception as e:
            logger.error(f'Error in generate: {e}')
            return LLMResponse(
                text="",
                success=False,
                error=f"Request failed: {str(e)}",
                #request_id=future.request_id
            )
    
    def generate_async(self,
                      messages: List[str],
                      bindings: Dict[str, Any] = None,
                      max_tokens: int = 150,
                      temperature: float = 0.7,
                      stops: List[str] = None,
                      is_json: bool = False) -> Future:
        """
        Generate a response asynchronously.
        
        Returns:
            Future object that will contain the LLMResponse
        """
        # Generate request ID
        request_id = str(uuid.uuid4())
        
        # Create request data
        request_data = {
            'request_id': request_id,
            'messages': messages,
            'bindings': bindings or {},
            'max_tokens': max_tokens,
            'temperature': temperature,
            'stops': stops or ['</end>'],
            'is_json': is_json,
            'timestamp': datetime.now().isoformat()
        }
        
        # Create future for this request
        future = Future()
        future.request_id = request_id
        
        # Track the request
        with self._lock:
            self.pending_requests[request_id] = future
        
        # Publish request
        self.request_publisher.put(json.dumps(request_data))
        
        #logger.debug(f'📤 Sent LLM request {request_id}')
        
        return future
    
    def _handle_response(self, sample):
        """Handle incoming LLM responses."""
        try:
            response_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            request_id = response_data.get('request_id', '')
            
            with self._lock:
                if request_id in self.pending_requests:
                    # Create response object
                    llm_response = LLMResponse(
                        text=response_data.get('response', ''),
                        success=response_data.get('success', False),
                        error=response_data.get('error', ''),
                        request_id=request_id
                    )
                    
                    # Complete the future
                    future = self.pending_requests[request_id]
                    if not future.done():
                        future.set_result(llm_response)
                    
                    # Clean up
                    del self.pending_requests[request_id]
                    
                    logger.debug(f'✅ Received LLM response for {request_id}')
                    
        except Exception as e:
            logger.error(f'❌ Error handling LLM response: {e}')
    
    def cancel_request(self, request_id: str):
        """Cancel a pending request."""
        with self._lock:
            if request_id in self.pending_requests:
                future = self.pending_requests[request_id]
                if not future.done():
                    future.cancel()
                del self.pending_requests[request_id]
                #logger.info(f'❌ Cancelled LLM request {request_id}')
    
    def cleanup(self):
        """Clean up resources."""
        with self._lock:
            # Cancel all pending requests immediately
            for request_id in list(self.pending_requests.keys()):
                self.cancel_request(request_id)
        
        # Close Zenoh session more carefully
        try:
            # Wait longer for cleanup to avoid Zenoh panics
            time.sleep(2.0)
            self.session.close()
            logger.info('🧹 LLM Client cleanup completed')
        except Exception as e:
            logger.error(f'Error closing LLM client session: {e}')


# Convenience function for simple requests
def ask_llm(prompt: str, 
           system_prompt: str = "You are a helpful AI assistant.",
           bindings: Dict[str, Any] = None,
           max_tokens: int = 150,
           temperature: float = 0.7) -> str:
    """
    Simple function to ask the LLM a question.
    
    Args:
        prompt: The user's question/prompt
        system_prompt: System prompt for the LLM
        bindings: Optional bindings
        max_tokens: Maximum tokens to generate
        temperature: Sampling temperature
        
    Returns:
        The LLM's response text, or error message if failed
    """
    client = ZenohLLMClient()
    try:
        response = client.generate(
            messages=[system_prompt, prompt],
            bindings=bindings,
            max_tokens=max_tokens,
            temperature=temperature
        )
        return response.text if response.success else f"Error: {response.error}"
    finally:
        client.cleanup()


if __name__ == '__main__':
    # Simple test
    print("Testing LLM client...")
    response = ask_llm("What is 2+2?")
    print(f"Response: {response}") 