#!/usr/bin/env python3
"""
Zenoh LLM Client

This module provides a simple client interface for making LLM requests via Zenoh.
Replaces ROS2 client complexity with direct Zenoh pub/sub.
"""

import zenoh
import json
import time
import threading
import uuid
import logging
import sys
from datetime import datetime
from typing import Dict, List, Any, Optional
from concurrent.futures import Future, ThreadPoolExecutor

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/llm_client.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
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
    
    def __init__(self, service_timeout: float = 30.0):
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
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
    
    def generate(self, 
                messages: List[str], 
                bindings: Dict[str, Any] = None,
                max_tokens: int = 150, 
                temperature: float = 0.7, 
                stops: List[str] = ['</end>'],
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
        future = self.generate_async(messages, bindings, max_tokens, temperature, stops)
        
        try:
            timeout_value = timeout or self.service_timeout
            result = future.result(timeout=timeout_value)
            return result
        except Exception as e:
            return LLMResponse(
                text="",
                success=False,
                error=f"Request failed: {str(e)}",
                request_id=future.request_id
            )
    
    def generate_async(self,
                      messages: List[str],
                      bindings: Dict[str, Any] = None,
                      max_tokens: int = 150,
                      temperature: float = 0.7,
                      stops: List[str] = None) -> Future:
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
        
        logger.debug(f'📤 Sent LLM request {request_id}')
        
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
                logger.info(f'❌ Cancelled LLM request {request_id}')
    
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