#!/usr/bin/env python3
"""
Zenoh LLM Service Node

This node provides LLM (Large Language Model) services via Zenoh.
Replaces ROS2 service complexity with direct Zenoh pub/sub.
"""

import zenoh
import json
import time
import threading
import logging
import sys
import signal
from datetime import datetime
from typing import Dict, List, Any, Optional
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass

# Configure logging with unbuffered output
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[
        logging.StreamHandler(sys.stdout),  # Console output
        logging.FileHandler('logs/llm_service_node.log', mode='w')  # File output
    ],
    force=True
)
logger = logging.getLogger('llm_service_node')

# Import LLM API
import os
# Add current directory to Python path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from llm_api import LLM
    from Messages import SystemMessage, UserMessage, AssistantMessage
    LLM_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  LLM API not available: {e}")
    print("   LLM service will return mock responses")
    LLM_AVAILABLE = False


class LLMRequest:
    """Simple LLM request structure."""
    def __init__(self, messages, bindings, max_tokens: int = 150, temperature: float = 0.7, stops: list = ['</end>']):
        self.bindings = bindings
        self.messages = messages
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.stops = stops
        self.timestamp = datetime.now().isoformat()


class LLMResponse:
    """Simple LLM response structure."""
    def __init__(self, response: str, success: bool = True, error: str = "", request_id: str = ""):
        self.response = response
        self.success = success
        self.error = error
        self.request_id = request_id
        self.timestamp = datetime.now().isoformat()


class ZenohLLMServiceNode:
    """
    The LLM Service node provides:
    - LLM request/response handling
    - Concurrent request processing
    - Request statistics and monitoring
    - Error handling and fallbacks
    """
    
    def __init__(self):
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Subscriber for LLM requests
        self.request_subscriber = self.session.declare_subscriber(
            "cognitive/llm_request",
            self.handle_llm_request
        )
        
        # Publisher for LLM responses
        self.response_publisher = self.session.declare_publisher("cognitive/llm_response")
        
        # Thread pool for concurrent processing
        self.thread_pool = ThreadPoolExecutor(max_workers=4)
        self.active_requests = {}
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Request tracking
        self.request_counter = 0
        self.request_stats = {
            'total_requests': 0,
            'successful_requests': 0,
            'failed_requests': 0,
            'avg_response_time': 0.0
        }
        
        # Initialize LLM
        self.llm = None
        if LLM_AVAILABLE:
            try:
                # Try to initialize with vllm server
                llm_params = {'server_name': 'vllm'}
                self.llm = LLM(**llm_params)
                logger.info(f'LLM __init__ accepts parameters: {llm_params}')
                logger.info('✅ LLM API initialized with vllm server')
            except Exception as e:
                try:
                    # Fallback to default initialization
                    self.llm = LLM()
                    logger.info('✅ LLM API initialized with default parameters')
                except Exception as e2:
                    logger.error(f'Failed to initialize LLM: {e2}')
                    self.llm = None
        
        logger.info('🤖 Zenoh LLM Service Node initialized - ready for concurrent requests')
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        logger.info(f'Received signal {signum}, initiating shutdown...')
        self.shutdown_requested = True
    
    def run(self):
        """Main node loop."""
        try:
            logger.info('LLM Service Node running - press Ctrl+C to stop')
            while not self.shutdown_requested:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info('LLM Service Node shutting down...')
        finally:
            self.shutdown()
    
    def handle_llm_request(self, sample):
        """Handle incoming LLM requests."""
        # Check if shutdown has been requested
        if self.shutdown_requested:
            logger.warning('Ignoring LLM request - shutdown in progress')
            return
            
        try:
            # Parse request JSON
            request_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            request_id = request_data.get('request_id', f'req_{self.request_counter}')
            self.request_counter += 1
            
            # Create LLM request object
            llm_request = LLMRequest(
                messages=request_data.get('messages', []),
                bindings=request_data.get('bindings', {}),
                max_tokens=request_data.get('max_tokens', 150),
                temperature=request_data.get('temperature', 0.7),
                stops=request_data.get('stops', ['</end>'])
            )
            
            # Log the first message as a preview
            preview_text = str(llm_request.messages[0])[:50] if llm_request.messages else "empty"
            logger.info(f'📥 Received LLM request {request_id}: "{preview_text}..."')
            
            # Check if thread pool is still active before submitting
            if self.thread_pool._shutdown:
                logger.warning(f'Thread pool is shutdown, rejecting request {request_id}')
                error_response = {
                    'request_id': request_id,
                    'response': '',
                    'success': False,
                    'error': 'Service is shutting down',
                    'processing_time': 0.0,
                    'timestamp': datetime.now().isoformat()
                }
                self.response_publisher.put(json.dumps(error_response))
                return
            
            # Submit to thread pool for async processing
            future = self.thread_pool.submit(self._process_llm_request, request_id, llm_request)
            self.active_requests[request_id] = {
                'future': future,
                'start_time': time.time(),
                'request': llm_request
            }
            
            self.request_stats['total_requests'] += 1
            
        except Exception as e:
            error_msg = f'Error handling LLM request: {str(e)}'
            logger.error(error_msg)
            
            # Publish error response
            error_response = {
                'request_id': request_data.get('request_id', 'unknown'),
                'response': '',
                'success': False,
                'error': error_msg,
                'processing_time': 0.0,
                'timestamp': datetime.now().isoformat()
            }
            
            self.response_publisher.put(json.dumps(error_response))
    
    def _process_llm_request(self, request_id: str, llm_request: LLMRequest) -> None:
        """
        Process an LLM request in a background thread.
        
        This method does the actual blocking LLM call and publishes the response.
        """
        start_time = time.time()
        
        try:
            if self.llm is not None and LLM_AVAILABLE:
                # Real LLM call
                messages = []
                for n, message in enumerate(llm_request.messages):
                    if n == 0:
                        messages.append(SystemMessage(content=message))
                    else:
                        messages.append(UserMessage(content=message))
                messages[-1].content = messages[-1].content + "\n\nend your response with </end>"

                stops = llm_request.stops
                if len(stops) == 0:
                    stops = ['</end>']
                
                # This is the blocking call, but it's in a separate thread
                response_text = self.llm.ask(llm_request.bindings, messages, temp=llm_request.temperature, max_tokens=llm_request.max_tokens, stops=stops)
                
                llm_response = LLMResponse(
                    response=response_text,
                    success=True,
                    request_id=request_id
                )
                
                self.request_stats['successful_requests'] += 1
                
            else:
                # Mock response when LLM not available
                mock_input = str(llm_request.messages[-1]) if llm_request.messages else "empty input"
                response_text = f"[MOCK] Cognitive response to: {mock_input}"
                llm_response = LLMResponse(
                    response=response_text,
                    success=True,
                    request_id=request_id
                )
                
                # Simulate processing time
                time.sleep(0.5)
                self.request_stats['successful_requests'] += 1
            
        except Exception as e:
            error_msg = f'LLM processing error: {str(e)}'
            logger.error(error_msg)
            
            llm_response = LLMResponse(
                response="",
                success=False,
                error=error_msg,
                request_id=request_id
            )
            
            self.request_stats['failed_requests'] += 1
        
        # Calculate timing
        processing_time = time.time() - start_time
        self._update_avg_response_time(processing_time)
        
        # Publish response
        response_msg = {
            'request_id': request_id,
            'response': llm_response.response,
            'success': llm_response.success,
            'error': llm_response.error,
            'processing_time': processing_time,
            'timestamp': llm_response.timestamp
        }
        
        self.response_publisher.put(json.dumps(response_msg))
        
        # Clean up tracking
        if request_id in self.active_requests:
            del self.active_requests[request_id]
        
        logger.info(f'📤 Completed LLM request {request_id} in {processing_time:.2f}s')
    
    def _update_avg_response_time(self, processing_time: float):
        """Update rolling average response time."""
        total = self.request_stats['total_requests']
        if total > 0:
            current_avg = self.request_stats['avg_response_time']
            # Weighted average favoring recent requests
            weight = min(0.1, 1.0 / total)
            self.request_stats['avg_response_time'] = (
                current_avg * (1 - weight) + processing_time * weight
            )
    
    def publish_status(self):
        """Publish service status for monitoring."""
        if len(self.active_requests) > 0:
            print(
                f'📊 LLM Service Status: {len(self.active_requests)} active, '
                f'{self.request_stats["total_requests"]} total, '
                f'{self.request_stats["avg_response_time"]:.2f}s avg'
            )
        
        # Schedule next status update
        self.status_timer = threading.Timer(10.0, self.publish_status)
        self.status_timer.start()
    
    def shutdown(self):
        """Clean shutdown."""
        logger.info('Shutting down LLM service...')
        
        # Set shutdown flag to prevent new requests
        self.shutdown_requested = True
        
        # Wait for active requests to complete (with timeout)
        if self.active_requests:
            logger.info(f'Waiting for {len(self.active_requests)} active requests...')
            for request_id, request_info in list(self.active_requests.items()):
                try:
                    request_info['future'].result(timeout=5.0)
                except Exception as e:
                    logger.warning(f'Request {request_id} did not complete: {e}')
        
        # Shutdown thread pool gracefully
        try:
            self.thread_pool.shutdown(wait=True, timeout=10.0)
            logger.info('Thread pool shutdown complete')
        except Exception as e:
            logger.error(f'Error shutting down thread pool: {e}')
        
        # Close Zenoh session
        try:
            self.session.close()
            logger.info('Zenoh session closed')
        except Exception as e:
            logger.error(f'Error closing Zenoh session: {e}')
        
        logger.info('LLM Service shutdown complete')


def main():
    """Main entry point for the LLM service node."""
    llm_service_node = ZenohLLMServiceNode()
    try:
        llm_service_node.run()
    finally:
        llm_service_node.shutdown()


if __name__ == '__main__':
    main() 