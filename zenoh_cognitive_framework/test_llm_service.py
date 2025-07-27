#!/usr/bin/env python3
"""
Test Driver for LLM Service Node

This script tests the LLM service node by sending various requests and verifying responses.
It can be used to test both the service functionality and performance.
"""

import zenoh
import json
import time
import threading
import logging
import sys
import uuid
from datetime import datetime
from typing import Dict, List, Any, Optional
from concurrent.futures import Future, ThreadPoolExecutor

# Configure logging
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/test_llm_service.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('test_llm_service')


class LLMResponse:
    """Simple LLM response structure for testing."""
    def __init__(self, text: str, success: bool = True, error: str = "", request_id: str = ""):
        self.text = text
        self.success = success
        self.error = error
        self.request_id = request_id
        self.timestamp = datetime.now().isoformat()


class LLMServiceTester:
    """
    Test driver for the LLM service node.
    
    Features:
    - Send various types of test requests
    - Verify responses and timing
    - Concurrent request testing
    - Performance benchmarking
    - Error handling tests
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
        self.completed_requests = {}
        self.service_timeout = service_timeout
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Test statistics
        self.test_stats = {
            'total_requests': 0,
            'successful_requests': 0,
            'failed_requests': 0,
            'total_time': 0.0,
            'avg_response_time': 0.0
        }
        
        logger.info('🤖 LLM Service Tester initialized')
    
    def send_request(self, 
                    messages: List[str], 
                    bindings: Dict[str, Any] = None,
                    max_tokens: int = 150, 
                    temperature: float = 0.7, 
                    stops: List[str] = None,
                    timeout: float = None) -> LLMResponse:
        """
        Send a single request to the LLM service.
        
        Args:
            messages: List of message strings (first is system prompt)
            bindings: Optional bindings for the LLM
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature
            stops: Stop sequences
            timeout: Request timeout
            
        Returns:
            LLMResponse object with the result
        """
        future = self.send_request_async(messages, bindings, max_tokens, temperature, stops)
        
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
    
    def send_request_async(self,
                          messages: List[str],
                          bindings: Dict[str, Any] = None,
                          max_tokens: int = 150,
                          temperature: float = 0.7,
                          stops: List[str] = None) -> Future:
        """
        Send a request asynchronously.
        
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
            self.pending_requests[request_id] = {
                'future': future,
                'start_time': time.time(),
                'request_data': request_data
            }
        
        # Publish request
        self.request_publisher.put(json.dumps(request_data))
        
        logger.info(f'📤 Sent test request {request_id}')
        
        return future
    
    def _handle_response(self, sample):
        """Handle incoming LLM responses."""
        try:
            response_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            request_id = response_data.get('request_id', '')
            
            with self._lock:
                if request_id in self.pending_requests:
                    # Calculate timing
                    request_info = self.pending_requests[request_id]
                    processing_time = time.time() - request_info['start_time']
                    
                    # Create response object
                    llm_response = LLMResponse(
                        text=response_data.get('response', ''),
                        success=response_data.get('success', False),
                        error=response_data.get('error', ''),
                        request_id=request_id
                    )
                    
                    # Update statistics
                    self.test_stats['total_requests'] += 1
                    self.test_stats['total_time'] += processing_time
                    
                    if llm_response.success:
                        self.test_stats['successful_requests'] += 1
                    else:
                        self.test_stats['failed_requests'] += 1
                    
                    # Update average response time
                    if self.test_stats['total_requests'] > 0:
                        self.test_stats['avg_response_time'] = (
                            self.test_stats['total_time'] / self.test_stats['total_requests']
                        )
                    
                    # Complete the future
                    future = request_info['future']
                    if not future.done():
                        future.set_result(llm_response)
                    
                    # Move to completed requests
                    self.completed_requests[request_id] = {
                        'response': llm_response,
                        'processing_time': processing_time,
                        'request_data': request_info['request_data']
                    }
                    
                    # Clean up pending request
                    del self.pending_requests[request_id]
                    
                    logger.info(f'✅ Received response for {request_id} in {processing_time:.2f}s')
                    
        except Exception as e:
            logger.error(f'❌ Error handling LLM response: {e}')
    
    def test_basic_functionality(self):
        """Test basic LLM service functionality."""
        logger.info('🧪 Testing basic functionality...')
        
        # Test 1: Simple question
        response1 = self.send_request(
            messages=[
                "You are a helpful AI assistant.",
                "What is 2 + 2?"
            ],
            max_tokens=50,
            temperature=0.1
        )
        
        logger.info(f'Test 1 - Simple question: {response1.success}')
        if response1.success:
            logger.info(f'Response: {response1.text[:100]}...')
        else:
            logger.error(f'Error: {response1.error}')
        
        # Test 2: Creative response
        response2 = self.send_request(
            messages=[
                "You are a creative writer.",
                "Write a short poem about robots."
            ],
            max_tokens=100,
            temperature=0.8
        )
        
        logger.info(f'Test 2 - Creative response: {response2.success}')
        if response2.success:
            logger.info(f'Response: {response2.text[:100]}...')
        else:
            logger.error(f'Error: {response2.error}')
        
        # Test 3: With bindings
        response3 = self.send_request(
            messages=[
                "You are a helpful assistant.",
                "Hello, my name is {{$name}}. How are you today?"
            ],
            bindings={'name': 'Alice'},
            max_tokens=50,
            temperature=0.3
        )
        
        logger.info(f'Test 3 - With bindings: {response3.success}')
        if response3.success:
            logger.info(f'Response: {response3.text[:100]}...')
        else:
            logger.error(f'Error: {response3.error}')
    
    def test_concurrent_requests(self, num_requests: int = 5):
        """Test concurrent request handling."""
        logger.info(f'🧪 Testing {num_requests} concurrent requests...')
        
        # Create concurrent requests
        futures = []
        for i in range(num_requests):
            future = self.send_request_async(
                messages=[
                    "You are a helpful assistant.",
                    f"Request number {i + 1}: What is the capital of France?"
                ],
                max_tokens=30,
                temperature=0.1
            )
            futures.append(future)
        
        # Wait for all responses
        responses = []
        for i, future in enumerate(futures):
            try:
                response = future.result(timeout=self.service_timeout)
                responses.append(response)
                logger.info(f'Concurrent request {i + 1}: {response.success}')
            except Exception as e:
                logger.error(f'Concurrent request {i + 1} failed: {e}')
        
        return responses
    
    def test_error_handling(self):
        """Test error handling scenarios."""
        logger.info('🧪 Testing error handling...')
        
        # Test 1: Empty messages
        response1 = self.send_request(
            messages=[],
            max_tokens=50
        )
        
        logger.info(f'Test 1 - Empty messages: {response1.success}')
        if not response1.success:
            logger.info(f'Expected error: {response1.error}')
        
        # Test 2: Very long message
        long_message = "This is a very long message. " * 1000
        response2 = self.send_request(
            messages=[
                "You are a helpful assistant.",
                long_message
            ],
            max_tokens=50
        )
        
        logger.info(f'Test 2 - Long message: {response2.success}')
        if response2.success:
            logger.info(f'Response: {response2.text[:100]}...')
        else:
            logger.info(f'Error: {response2.error}')
    
    def test_performance(self, num_requests: int = 10):
        """Test performance with multiple requests."""
        logger.info(f'🧪 Testing performance with {num_requests} requests...')
        
        start_time = time.time()
        
        # Send requests sequentially
        for i in range(num_requests):
            response = self.send_request(
                messages=[
                    "You are a helpful assistant.",
                    f"Performance test request {i + 1}: What is the weather like?"
                ],
                max_tokens=30,
                temperature=0.1
            )
            
            if response.success:
                logger.info(f'Request {i + 1}: Success in {time.time() - start_time:.2f}s')
            else:
                logger.error(f'Request {i + 1}: Failed - {response.error}')
        
        total_time = time.time() - start_time
        logger.info(f'Performance test completed in {total_time:.2f}s')
        logger.info(f'Average time per request: {total_time / num_requests:.2f}s')
    
    def print_statistics(self):
        """Print test statistics."""
        logger.info('📊 Test Statistics:')
        logger.info(f'  Total requests: {self.test_stats["total_requests"]}')
        logger.info(f'  Successful: {self.test_stats["successful_requests"]}')
        logger.info(f'  Failed: {self.test_stats["failed_requests"]}')
        logger.info(f'  Success rate: {self.test_stats["successful_requests"] / max(1, self.test_stats["total_requests"]) * 100:.1f}%')
        logger.info(f'  Average response time: {self.test_stats["avg_response_time"]:.2f}s')
        logger.info(f'  Total test time: {self.test_stats["total_time"]:.2f}s')
    
    def run_all_tests(self):
        """Run all test scenarios."""
        logger.info('🚀 Starting comprehensive LLM service tests...')
        
        try:
            # Test 1: Basic functionality
            self.test_basic_functionality()
            time.sleep(2)  # Brief pause between tests
            
            # Test 2: Concurrent requests
            self.test_concurrent_requests(3)
            time.sleep(2)
            
            # Test 3: Error handling
            self.test_error_handling()
            time.sleep(2)
            
            # Test 4: Performance
            self.test_performance(5)
            
            # Print final statistics
            self.print_statistics()
            
            logger.info('✅ All tests completed!')
            
        except Exception as e:
            logger.error(f'❌ Test suite failed: {e}')
    
    def cleanup(self):
        """Cleanup resources."""
        try:
            # Wait for any pending requests
            if self.pending_requests:
                logger.info(f'Waiting for {len(self.pending_requests)} pending requests...')
                for request_id, request_info in list(self.pending_requests.items()):
                    try:
                        request_info['future'].result(timeout=5.0)
                    except:
                        pass
            
            # Close session
            self.session.close()
            logger.info('Test driver cleanup complete')
            
        except Exception as e:
            logger.error(f'Error during cleanup: {e}')


def main():
    """Main test function."""
    tester = LLMServiceTester()
    
    try:
        # Run all tests
        tester.run_all_tests()
        
    except KeyboardInterrupt:
        logger.info('Test interrupted by user')
    except Exception as e:
        logger.error(f'Test failed: {e}')
    finally:
        tester.cleanup()


if __name__ == '__main__':
    main() 