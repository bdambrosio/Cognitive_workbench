#!/usr/bin/env python3
"""
Zenoh Perception Node

PURPOSE:
This node processes sensory data and perception-related events for a character.
It serves as the interface between raw sense data and higher-level cognitive processing.

RESPONSIBILITIES:
- Receive and process sense_data from the environment (currently handled by situation_node)
- Receive and track 'use' action results from executive_node
- Build and maintain perceptual models of the environment
- Provide processed perception data to other cognitive nodes

FUTURE ENHANCEMENTS:
- Object recognition and tracking
- Spatial reasoning about perceived entities
- Prediction of entity behavior based on observations
- Integration with memory for persistent perceptual knowledge
"""

import zenoh
import json
import time
import logging
import sys
import signal
import argparse
import os
from pathlib import Path
from typing import Dict, Any
from llm_client import ZenohLLMClient

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/perception_node.log', mode='w')
file_handler.setLevel(logging.WARNING)
if os.getenv('CWB_DEBUG', '') in ('1', 'true', 'yes', 'on'):
    file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('perception_node')

# Prompt template for comparing expectations against actual results

class ZenohPerceptionNode:
    """
    The Perception node processes sensory input and builds perceptual models.
    
    Features:
    - Process sense_data from environment
    - Track action results with expectations
    - Maintain perceptual state (future)
    """
    
    def __init__(self, character_name="default", character_config=None):
        # Store character info (canonicalized)
        self.character_name = character_name.capitalize()
        self.character_config = character_config or {}
        
        # Debug mode flag
        self.debug = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
        if self.debug:
            logger.info(f'🔧 Debug mode enabled for {self.character_name}')
        
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Initialize LLM client for expectation comparison (optional feature)
        compare_expectations = True
        if compare_expectations:
            llm_config = self.character_config.get('llm_config', {})
            server_name = llm_config.get('server_name', 'openai')
            model_name = llm_config.get('model_name', 'gpt-4.1')
            self.llm_client = ZenohLLMClient(server_name=server_name, model_name=model_name)
            logger.info(f'🤖 LLM expectation comparison enabled for {self.character_name} (server={server_name}, model={model_name})')
        else:
            self.llm_client = None
        
        logger.info(f'🎯 Perception Node initialized for {self.character_name}')
        
        # Subscriber for sense data (character-specific)
        self.sense_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/sense_data",
            self.sense_data_callback
        )
        
        # Subscriber for action results (character-specific)
        self.action_result_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/perception/action_result",
            self.action_result_callback
        )
        
        # Publisher for action anomalies (character-specific)
        self.action_anomaly_publisher = self.session.declare_publisher(
            f"cognitive/{character_name}/perception/action_anomaly"
        )
        
        logger.info(f'✅ {self.character_name} perception_node ready')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/perception/action_result')
        logger.info(f'   - Publishing to: cognitive/{character_name}/perception/action_anomaly')
    
    def sense_data_callback(self, sample):
        """Handle incoming sense data from the environment."""
        try:
            payload = sample.payload.to_bytes().decode('utf-8')
            sense_data = json.loads(payload)
            
            if self.debug:
                logger.info(f'Received sense_data: {json.dumps(sense_data, indent=2)[:200]}...')
            
            # TODO: Process sense data and update perceptual models
            # For now, this is a stub - processing will be added in future iterations
            
        except Exception as e:
            logger.error(f'Error processing sense_data: {e}')
    
    def action_result_callback(self, sample):
        """Handle action results with expectations for all monitored actions."""
        try:
            payload = sample.payload.to_bytes().decode('utf-8')
            action_result = json.loads(payload)
            
            action = action_result.get('action', {})
            update_text = action_result.get('update_text', '')
            expect = action_result.get('expect', '')
            
            action_type = action.get('type', 'unknown')
            
            if self.debug:
                logger.info(f'Received action_result: type={action_type}, '
                           f'expect={expect[:50]}..., result={update_text[:50]}...')
            
            # Compare expectation against actual result using LLM
            if self.llm_client and expect and update_text:
                prompt_text = f"""Compare the expected outcome against the actual result for an action.
#Action:
{{$action_text}}

#Expected: 
{{$expect_text}}

#Actual Result:
{{$update_text}}

Are these significantly different? Answer 'yes' or 'no', followed by a brief explanation (max 20 words).
"""
                response = self.llm_client.generate(
                    [prompt_text],
                    bindings={
                        "action_text": action,
                        "expect_text": expect,
                        "update_text": update_text
                    },
                    max_tokens=100,
                    temperature=0.3,
                    timeout=30.0
                )
                if response.success:
                    # Publish action anomaly with LLM comparison
                    anomaly_payload = {
                        'action': action,
                        'update_text': update_text,
                        'response': response.text
                    }
                    self.action_anomaly_publisher.put(json.dumps(anomaly_payload))
                    
                    if response.text.strip().lower().startswith('yes'):
                        logger.warning(f'Significant expectation mismatch for {action_type}: {response.text}')
                    else:
                        logger.info(f'Expectation match for {action_type}: {response.text}')
            
        except Exception as e:
            logger.error(f'Error processing action_result: {e}')
    
    def shutdown(self):
        """Clean shutdown of the perception node."""
        logger.info(f'Shutting down perception_node for {self.character_name}')
        self.shutdown_requested = True
        if self.llm_client:
            self.llm_client.cleanup()
        self.session.close()


def signal_handler(signum, frame):
    """Handle shutdown signals."""
    logger.info('Received shutdown signal')
    if 'node' in globals():
        node.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Zenoh Perception Node')
    parser.add_argument('-c', '--character', default='default', help='Character name')
    parser.add_argument('-config', '--config', default='{}', help='Character configuration JSON')
    args = parser.parse_args()
    
    # Parse character config
    try:
        character_config = json.loads(args.config)
    except json.JSONDecodeError:
        logger.error(f'Invalid character config JSON: {args.config}')
        character_config = {}
    
    # Create and run the node
    node = ZenohPerceptionNode(args.character, character_config)
    
    logger.info(f'Perception node running for {args.character}')
    
    # Keep the node alive
    try:
        while not node.shutdown_requested:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info('Keyboard interrupt received')
    finally:
        node.shutdown()

