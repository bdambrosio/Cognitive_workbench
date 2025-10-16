#!/usr/bin/env python3
"""
Zenoh Agenda Node

PURPOSE:
This node manages a character's agenda, commitments, and higher-level task coordination.
It serves as a supervisory layer above executive_node's OODA loop, managing longer-term
goals, social commitments, and activity scheduling.

RESPONSIBILITIES:
- Process discourse analysis to extract commitments and agreements
- Track selected activities from executive_node
- Manage activity lifecycle and transitions
- Coordinate between social obligations and personal goals
- Supervise executive_node's action selection (future)

FUTURE ENHANCEMENTS:
- Commitment tracking and fulfillment monitoring
- Activity scheduling and prioritization
- Conflict resolution between competing goals
- Social obligation management (promises, agreements, requests)
- Long-term goal decomposition into actionable tasks
- Integration with planning for multi-step goal achievement
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
from typing import Dict, Any, List

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/agenda_node.log', mode='w')
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
logger = logging.getLogger('agenda_node')


class ZenohAgendaNode:
    """
    The Agenda node manages a character's commitments, activities, and long-term goals.
    
    Features:
    - Track discourse-derived commitments and agreements
    - Monitor current activity selection
    - Coordinate task and goal management (future)
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
        
        # Agenda state (stub for now)
        self.current_activity = None
        self.commitments = []
        self.pending_tasks = []
        
        logger.info(f'📋 Agenda Node initialized for {self.character_name}')
        
        # Subscriber for discourse analysis (character-specific)
        # Will receive updates when entity_model creates discourse analysis
        self.discourse_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/agenda/discourse_analysis",
            self.discourse_analysis_callback
        )
        
        # Subscriber for activity selection (character-specific)
        # Receives updates when executive_node selects a new activity
        self.activity_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/current_activity",
            self.activity_callback
        )
        
        # TODO: Add subscribers for:
        # - Task completion notifications
        # - Goal updates
        # - Planning results
        # - Social interaction outcomes
        
        logger.info(f'✅ {self.character_name} agenda_node ready')
    
    def discourse_analysis_callback(self, sample):
        """Handle discourse analysis updates from entity_model."""
        try:
            payload = sample.payload.to_bytes().decode('utf-8')
            analysis = json.loads(payload)
            
            if self.debug:
                logger.info(f'Received discourse_analysis: {json.dumps(analysis, indent=2)[:200]}...')
            
            # TODO: Extract commitments, agreements, requests from discourse analysis
            # For now, this is a stub - processing will be added in future iterations
            
        except Exception as e:
            logger.error(f'Error processing discourse_analysis: {e}')
    
    def activity_callback(self, sample):
        """Handle activity selection updates from executive_node."""
        try:
            payload = sample.payload.to_bytes().decode('utf-8')
            activity_data = json.loads(payload)
            
            activity_name = activity_data.get('activity', 'unknown')
            self.current_activity = activity_name
            
            if self.debug:
                logger.info(f'Activity selected: {activity_name}')
            
            # TODO: Update agenda based on activity selection
            # For now, this is a stub - processing will be added in future iterations
            
        except Exception as e:
            logger.error(f'Error processing activity: {e}')
    
    def shutdown(self):
        """Clean shutdown of the agenda node."""
        logger.info(f'Shutting down agenda_node for {self.character_name}')
        self.shutdown_requested = True
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
    parser = argparse.ArgumentParser(description='Zenoh Agenda Node')
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
    node = ZenohAgendaNode(args.character, character_config)
    
    logger.info(f'Agenda node running for {args.character}')
    
    # Keep the node alive
    try:
        while not node.shutdown_requested:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info('Keyboard interrupt received')
    finally:
        node.shutdown()

