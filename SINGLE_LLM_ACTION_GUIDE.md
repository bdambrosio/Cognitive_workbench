# Single LLM Action Example Guide

This guide covers the new Single LLM Action Example configurations that provide a clean, simple demonstration of LLM integration.

## Overview

The `single_llm_action_example.py` node demonstrates a simplified LLM integration that makes exactly **ONE LLM call per text input**. This is perfect for:

- **Testing LLM behavior** without complex multi-call scenarios
- **Understanding LLM responses** in isolation
- **Demonstrations** where predictable behavior is needed
- **Debugging** LLM integration issues

## Key Features

- **Single LLM Call**: Exactly one LLM call per text input
- **Simple Prompt**: Clean, focused prompts for predictable responses
- **Fallback Handling**: Graceful fallback when LLM is unavailable
- **Clear Logging**: Detailed logging of LLM calls and responses
- **Action Publishing**: Publishes structured actions to `/cognitive/action_data`

## Available Launch Configurations

### 1. Cursor IDE Debug Configurations

#### Individual Node Debug
- **Name**: "Debug Single LLM Action Example"
- **Type**: debugpy
- **Program**: `cognitive_framework/single_llm_action_example.py`
- **Usage**: Debug the single LLM action node in isolation

#### System Debug
- **Name**: "Debug Single LLM Action System"
- **Type**: debugpy
- **Program**: `launch/single_llm_action_system.launch.py`
- **Usage**: Debug the complete system (sense + memory + single LLM action)

### 2. ROS2 Launch Files

#### Individual Node Launch
```bash
# Basic launch
ros2 launch cognitive_framework single_llm_action_example.launch.py

# With custom log level
ros2 launch cognitive_framework single_llm_action_example.launch.py log_level:=debug
```

#### Complete System Launch
```bash
# Launch sense + memory + single LLM action
ros2 launch cognitive_framework single_llm_action_system.launch.py

# With custom log level
ros2 launch cognitive_framework single_llm_action_system.launch.py log_level:=debug
```

### 3. Development Launchers

#### Simple Node Launcher
```bash
# Launch individual node
python3 launch_nodes.py single_llm_action_example --log-level debug

# With custom namespace
python3 launch_nodes.py single_llm_action_example --namespace my_agent
```

#### Simple Multi-Node Launcher
```bash
# Launch with other nodes
python3 launch/simple_launch.py --nodes sense_node memory_node single_llm_action_example

# Launch all nodes (includes single_llm_action_example)
python3 launch/simple_launch.py
```

## Usage Examples

### 1. Quick Test (Individual Node)
```bash
# Terminal 1: Start the node
ros2 launch cognitive_framework single_llm_action_example.launch.py

# Terminal 2: Send test input
ros2 topic pub /cognitive/external_text_input std_msgs/msg/String "data: 'Hello, how are you?'"
```

### 2. Complete System Test
```bash
# Terminal 1: Start the complete system
ros2 launch cognitive_framework single_llm_action_system.launch.py

# Terminal 2: Type in the sense node terminal
# The system will automatically process your input through the single LLM action
```

### 3. Debug in Cursor IDE
1. Open the Debug panel (Ctrl+Shift+D)
2. Select "Debug Single LLM Action Example"
3. Set breakpoints in the code
4. Press F5 to start debugging
5. Send input via ROS2 topic or console

## Node Behavior

### Input Processing
- Subscribes to `/cognitive/sense_data`
- Detects text input from console or external sources
- Triggers exactly ONE LLM call per input

### LLM Integration
- Uses `LLMClient` for API calls
- Simple, focused prompts for predictable responses
- 10-second timeout for LLM calls
- Graceful fallback on failure

### Action Creation
- Creates structured action objects
- Includes LLM response and metadata
- Publishes to `/cognitive/action_data`
- Tracks processing time and confidence

### Logging
- Detailed logging of input processing
- LLM call status and responses
- Action creation and publishing
- Error handling and fallbacks

## Troubleshooting

### Common Issues

1. **LLM Service Not Available**
   - Check if `llm_service_node` is running
   - Verify LLM API configuration
   - Node will use fallback actions

2. **No Text Input Detected**
   - Ensure `sense_node` is running
   - Check topic connections
   - Verify input format

3. **LLM Call Timeout**
   - Check network connectivity
   - Verify LLM service configuration
   - Increase timeout if needed

### Debug Commands
```bash
# Check node status
ros2 node list
ros2 node info /single_llm_action_example

# Monitor topics
ros2 topic echo /cognitive/sense_data
ros2 topic echo /cognitive/action_data

# Check node logs
ros2 node info /single_llm_action_example --show-parameters
```

## Configuration

### Log Levels
- `debug`: Detailed debugging information
- `info`: General operational information (default)
- `warn`: Warning messages only
- `error`: Error messages only

### Parameters
- `use_sim_time`: Set to false for real-time operation
- `service_timeout`: LLM service timeout (default: 15.0 seconds)

## Integration with Other Nodes

The single LLM action example works seamlessly with:

- **Sense Node**: Provides text input
- **Memory Node**: Stores conversation history
- **Action Display Node**: Shows generated actions
- **LLM Service Node**: Provides LLM API access

This creates a complete cognitive system with simplified, predictable LLM behavior. 