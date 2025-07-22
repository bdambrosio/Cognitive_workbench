# Launch Configurations for Cognitive Framework

This document describes the available launch configurations for the ROS2 Cognitive Framework.

## Available Launch Files

### Individual Node Launches

#### 1. Sense Node Only
```bash
# Launch just the sense node
ros2 launch cognitive_framework sense_node.launch.py

# With custom parameters
ros2 launch cognitive_framework sense_node.launch.py namespace:=my_agent log_level:=debug
```

**Features:**
- Publishes to `/cognitive/sense_data`
- Accepts console input in terminal
- Subscribes to `/cognitive/external_text_input` (optional)

#### 2. Memory Node Only
```bash
# Launch just the memory node
ros2 launch cognitive_framework memory_node.launch.py

# With custom parameters
ros2 launch cognitive_framework memory_node.launch.py namespace:=my_agent log_level:=debug
```

**Features:**
- Subscribes to `/cognitive/sense_data`
- Publishes to `/cognitive/memory_data`
- Provides `/cognitive/memory_request` service
- Manages short-term, long-term, and working memory

#### 3. Action Node with LLM Only
```bash
# Launch just the action node with LLM integration
ros2 launch cognitive_framework action_node_with_llm.launch.py

# With custom parameters
ros2 launch cognitive_framework action_node_with_llm.launch.py namespace:=my_agent log_level:=debug
```

**Features:**
- Subscribes to `/cognitive/sense_data` and `/cognitive/memory_data`
- Publishes to `/cognitive/action_data`
- Enhanced decision making with LLM integration
- Strategic reasoning and goal creation

### Complete System Launches

#### 4. Complete System (Original)
```bash
# Launch all three nodes with standard action node
ros2 launch cognitive_framework cognitive_system.launch.py
```

#### 5. Complete System with LLM
```bash
# Launch all three nodes with LLM-enhanced action node
ros2 launch cognitive_framework cognitive_system_with_llm.launch.py

# With custom parameters
ros2 launch cognitive_framework cognitive_system_with_llm.launch.py namespace:=my_agent log_level:=debug
```

## Development Launch Methods

### Alternative Launch Methods (No launch_ros Required)

If you're in a development environment without the full ROS2 setup, you can use these alternative methods:

#### 1. Simple Node Launcher
```bash
# Launch individual nodes
python3 launch_nodes.py sense_node --log-level debug
python3 launch_nodes.py memory_node --log-level debug
python3 launch_nodes.py action_node_with_llm --log-level debug

# With custom namespace
python3 launch_nodes.py sense_node --namespace my_agent --log-level debug
```

#### 2. Simple Multi-Node Launcher
```bash
# Launch all nodes
python3 launch/simple_launch.py

# Launch specific nodes
python3 launch/simple_launch.py --nodes sense_node memory_node

# With custom parameters
python3 launch/simple_launch.py --log-level debug --namespace my_agent
```

### Standard ROS2 Launch Methods

#### 3. Individual Node Launches
Use individual node launches when:
- **Debugging specific components** - Launch only the node you're working on
- **Testing node isolation** - Verify each node works independently
- **Resource optimization** - Run only what you need for testing

### Production Deployment
Use complete system launches when:
- **Full cognitive system** - Need all three nodes working together
- **LLM integration** - Want enhanced decision making capabilities
- **Standard operation** - Normal cognitive framework operation

## Launch Parameters

All launch files support these parameters:

### `log_level`
- **Default**: `info`
- **Options**: `debug`, `info`, `warn`, `error`, `fatal`
- **Usage**: Controls verbosity of node logging

### `namespace`
- **Default**: `cognitive_system`
- **Usage**: Sets the ROS2 namespace for all topics and services
- **Example**: `namespace:=my_agent` creates topics like `/my_agent/cognitive/sense_data`

## Topic Mapping

When using custom namespaces, topics are automatically mapped:

| Default Topic | With `namespace:=my_agent` |
|---------------|---------------------------|
| `/cognitive/sense_data` | `/my_agent/cognitive/sense_data` |
| `/cognitive/memory_data` | `/my_agent/cognitive/memory_data` |
| `/cognitive/action_data` | `/my_agent/cognitive/action_data` |
| `/cognitive/memory_request` | `/my_agent/cognitive/memory_request` |

## Examples

### Basic Testing
```bash
# Test sense node only
ros2 launch cognitive_framework sense_node.launch.py log_level:=debug

# In another terminal, monitor the output
ros2 topic echo /cognitive/sense_data
```

### Multi-Agent Setup
```bash
# Launch agent 1
ros2 launch cognitive_framework cognitive_system_with_llm.launch.py namespace:=agent1

# Launch agent 2 (in another terminal)
ros2 launch cognitive_framework cognitive_system_with_llm.launch.py namespace:=agent2
```

### Development Workflow
```bash
# 1. Start memory node for testing
ros2 launch cognitive_framework memory_node.launch.py log_level:=debug

# 2. In another terminal, test memory service
ros2 service call /cognitive/memory_request cognitive_framework/srv/MemoryRequest "{memory_type: 'short_term', num_entries: 5}"

# 3. Start sense node to generate data
ros2 launch cognitive_framework sense_node.launch.py

# 4. Monitor memory processing
ros2 topic echo /cognitive/memory_data
```

## Troubleshooting

### Import Errors (Development Environment)
If you get import errors like "Import 'launch_ros.actions' could not be resolved":

1. **Install missing dependencies**:
   ```bash
   pip install launch launch_ros
   ```

2. **Use alternative launch methods** (see Development Launch Methods below)

3. **Source ROS2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash  # or your ROS2 version
   ```

### Node Not Found
If you get "executable not found" errors:
1. **Rebuild the package**:
   ```bash
   colcon build --packages-select cognitive_framework
   source install/setup.bash
   ```

2. **Check entry points**:
   ```bash
   ros2 pkg executables cognitive_framework
   ```

### Topics Not Connecting
If nodes aren't communicating:
1. **Check namespace consistency** - All nodes must use the same namespace
2. **Verify topic names**:
   ```bash
   ros2 topic list
   ros2 topic info /cognitive/sense_data
   ```

### LLM Integration Issues
If the LLM-enhanced action node isn't working:
1. **Check LLM service** - Ensure LLM service is running
2. **Verify dependencies** - Check that `llm_client.py` is properly configured
3. **Review logs** - Use `log_level:=debug` for detailed error messages

## Next Steps

After setting up the launch configurations:

1. **Test individual nodes** - Verify each component works independently
2. **Test complete system** - Ensure all nodes communicate properly
3. **Configure LLM integration** - Set up LLM service for enhanced decision making
4. **Customize behavior** - Modify node parameters and logic as needed 