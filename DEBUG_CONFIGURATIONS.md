# IDE Debug Configurations for Cognitive Framework

This document describes the VS Code/Cursor debug configurations for the ROS2 Cognitive Framework.

## 🚀 Quick Start

1. **Open the project** in VS Code/Cursor
2. **Select Python interpreter** - Choose `./ros_venv/bin/python3`
3. **Set breakpoints** in your code
4. **Press F5** or use the Run and Debug panel
5. **Select a debug configuration** from the dropdown

## 📋 Available Debug Configurations

### Individual Node Debugging

#### **Debug Sense Node**
- **Purpose**: Debug the sense node in isolation
- **File**: `cognitive_framework/sense_node.py`
- **Features**: Console input, sensor simulation, topic publishing
- **Use Case**: Test sensory input processing and console interaction

#### **Debug Memory Node**
- **Purpose**: Debug the memory node in isolation
- **File**: `cognitive_framework/memory_node.py`
- **Features**: Memory consolidation, importance calculation, service interface
- **Use Case**: Test memory processing and storage logic

#### **Debug Action Node with LLM**
- **Purpose**: Debug the enhanced action node with LLM integration
- **File**: `cognitive_framework/action_node_with_llm.py`
- **Features**: LLM-guided decision making, strategic reasoning, goal creation
- **Use Case**: Test LLM integration and enhanced decision making

#### **Debug Action Node (Standard)**
- **Purpose**: Debug the standard action node without LLM
- **File**: `cognitive_framework/action_node.py`
- **Features**: Basic decision making, reactive actions, goal pursuit
- **Use Case**: Test basic cognitive decision making

#### **Debug LLM Service Node**
- **Purpose**: Debug the LLM service interface
- **File**: `cognitive_framework/llm_service_node.py`
- **Features**: LLM client communication, request handling
- **Use Case**: Test LLM service integration

#### **Debug Action Display Node**
- **Purpose**: Debug the action display interface
- **File**: `cognitive_framework/action_display_node.py`
- **Features**: Action visualization, user interface
- **Use Case**: Test action display and user interaction

### System-Level Debugging

#### **Debug Simple Launch (All Nodes)**
- **Purpose**: Debug all nodes running together
- **File**: `launch/simple_launch.py`
- **Features**: Multi-node coordination, system integration
- **Use Case**: Test complete system behavior

#### **Debug Simple Launch (Sense + Memory)**
- **Purpose**: Debug sense and memory nodes together
- **File**: `launch/simple_launch.py`
- **Args**: `--nodes sense_node memory_node`
- **Use Case**: Test sensory-memory pipeline

#### **Debug Individual Node Launcher**
- **Purpose**: Debug the node launcher utility
- **File**: `launch_nodes.py`
- **Args**: `sense_node --log-level debug`
- **Use Case**: Test node launching infrastructure

### Testing and Validation

#### **Debug Test Console Input**
- **Purpose**: Debug console input testing
- **File**: `test_console_input.py`
- **Use Case**: Test console input functionality

#### **Debug Test Cognitive System**
- **Purpose**: Debug system health checks
- **File**: `test_cognitive_system.py`
- **Use Case**: Test system integration and health

## 🛠️ Available Tasks

### Build Tasks
- **setup-environment**: Activates virtual environment and sources ROS2
- **build-package**: Builds the cognitive_framework package
- **clean-build**: Removes build artifacts
- **rebuild-package**: Clean build of the package

### Test Tasks
- **test-sense-node**: Shows sense node launch arguments
- **test-memory-node**: Shows memory node launch arguments
- **test-action-node-with-llm**: Shows action node with LLM launch arguments
- **test-complete-system**: Shows complete system launch arguments

## 🔧 Configuration Details

### Environment Setup
Each debug configuration automatically:
1. **Activates virtual environment** (`ros_venv`)
2. **Sources ROS2 Jazzy** (`/opt/ros/jazzy/setup.bash`)
3. **Sources workspace** (`install/setup.bash`)
4. **Sets debug log level** (`ROS_LOG_LEVEL=debug`)

### Debug Settings
- **Console**: Integrated terminal for interactive debugging
- **Working Directory**: Project root
- **Arguments**: ROS2 arguments for namespace and log level
- **Environment**: Debug log level enabled

### Python Configuration
- **Interpreter**: `./ros_venv/bin/python3`
- **Linting**: flake8 with custom rules
- **Formatting**: black with 120 character line length
- **File Associations**: Launch files and ROS2 message files

## 🎯 Usage Examples

### Debugging Individual Components

1. **Set breakpoint** in `sense_node.py` line 100
2. **Select "Debug Sense Node"** configuration
3. **Press F5** to start debugging
4. **Type console input** to trigger breakpoint
5. **Inspect variables** and step through code

### Debugging System Integration

1. **Set breakpoints** in multiple nodes
2. **Select "Debug Simple Launch (All Nodes)"**
3. **Press F5** to start system debugging
4. **Monitor topic communication** between nodes
5. **Debug cross-node interactions**

### Testing Launch Configurations

1. **Open Command Palette** (Ctrl+Shift+P)
2. **Run "Tasks: Run Task"**
3. **Select test task** (e.g., "test-sense-node")
4. **Review launch arguments** and configuration

## 🔍 Debugging Tips

### Breakpoint Strategy
- **Entry points**: Set breakpoints in `main()` functions
- **Callbacks**: Set breakpoints in topic callback functions
- **Decision points**: Set breakpoints in decision-making logic
- **Error handling**: Set breakpoints in exception handlers

### Variable Inspection
- **ROS2 messages**: Inspect message contents and structure
- **Memory state**: Check short-term and long-term memory
- **Action history**: Review action selection and execution
- **LLM responses**: Examine LLM integration data

### Console Interaction
- **Sense node**: Type console input to test text processing
- **Action display**: Monitor action visualization
- **Memory service**: Use service calls to query memory
- **Topic monitoring**: Use `ros2 topic echo` in separate terminal

## 🚨 Troubleshooting

### Common Issues

#### **Import Errors**
- **Solution**: Ensure virtual environment is activated
- **Check**: Python interpreter path in settings
- **Verify**: All dependencies are installed

#### **ROS2 Not Found**
- **Solution**: Source ROS2 environment
- **Check**: `ROS_DISTRO` environment variable
- **Verify**: ROS2 installation path

#### **Breakpoints Not Hit**
- **Solution**: Check if correct configuration is selected
- **Verify**: Code is actually running (add print statements)
- **Check**: Console output for errors

#### **Permission Errors**
- **Solution**: Make setup script executable
- **Command**: `chmod +x setup_environment.sh`

### Environment Verification

```bash
# Check Python interpreter
which python3

# Check ROS2 environment
echo $ROS_DISTRO

# Check virtual environment
echo $VIRTUAL_ENV

# Test imports
python3 -c "import rclpy; import launch_ros; print('OK')"
```

## 📚 Next Steps

1. **Set up breakpoints** in your code
2. **Try debugging** individual nodes first
3. **Test system integration** with multiple nodes
4. **Explore LLM integration** debugging
5. **Customize configurations** for your specific needs

Happy debugging! 🐛✨ 