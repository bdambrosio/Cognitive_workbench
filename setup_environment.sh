#!/bin/bash
# Setup script for ROS2 Cognitive Framework with Virtual Environment

echo "🧠 Setting up ROS2 Cognitive Framework Environment"
echo "=================================================="

# Check if virtual environment exists
if [ ! -d "ros_venv" ]; then
    echo "❌ Virtual environment not found. Creating one..."
    python3 -m venv ros_venv
    source ros_venv/bin/activate
    
    echo "📦 Installing required packages..."
    pip install empy catkin_pkg numpy setuptools launch launch_ros rclpy std_msgs geometry_msgs requests sympy
else
    echo "✅ Virtual environment found. Activating..."
    source ros_venv/bin/activate
fi

# Source ROS2 environment
echo "🔧 Sourcing ROS2 Jazzy environment..."
source /opt/ros/jazzy/setup.bash

# Source the workspace if it exists
if [ -f "install/setup.bash" ]; then
    echo "📁 Sourcing workspace..."
    source install/setup.bash
    echo "✅ Workspace sourced successfully!"
else
    echo "⚠️  Workspace not built yet. Run 'colcon build --packages-select cognitive_framework --cmake-args -DPYTHON_EXECUTABLE=\$(which python3)' to build."
fi

echo ""
echo "🚀 Environment ready! Available commands:"
echo "  • ros2 launch cognitive_framework sense_node.launch.py"
echo "  • ros2 launch cognitive_framework memory_node.launch.py"
echo "  • ros2 launch cognitive_framework action_node_with_llm.launch.py"
echo "  • ros2 launch cognitive_framework cognitive_system_with_llm.launch.py"
echo "  • python3 launch_nodes.py sense_node --log-level debug"
echo "  • python3 launch/simple_launch.py"
echo ""
echo "💡 To activate this environment in a new terminal, run:"
echo "   source setup_environment.sh" 