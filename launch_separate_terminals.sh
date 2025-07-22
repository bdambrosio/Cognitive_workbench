#!/bin/bash
# Launch Cognitive Framework Nodes in Separate Terminals
# Usage: ./launch_separate_terminals.sh [log_level] [namespace]

LOG_LEVEL=${1:-info}
NAMESPACE=${2:-cognitive_system}

echo "🧠 Launching Cognitive Framework in Separate Terminals"
echo "======================================================"
echo "Log Level: $LOG_LEVEL"
echo "Namespace: $NAMESPACE"
echo ""

# Source the environment
source install/setup.bash

# Function to launch a node in a new terminal
launch_node() {
    local node_name=$1
    local executable=$2
    local title=$3
    
    echo "🚀 Launching $title in new terminal..."
    gnome-terminal --title="$title" -- bash -c "
        echo '🧠 $title'
        echo '================'
        source install/setup.bash
        ros2 run cognitive_framework $executable --ros-args --log-level $LOG_LEVEL --ros-args -r __ns:=/$NAMESPACE
        echo ''
        echo 'Press Enter to close this terminal...'
        read
    "
    sleep 1
}

# Launch each node in separate terminals
launch_node "sense_node" "sense_node" "Sense Node - Perception"
launch_node "memory_node" "memory_node" "Memory Node - Storage"
launch_node "action_node_with_llm" "action_node_with_llm" "Action Node - LLM Decision Making"

echo ""
echo "✅ All nodes launched in separate terminals!"
echo ""
echo "📋 Available commands:"
echo "  • ros2 topic list                    # List all topics"
echo "  • ros2 topic echo /cognitive_system/sense_data    # Monitor sense data"
echo "  • ros2 topic echo /cognitive_system/memory_data   # Monitor memory data"
echo "  • ros2 topic echo /cognitive_system/action_data   # Monitor action data"
echo "  • ros2 node list                     # List running nodes"
echo "  • ros2 service list                  # List available services"
echo ""
echo "💡 Tips:"
echo "  • Type in the Sense Node terminal to send text input"
echo "  • Each terminal shows logs from its respective node"
echo "  • Close any terminal to stop that node"
echo "" 