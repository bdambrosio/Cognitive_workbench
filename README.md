# Cognitive workbench

My new playground for cognitive AI experiments.
A simplified cognitive framework using Zenoh for communication, replacing ROS2 complexity with clean Python applications that can run on separate cores.
Most of the below is out of date, I'll try to update in the next day or so. 
First level functionality.
Multiple characters can be defined in a simple 2 1/2 grid world. you define personality and drives. Each then creates goals, implements plans (simple scripts with control flow - if/then/else and do_while), maintains beliefs and TOM models of others, etc.

System even has a minimal UI that allows stepping, manual input of simple plans, etc.

## 🚀 Quick Start

### 1. Setup Environment

```bash
# Run the setup script
./zenoh_cognitive_framework/setup.sh
```

Or manually:

```bash
# Create virtual environment
python3 -m venv zenoh_venv

# Activate virtual environment
source zenoh_venv/bin/activate

# Install dependencies
pip install -r zenoh_cognitive_framework/requirements.txt
```

### 2. Launch the System

```bash
# Activate virtual environment (if not already active)
source zenoh_venv/bin/activate

# Launch all nodes
python zenoh_cognitive_framework/launch_all_nodes.py
```

### 3. Use the System

1. Type text input in the Action Display Node terminal
2. Watch the cognitive loop process your input
3. See responses and actions displayed

## 📁 Project Structure

```
ros_cognitive_framework/
├── zenoh_cognitive_framework/     # Main framework code
│   ├── sense_node.py              # Sensory input processing
│   ├── memory_node.py             # Memory storage and retrieval
│   ├── llm_service_node.py        # LLM API service
│   ├── action_node.py # Complete cognitive loop
│   ├── action_display_node.py     # Action display and input
│   ├── llm_client.py              # LLM client interface
│   ├── launch_all_nodes.py        # Multi-process launcher
│   ├── test_zenoh_installation.py # Installation verification
│   ├── setup.sh                   # Setup script
│   ├── requirements.txt           # Dependencies
│   └── README.md                  # Detailed documentation
├── zenoh_venv/                    # Python virtual environment
└── README.md                      # This file
```

## 🎯 Features

- **Multi-core performance**: Each node runs as a separate Python process
- **Simple communication**: Zenoh pub/sub instead of ROS2 topics/services
- **Built-in storage**: Zenoh's storage capabilities (when configured)
- **Easy deployment**: Just Python processes, no complex configuration
- **Fault tolerance**: Automatic reconnection and recovery
- **Standard Python**: No ROS2 dependencies or build system

## 🔧 Architecture

The framework consists of these nodes, each running as a separate process:

1. **Memory Node** - Provides persistent storage using Zenoh's built-in storage
2. **LLM Service Node** - Provides LLM API access via Zenoh pub/sub
3. **Sense Node** - Simulates sensory input and publishes perception data
4. **Single LLM Action Example** - Demonstrates complete cognitive loop
5. **Action Display Node** - Displays incoming actions and provides text input

## 📚 Documentation

For detailed documentation, see [zenoh_cognitive_framework/README.md](zenoh_cognitive_framework/README.md).

## 🔄 Migration from ROS2

This framework replaces the complex ROS2 setup with a simple Zenoh-based solution:

| Feature | ROS2 | Zenoh |
|---------|------|-------|
| Communication | Topics/Services | Pub/Sub |
| Storage | Separate services | Built-in |
| Configuration | Launch files | Python code |
| Dependencies | Complex | Simple |
| Deployment | ROS2 workspace | Python processes |

## 🛠️ Development

### Adding New Nodes

1. Create a new Python file in `zenoh_cognitive_framework/`
2. Import Zenoh and initialize session with config
3. Declare publishers/subscribers
4. Implement your logic
5. Add to `launch_all_nodes.py` if needed

### Testing

```bash
# Test Zenoh installation
python zenoh_cognitive_framework/test_zenoh_installation.py

# Launch individual nodes for testing
cd zenoh_cognitive_framework
python memory_node.py
python llm_service_node.py
# etc.
```

## 📝 License

This framework is provided as-is for educational and research purposes.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request 
