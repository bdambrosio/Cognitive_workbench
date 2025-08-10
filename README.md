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
mkdir Cog
cd Cog
# Create virtual environment
python3 -m venv zenoh_venv

# Activate virtual environment
source zenoh_venv/bin/activate

# Install dependencies
pip install -r zenoh_cognitive_framework/requirements.txt
```

### 2. Launch the System

First, be sure you edit the scenario you want to run to use the proper llm.
Scenarios are in Cognitive_workbench/scenarios (but maps are in src/maps, I should fix that)

The system supports OpenAI and OpenRouter as well as a local option on port 5000
scenarios are in yaml format, the first few lines look like:

```yaml
map: forest.py
# Cognitive Workbench Configuration
llm_config:
  server_name: "vllm"
  model_name: "qwen/qwen3-235b-a22b-2507"
```

model doesn't actually matter for vllm, it will get model from server

```bash
# Activate virtual environment (if not already active)
source zenoh_venv/bin/activate

# Launch all nodes
cd src
python3 launcher.py lost.yaml --ui
# watch for 'Reuse existing world' message (if it finds saves for this scenario). probably safest to answer no, this often fails at the moment.
```

### 3. Use the System

A browser tab should open (localhost:3000) with a minimal UI.
Characters run in parallel. 'Step' gives every character a single turn. Run is just a loop calling step repeatedly.
Condition testing for an if or while does not count as a step. 

Shutdown button or ^C on console shuts everything down (15-20 secs, be patient).

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
