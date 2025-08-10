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
The main panel lists actions characters perform and the results.
The left panel allows inspection. Click on a character name and 
- its current goal will be displayed above the main panel
- below in the left panel are a set of tabs:
  - plan displays its current_plan
  - view displays its current report from the map
  - this will be expanded to show inventory, TOM, plan history and analysis, ... work in progress
The bottom, in addition to the control buttons, contains an entry area.
- A small entry area for a character name
- A larger text area for chatting with that character. type text and press send. The named character will usually answer.
- You can also start with goal: or plan: to give the character a new goal or a plan. The plan text format is a bit fragile.
Characters run in parallel. 'Step' gives every character a single turn. Run is just a loop calling step repeatedly.
Condition testing for an if or while does not count as a step. 

Shutdown button or ^C on console shuts everything down (15-20 secs, be patient).

## 📁 Project Structure

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/bdambrosio/Cognitive_workbench)

Rather out of date, work in progress
```
Cognitive_workbench/
├── src/     # Main framework code
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

- **Multi-core performance**: Each node runs as multiple Python processes each supporting a zenoh node
- **Simple communication**: Zenoh pub/sub and queryables (simple? hmm)
- **Built-in storage**: Zenoh's storage capabilities (when configured)
- **Easy deployment**: Just Python processes, no complex configuration
- **Fault tolerance**: Automatic reconnection and recovery
- **Standard Python**: except for HTMX/js ui

## 🔧 Architecture

The framework consists of these nodes, each running as a separate process:

1. **Fastapi_action_display Node** - Shared - horrible name for UI
2. **Map Node** - Shared - simple 2D world
3. **LLM Service Node** - Shared - Provides LLM API access via Zenoh pub/sub - uses futures, but actually not needed since vllm / openai / openrouter can all handle simultaneous requests. refactoring needed
4. **Executive Node** - per character, overall orchestration for a character.
5. **Memory Node** - per character, provides memory services and persistent storage using json format files
6. **Situation node** - per character, manages current external environment interface.
7. **Sense Node** - per character - handles sensory input and publishes perception data - mostly a stub right now.

## 📚 Documentation



## 📝 License

This framework is provided as-is for educational and research purposes.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request 
