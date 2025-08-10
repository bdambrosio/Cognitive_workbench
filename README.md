# Cognitive workbench

My new playground for cognitive AI experiments.
A simplified cognitive framework using Zenoh for parallelism and communication.
Barest of functionality, this is the initial skeleton, tested enough I believe it is worth building on.
Multiple characters can be defined in a simple 2 1/2 grid world. you define personality and drives. Each then creates goals, implements plans (simple scripts with control flow - if/then/else and while), maintains beliefs and TOM models of others, etc.

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

Word of caution: debugging is hard because if you set a breakpoint a character will time out and be deemed unresponsive, and dropped from turn manager in launcher.py. Should pbly add a debug mode to launch that disables that, but there are timeouts in zenoh queryables too. tbd. Ideas welcome.

## 📁 Project Structure

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/bdambrosio/Cognitive_workbench)


## 🎯 Features

- **Multi-core performance**: Each character is modeled as multiple zenoh nodes. Each zenoh node runs as a separate Python job
- **Simple communication**: Zenoh pub/sub and queryables (simple? hmm)
- **Built-in storage**: json / files, maybe migrate to Zenoh's storage capabilities in future.
- **Easy deployment**: Just Python processes, no complex configuration (tried ROS2 earlier, bad idea)
- **Fault tolerance**: Automatic reconnection and recovery
- **Standard Python**: except for HTMX/js ui (may move this to React later to better support modularity as UI complexity increases)

## 🔧 Architecture

The workbench consists of these nodes, each running as a separate process:

1. **Fastapi_action_display Node** - Shared - horrible name for UI
2. **Map Node** - Shared - simple 2D world
3. **LLM Service Node** - Shared - Provides LLM API access via Zenoh pub/sub - uses futures, but actually not needed since vllm / openai / openrouter can all handle simultaneous requests. refactoring needed
4. **Executive Node** - per character, overall orchestration for a character.
5. **Memory Node** - per character, provides memory services and persistent storage using json format files. (aside - 'memory', what a simple word for the heart of cognition.
6. **Situation node** - per character, manages current external environment interface.
7. **Sense Node** - per character - handles sensory input and publishes perception data - mostly a stub right now, this, like memory, is where most of the complexity lies.

## 📚 Documentation

Ain't none yet but what you see here. 

## 📝 License

This framework is provided as-is for educational and research purposes.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request 
