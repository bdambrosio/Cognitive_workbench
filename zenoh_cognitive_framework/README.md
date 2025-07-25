# Zenoh Cognitive Framework

A simplified cognitive framework using Zenoh for communication, replacing ROS2 complexity with clean Python applications that can run on separate cores.

## 🚀 Features

- **Multi-core performance**: Each node runs as a separate Python process
- **Simple communication**: Zenoh pub/sub instead of ROS2 topics/services
- **Built-in storage**: Zenoh's storage capabilities replace separate memory services
- **Easy deployment**: Just Python processes, no complex configuration
- **Fault tolerance**: Automatic reconnection and recovery
- **Standard Python**: No ROS2 dependencies or build system

## 📦 Installation

1. **Install Zenoh Python**:
   ```bash
   pip install zenoh-python
   ```

2. **Clone the framework**:
   ```bash
   cd zenoh_cognitive_framework
   ```

## 🏗️ Architecture

The framework consists of these nodes, each running as a separate process:

### Core Nodes

1. **Memory Node** (`memory_node.py`)
   - Provides persistent storage using Zenoh's built-in storage
   - Handles short-term, working, and long-term memory
   - Automatic cleanup and management

2. **LLM Service Node** (`llm_service_node.py`)
   - Provides LLM API access via Zenoh pub/sub
   - Concurrent request handling with thread pool
   - Automatic fallback to mock responses if LLM unavailable

3. **Sense Node** (`sense_node.py`)
   - Simulates sensory input and publishes perception data
   - Handles console text input
   - Publishes structured sense data

4. **Action Node** (`action_node.py`)
   - Demonstrates complete cognitive loop
   - Receives sense data → makes LLM call → publishes action → stores in memory
   - Uses memory context for better responses

5. **Action Display Node** (`action_display_node.py`)
   - Displays incoming actions in formatted output
   - Provides text input interface
   - Maintains action history and statistics

### Communication Flow

```
Sense Node → cognitive/sense_data
    ↓
Single LLM Action → cognitive/llm_request → LLM Service
    ↓                                    ↓
Action Display ← cognitive/action    cognitive/llm_response
    ↓
Memory Node ← cognitive/memory/store
```

## 🚀 Quick Start

### Option 1: Launch All Nodes (Recommended)

```bash
python launch_all_nodes.py
```

This will:
- Start all nodes in the correct order
- Each node runs on a separate process
- Monitor and restart nodes if they crash
- Graceful shutdown on Ctrl+C

### Option 2: Launch Individual Nodes

Open separate terminals and run:

```bash
# Terminal 1: Memory Node
python memory_node.py

# Terminal 2: LLM Service Node  
python llm_service_node.py

# Terminal 3: Sense Node
python sense_node.py

# Terminal 4: Single LLM Action Example
python action_node.py

# Terminal 5: Action Display Node
python action_display_node.py
```

## 💬 Usage

1. **Start the system** using one of the methods above
2. **Type text input** in the Action Display Node terminal
3. **Watch the cognitive loop**:
   - Sense Node receives input
   - Single LLM Action makes LLM call
   - Action Display shows the response
   - Everything is stored in memory

### Example Session

```
🎯 ACTION #1 [14:30:15]
   ID: action_0
   Type: cognitive_response
   Input: "What is the weather like?"
   Response: Based on the current information available, I cannot provide real-time weather data...
   Confidence: 0.80
   Model: zenoh_llm, Time: 0.523s, Memory: 2 entries
```

## 🔧 Configuration

### Zenoh Configuration

The framework uses Zenoh's default configuration. For custom settings:

```python
import zenoh

# Custom configuration
config = zenoh.Config()
config.insert_json5("mode", '"peer"')
config.insert_json5("connect/endpoints", '["tcp/192.168.1.100:7447"]')

session = zenoh.open(config)
```

### Node Configuration

Each node can be configured by modifying the Python files:

- **Memory limits**: Modify `max_short_term_entries` in `memory_node.py`
- **LLM parameters**: Adjust `max_tokens`, `temperature` in `llm_client.py`
- **Processing timeouts**: Change `service_timeout` in various nodes

## 🧠 Memory System

The memory system provides three types of storage:

### Short-term Memory
- Recent events and interactions
- Automatic cleanup (keeps last 1000 entries)
- Queryable via `cognitive/memory/short_term/*`

### Working Memory
- Current context and active information
- Persistent during session
- Queryable via `cognitive/memory/working/*`

### Long-term Memory
- Persistent knowledge and learned information
- Manual cleanup only
- Queryable via `cognitive/memory/long_term/*`

### Memory Queries

```python
# Get recent short-term memories
for reply in session.get("cognitive/memory/short_term/*"):
    content = json.loads(reply.payload.decode('utf-8'))
    print(content)

# Store new memory entry
memory_data = {
    'memory_type': 'short_term',
    'key': 'my_entry',
    'content': {'text': 'Hello world', 'timestamp': '2024-01-01T12:00:00'}
}
session.put("cognitive/memory/store", json.dumps(memory_data))
```

## 🔍 Troubleshooting

### Common Issues

1. **Zenoh not found**:
   ```bash
   pip install zenoh-python
   ```

2. **LLM service unavailable**:
   - Check if LLM API is properly configured
   - Service will use mock responses as fallback

3. **Nodes not communicating**:
   - Ensure all nodes are running
   - Check Zenoh network configuration
   - Verify topic names match

4. **Memory queries failing**:
   - Ensure memory node is running
   - Check memory node logs for errors

### Debug Mode

Run individual nodes with verbose logging:

```bash
python -u memory_node.py  # -u for unbuffered output
```

## 🏗️ Extending the Framework

### Adding New Nodes

1. Create a new Python file
2. Import Zenoh and initialize session
3. Declare publishers/subscribers
4. Implement your logic
5. Add to `launch_all_nodes.py` if needed

Example:
```python
import zenoh
import json

class MyNode:
    def __init__(self):
        self.session = zenoh.open()
        self.publisher = self.session.declare_publisher("my/topic")
        self.subscriber = self.session.declare_subscriber("other/topic", self.callback)
    
    def callback(self, sample):
        data = json.loads(sample.payload.decode('utf-8'))
        # Process data
        self.publisher.put(json.dumps(result))
    
    def run(self):
        while True:
            time.sleep(1)

if __name__ == '__main__':
    node = MyNode()
    node.run()
```

### Custom LLM Integration

Modify `llm_service_node.py` to integrate with your LLM:

```python
# Replace the LLM import with your implementation
from my_llm import MyLLM

# Initialize your LLM
self.llm = MyLLM()

# Use your LLM in _process_llm_request
response_text = self.llm.generate(messages, **params)
```

## 📊 Performance

### Multi-core Benefits

- Each node runs on a dedicated CPU core
- No GIL contention between processes
- Better resource isolation
- Independent scaling

### Monitoring

The launcher provides basic monitoring:
- Process status and PIDs
- Automatic restart on failure
- Graceful shutdown

### Optimization Tips

1. **CPU Affinity**: Set specific cores for each node
2. **Memory Limits**: Adjust memory cleanup intervals
3. **Network**: Use local Zenoh for better performance
4. **Threading**: Use thread pools for I/O operations

## 🔄 Migration from ROS2

### Key Differences

| Feature | ROS2 | Zenoh |
|---------|------|-------|
| Communication | Topics/Services | Pub/Sub |
| Storage | Separate services | Built-in |
| Configuration | Launch files | Python code |
| Dependencies | Complex | Simple |
| Deployment | ROS2 workspace | Python processes |

### Migration Steps

1. Replace ROS2 imports with Zenoh
2. Convert topics to Zenoh pub/sub
3. Replace services with Zenoh queries
4. Remove ROS2 configuration files
5. Update launcher scripts

## 📝 License

This framework is provided as-is for educational and research purposes.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📚 Resources

- [Zenoh Documentation](https://zenoh.io/docs/)
- [Zenoh Python API](https://github.com/eclipse-zenoh/zenoh-python)
- [Cognitive Architecture Patterns](https://en.wikipedia.org/wiki/Cognitive_architecture) 