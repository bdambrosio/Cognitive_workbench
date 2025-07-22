# 🎨 Action Display Node Guide

The Action Display Node provides a beautiful, real-time terminal interface to monitor your cognitive framework's actions and responses.

## ✨ Features

- **🎤 Text Input Tracking**: Shows when you type in the sense node
- **🧠 LLM Response Display**: Real-time LLM processing and responses
- **🤖 Action Monitoring**: All cognitive actions with confidence scores
- **📊 Live Statistics**: Total actions, LLM calls, uptime
- **🌈 Color-coded Interface**: Different colors for different event types
- **📜 Scrolling History**: Automatic scrolling display of recent events

## 🚀 Quick Start

### Option 1: Individual Node (for debugging)
```bash
# Press F5 in VSCode
# Choose: "Debug Action Display"
```

### Option 2: Full System (recommended)
```bash
# Press F5 in VSCode  
# Choose: "Full System with Rich Display"
```

This will start:
- **SENSE NODE** - Type here for input
- **LLM SERVICE** - Processes requests
- **SIMPLE LLM ACTION** - Generates actions
- **ACTION DISPLAY** - Beautiful monitoring

## 🎯 How to Use

1. **Start the full system** using "Full System with Rich Display"
2. **Wait for all terminals** to initialize (4 terminals will appear)
3. **Find the Action Display terminal** - it will show colorful output
4. **Look for input prompts**: `💬 Enter your message:`
5. **Type your message** when prompted and press Enter
6. **Watch the flow** appear in real-time:
   ```
   🎤 Input: "hello"
   🧠 LLM: Hello! How can I help you today?
   🤖 Action: Generated response (Conf: 0.85)
   ```
7. **Continue typing** - new prompts will appear automatically

## 📱 Display Layout (VSCode-Friendly with Full Text)

```
🎨 Action Display Started!
💬 The input thread will prompt you for messages
============================================================

📊 Stats: Total: 3 | LLM: 2 | Uptime: 00h 02m
📝 Recent Actions:
  14:23:15 🎤 Input:
    "hello"

  14:23:16 🧠 LLM Response (ID: req_1):
    Hello! How can I help you today? I'm here to assist with any 
    questions or tasks you might have. Feel free to ask me about 
    anything you'd like to know more about.

  14:23:17 🤖 Action: llm_guided_action (Conf: 0.85)
    Summary: Generated friendly greeting response
    Response Type: conversational
    Intent: greeting_acknowledgment

────────────────────────────────────────────────────────────

💬 Enter your message: _

✅ Sent:
    "hello"

💬 Enter your message: _
```

## 🎨 Color Coding & Full Text Display

- **🎤 Text Input**: Cyan headers with italic text content
- **🧠 LLM Responses**: Yellow headers with full response text wrapped at 100 characters
- **🤖 Actions**: Green headers with complete action details and structured content
- **Confidence Scores**:
  - 🟢 Green: High confidence (> 0.8)
  - 🟡 Yellow: Medium confidence (0.5 - 0.8)
  - 🔴 Red: Low confidence (< 0.5)

### **New Full-Text Features:**
- ✅ **No truncation** - All responses shown in full
- ✅ **Automatic wrapping** - Long text wraps at 100 characters for readability
- ✅ **Structured display** - Action content shows all fields (summary, type, intent, etc.)
- ✅ **Paragraph handling** - Multi-line responses preserve formatting
- ✅ **Visual separation** - Clear separators between entries

## 🔧 Terminal Management in VSCode

When you run "Full System with Rich Display", you'll see 4 terminal tabs:

1. **SENSE NODE** - Shows sensor processing logs
2. **LLM SERVICE** - Shows LLM processing logs  
3. **SIMPLE LLM ACTION** - Shows action generation logs
4. **ACTION DISPLAY** ← Type AND watch here! (Main terminal)

## 📊 Statistics Tracked

- **Total Actions**: All actions generated
- **LLM Actions**: Actions involving LLM processing
- **Sensor Actions**: Actions from sensor input
- **Uptime**: How long the system has been running

## 🛠️ Troubleshooting

### No Display Appears
- Check that `rich` is installed: `pip install rich`
- Ensure you're using the owl virtual environment
- Look for error messages in the ACTION DISPLAY terminal

### No Actions Shown
- Make sure all 4 nodes are running
- Wait for input prompt: `💬 Enter your message:`
- Type a message when prompted (in the Action Display terminal)
- Check that topics are connected: other terminals should show activity

### Rich Library Error
```bash
# Install rich in the owl environment
source src/owl/bin/activate
pip install rich
```

## 🎯 Expected Flow

1. **Wait for the input prompt**: `💬 Enter your message:`
2. **Type "hello" and press Enter**
3. **See confirmation with full text**:
   ```
   ✅ Sent:
       "hello"
   ```
4. **Watch the complete flow appear with full responses**:
   ```
   📊 Stats: Total: 1 | LLM: 1 | Uptime: 00h 01m
   📝 Recent Actions:
     14:30:15 🎤 Input:
       "hello"

     14:30:16 🧠 LLM Response (ID: req_1):
       Hello! How can I help you today? I'm here to assist with any 
       questions or tasks you might have. Feel free to ask me about 
       anything you'd like to know more about.

     14:30:17 🤖 Action: llm_guided_action (Conf: 0.85)
       Summary: Generated friendly greeting response
       Response Type: conversational
       Intent: greeting_acknowledgment
   ```
5. **Next input prompt appears**: `💬 Enter your message:`
6. **Continue the conversation** - all responses shown in full!

## 🚀 Next Steps

- Try different types of input to see various action types
- Observe confidence scores for different prompts
- Use this for debugging your cognitive framework
- Extend the display to show additional data types

The Action Display makes it easy to understand what your cognitive system is thinking and doing in real-time! 🧠✨ 