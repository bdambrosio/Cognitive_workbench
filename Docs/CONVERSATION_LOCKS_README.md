# Conversation Lock System

## Overview

The conversation lock system prevents characters from talking over each other by implementing a per-character lock mechanism managed by the `map_node`. When a character wants to execute a `say` action, they must first obtain locks for both themselves and the target character.

## Design Principles

1. **Single lock per character**: Each character can only be in one conversation at a time
2. **Order-insensitive**: Either character can initiate a conversation
3. **Automatic timeout**: After 3 failed lock attempts, existing locks are automatically released
4. **Self-healing**: System automatically recovers from stuck conversations

## Implementation Details

### Map Node (`src/map_node.py`)

The `MapNode` class manages conversation locks with the following components:

- **`conversation_locks`**: Dictionary mapping character names to their conversation partner
- **`lock_request_counts`**: Tracks failed lock attempts for timeout detection
- **`lock_timeout_threshold`**: Number of failed attempts before timeout (default: 3)

#### Key Methods

- `acquire_conversation_lock(requester, target)`: Attempts to acquire locks for both characters
- `release_conversation_locks(character1, character2)`: Releases locks for both characters
- `_timeout_conversation_locks(requester, target)`: Automatically releases stuck locks
- `get_conversation_lock_status(character)`: Returns lock status for a character

#### Queryables

- `cognitive/map/conversation/lock/acquire`: Acquire conversation locks
- `cognitive/map/conversation/lock/release`: Release conversation locks  
- `cognitive/map/conversation/lock/status/*`: Check lock status for a character

### Executive Node (`src/executive_node.py`)

The `ZenohExecutiveNode` class integrates with the lock system:

- **`_acquire_conversation_lock(target_character)`**: Requests lock acquisition before say actions
- **`_release_conversation_lock(target_character)`**: Releases locks when conversations end
- **`check_conversation_lock_availability(target_character)`**: Checks if locks are available before planning

#### Lock Integration Points

1. **Say Action Execution**: Locks are acquired before `generate_speech()` is called
2. **Dialog End**: Locks are released when `publish_dialog_end()` is called
3. **Dialog End Callback**: Locks are released when `_dialog_end_callback()` is received
4. **Shutdown**: Locks are cleaned up when characters shut down

#### Retry Implementation

The retry mechanism is implemented through return value control:

- **`_act()` method**: Returns `True` on success, `False` on failure (e.g., lock unavailable)
- **`_run_ooda_loop()`**: Checks return value and preserves plan state on failure
- **Turn completion**: Always happens regardless of action success/failure
- **Plan state**: Only advances when actions succeed

This ensures that failed say actions (due to lock unavailability) are retried on subsequent turns without plan state corruption.

## Usage Examples

### Acquiring a Lock

```python
# In executive node
if self._acquire_conversation_lock("Joe"):
    self.generate_speech("Hello Joe!", "Joe", mode='say')
else:
    # Lock unavailable - action fails
    pass
```

### Releasing a Lock

```python
# When dialog ends
self._release_conversation_lock("Joe")
```

### Checking Lock Availability

```python
# Before planning
if self.check_conversation_lock_availability("Joe"):
    # Can plan to talk to Joe
    pass
else:
    # Joe is in another conversation
    pass
```

## API Endpoints

### Acquire Lock

**Endpoint**: `cognitive/map/conversation/lock/acquire`

**Payload**:
```json
{
    "requester": "Alice",
    "target": "Joe"
}
```

**Response**:
```json
{
    "success": true,
    "requester": "Alice", 
    "target": "Joe",
    "lock_acquired": true
}
```

### Release Lock

**Endpoint**: `cognitive/map/conversation/lock/release`

**Payload**:
```json
{
    "character1": "Alice",
    "character2": "Joe"
}
```

**Response**:
```json
{
    "success": true,
    "character1": "Alice",
    "character2": "Joe", 
    "message": "Locks released successfully"
}
```

### Check Lock Status

**Endpoint**: `cognitive/map/conversation/lock/status/{character_name}`

**Response**:
```json
{
    "success": true,
    "character": "Joe",
    "locked_with": "Alice",
    "is_locked": true
}
```

## Timeout Mechanism

The system uses a request-counting timeout mechanism:

1. When a character tries to acquire a lock with a locked character, the attempt count increases
2. After 3 failed attempts, the existing locks are automatically released
3. The requesting character can then acquire the locks

This prevents deadlocks from unresponsive characters while maintaining system responsiveness.

## Error Handling

- **Lock acquisition failures**: Say actions fail gracefully with error status
- **Network issues**: Timeouts prevent indefinite waiting
- **Character shutdown**: Locks are automatically cleaned up
- **Invalid requests**: Proper error responses for malformed requests

## Telemetry Synchronization

The system maintains proper telemetry synchronization through centralized plan management methods:

### **Centralized Plan Management**

1. **`_plan_completed()` Method**:
   - **Purpose**: Handle successful plan completion
   - **Actions**: Calls `_summarize_plan_execution()`, clears all plan state, resets `action_history = []`
   - **When called**: After all plan steps execute successfully

2. **`_plan_abandoned()` Method**:
   - **Purpose**: Handle plan abandonment for any reason
   - **Actions**: Clears all plan state, resets `action_history = []` (does NOT call summary)
   - **When called**: When abandoning plan before completion

### **Benefits of Centralization**

- **Single source of truth** for plan state cleanup
- **Easier maintenance** - change cleanup logic in one place
- **Consistent behavior** - all plan transitions use same logic
- **Clear intent** - method names make the purpose obvious
- **Reduced duplication** - eliminated scattered `action_history = []` lines

### **Usage Locations**

- **Plan Completion**: All successful plan completions call `_plan_completed()`
- **Plan Abandonment**: All plan replacements/abandonments call `_plan_abandoned()`
- **Error Recovery**: Plan execution errors call `_plan_abandoned()`

### **Future Enhancements (Phase 2)**

- **Comprehensive state capture**: JSON objects for memory storage and analysis
- **Enhanced telemetry**: Detailed plan termination conditions and outcomes
- **Memory integration**: Publish plan state to character memory for later processing

## Retry Mechanism

When a conversation lock is unavailable, the system implements a clean retry mechanism:

1. **Action Failure**: The `_act()` method returns `False` when lock acquisition fails
2. **Plan State Preservation**: The plan state is NOT advanced, preserving the current step
3. **Turn Completion**: The turn is still completed to maintain launcher synchronization
4. **Automatic Retry**: On the next turn, the same say action will be attempted again
5. **Self-Healing**: When locks become available (due to timeout or conversation ending), the retry will succeed

This approach:
- **Maintains plan integrity**: No plan state corruption or manipulation
- **Preserves turn structure**: Launcher continues to work normally
- **Natural retry semantics**: Characters keep trying until locks become available
- **No fallback actions**: Eliminates the need for complex action rewriting or insertion

### Example Flow

```
Turn 1: Character A tries to say "Hello" to Character B
        → Lock unavailable (B is talking to C)
        → Action fails, turn completes
        → Plan state preserved

Turn 2: Character A tries to say "Hello" to Character B again
        → Lock available (B's conversation with C ended)
        → Action succeeds, plan advances
```

## Testing

Use the provided test script to verify the system:

```bash
python test_conversation_locks.py
```

This tests:
- Lock acquisition and release
- Lock status checking
- Failed lock attempts
- Timeout behavior
- Error handling

## Future Enhancements

1. **Group conversations**: Support for 3+ character conversations
2. **Priority locks**: Allow certain characters to interrupt conversations
3. **Lock queuing**: Queue lock requests instead of rejecting them
4. **Configurable timeouts**: Per-scenario timeout thresholds
5. **Lock analytics**: Track lock usage patterns and performance

## Troubleshooting

### Common Issues

1. **Locks not releasing**: Check if `dialog_end` events are being published
2. **Characters stuck**: Verify timeout mechanism is working (3 failed attempts)
3. **Network timeouts**: Increase timeout values in debug mode
4. **Lock leaks**: Check character shutdown cleanup
5. **Retry not working**: Verify `_act()` returns `False` and plan state is preserved

### Debug Logging

Enable debug mode to see detailed lock operations:

```bash
export CWB_DEBUG=1
```

Look for log messages with:
- 🔒 (lock acquired)
- 🔓 (lock released) 
- ⏰ (timeout)
- 📤 (action published)
- 🔄 (retry logic)