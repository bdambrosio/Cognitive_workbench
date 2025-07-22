# IDE Import Troubleshooting Guide

## Problem
The IDE (VS Code/Cursor) shows import errors for ROS2 packages like:
- `Import "rclpy" could not be resolved`
- `Import "std_msgs.msg" could not be resolved`
- `Import "launch_ros.actions" could not be resolved`

## Root Cause
The IDE's Python interpreter doesn't automatically source the ROS2 environment, so it can't find ROS2 packages.

## Solutions

### ✅ Solution 1: Updated IDE Settings (Recommended)
The `.vscode/settings.json` and `.vscode/pyrightconfig.json` files have been updated to include:
- ROS2 package paths
- Virtual environment paths
- Proper Python analysis settings

### ✅ Solution 2: Restart IDE
After the settings are updated:
1. **Restart your IDE** (VS Code/Cursor)
2. **Reload the Python extension** (Ctrl+Shift+P → "Python: Reload Window")
3. **Select the correct Python interpreter** (Ctrl+Shift+P → "Python: Select Interpreter" → Choose `./ros_venv/bin/python3`)

### ✅ Solution 3: Run Setup Script
Run the setup script to verify all imports work:
```bash
source setup_environment.sh
python3 setup_ide_environment.py
```

### ✅ Solution 4: Manual Environment Setup
If the IDE still shows errors, manually set the Python path in your IDE:
1. Open Command Palette (Ctrl+Shift+P)
2. Type "Python: Select Interpreter"
3. Choose "Enter interpreter path..."
4. Enter: `./ros_venv/bin/python3`

## Verification
To verify the setup is working:
```bash
# Test ROS2 imports
source setup_environment.sh
python3 -c "import rclpy; import std_msgs.msg; print('✅ ROS2 imports work')"

# Test our package imports
python3 -c "from cognitive_framework.srv import MemoryRequest; print('✅ Service imports work')"
```

## Important Notes

### ⚠️ IDE vs Runtime
- **IDE warnings are cosmetic** - they don't affect actual execution
- **Code runs fine** when the environment is properly sourced
- **The warnings appear because** the IDE doesn't automatically source ROS2

### 🔧 Environment Requirements
- ROS2 Jazzy must be installed
- Virtual environment must be activated
- Workspace must be sourced
- All dependencies must be installed

### 📁 Key Files
- `.vscode/settings.json` - IDE settings
- `.vscode/pyrightconfig.json` - Python analysis settings
- `setup_environment.sh` - Environment setup script
- `setup_ide_environment.py` - Import verification script

## Troubleshooting Steps

1. **Check if imports work in terminal**:
   ```bash
   source setup_environment.sh
   python3 -c "import rclpy; print('OK')"
   ```

2. **Verify Python interpreter**:
   - Make sure IDE is using `./ros_venv/bin/python3`

3. **Check file paths**:
   - Ensure ROS2 is installed at `/opt/ros/jazzy/`
   - Ensure virtual environment exists at `./ros_venv/`

4. **Restart IDE components**:
   - Reload Python extension
   - Restart IDE completely

5. **Check for missing dependencies**:
   ```bash
   source ros_venv/bin/activate
   pip install packaging  # If launch_ros fails
   ```

## Expected Behavior
After proper setup:
- ✅ All ROS2 imports should resolve
- ✅ No more "could not be resolved" errors
- ✅ Code completion should work for ROS2 packages
- ✅ Debugging should work normally

## If Problems Persist
1. Delete `.vscode/` folder and recreate settings
2. Reinstall Python extension
3. Check ROS2 installation: `ros2 --version`
4. Verify virtual environment: `which python3` (should point to `./ros_venv/bin/python3`) 