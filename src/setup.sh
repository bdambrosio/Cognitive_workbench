#!/bin/bash
# Setup script for Cognitive Workbench
#
# This script sets up the Python virtual environment and installs dependencies
# for the Cognitive Workbench project.

set -e  # Exit on error

echo "🚀 Setting up Cognitive Workbench"
echo "=================================="

# Determine script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$PROJECT_ROOT/zenoh_venv"

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.10 or higher."
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
echo "✅ Python 3 found: $(python3 --version)"

# Check Python version (3.10+)
PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d'.' -f1)
PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d'.' -f2)
if [ "$PYTHON_MAJOR" -lt 3 ] || ([ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 10 ]); then
    echo "❌ Python 3.10 or higher is required. Found: $PYTHON_VERSION"
    exit 1
fi

# Create virtual environment if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
    echo "✅ Virtual environment created at $VENV_DIR"
else
    echo "✅ Virtual environment already exists at $VENV_DIR"
fi

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Upgrade pip
echo "⬆️  Upgrading pip..."
pip install --upgrade pip --quiet

# Install dependencies
echo "📦 Installing dependencies from requirements.txt..."
cd "$SCRIPT_DIR"
pip install -r requirements.txt

# Verify key dependencies
echo "🧪 Verifying installation..."
python3 -c "import zenoh; import fastapi; import sentence_transformers; print('✅ Core dependencies verified')" 2>/dev/null || {
    echo "⚠️  Some dependencies may not be fully installed. Check errors above."
}

echo ""
echo "🎉 Setup completed successfully!"
echo ""
echo "To use Cognitive Workbench:"
echo "1. Activate the virtual environment:"
echo "   source $VENV_DIR/bin/activate"
echo ""
echo "2. Launch the system:"
echo "   cd $SCRIPT_DIR"
echo "   python launcher.py --help"
echo ""
echo "For more information, see README.md"
echo "" 