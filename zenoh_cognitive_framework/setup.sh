#!/bin/bash
"""
Setup script for Zenoh Cognitive Framework

This script sets up the environment for the Zenoh cognitive framework.
"""

echo "🚀 Setting up Zenoh Cognitive Framework"
echo "========================================"

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.8 or higher."
    exit 1
fi

echo "✅ Python 3 found: $(python3 --version)"

# Create virtual environment if it doesn't exist
if [ ! -d "zenoh_venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv zenoh_venv
    echo "✅ Virtual environment created"
else
    echo "✅ Virtual environment already exists"
fi

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source zenoh_venv/bin/activate

# Upgrade pip
echo "⬆️  Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo "📦 Installing dependencies..."
pip install -r requirements.txt

# Test installation
echo "🧪 Testing Zenoh installation..."
cd zenoh_cognitive_framework
python test_zenoh_installation.py

if [ $? -eq 0 ]; then
    echo ""
    echo "🎉 Setup completed successfully!"
    echo ""
    echo "To use the framework:"
    echo "1. Activate the virtual environment: source zenoh_venv/bin/activate"
    echo "2. Launch all nodes: python zenoh_cognitive_framework/launch_all_nodes.py"
    echo "3. Or launch individual nodes from the zenoh_cognitive_framework directory"
    echo ""
    echo "For more information, see zenoh_cognitive_framework/README.md"
else
    echo ""
    echo "❌ Setup failed. Please check the errors above."
    exit 1
fi 