#!/bin/bash
# install_deps.sh - One-click setup for GELLO Gravity Comp
# Usage: ./install_deps.sh

echo "📦 Installing GELLO Gravity Compensation Dependencies..."

# Check for Python
if ! command -v python3 &> /dev/null
then
    echo "❌ Error: python3 is not installed."
    exit 1
fi

# Install pip requirements
python3 -m pip install --upgrade pip
python3 -m pip install dynamixel-sdk pin numpy pyyaml

echo "✅ All dependencies installed successfully."
echo "👉 Next step: Run './start.sh' to begin."
