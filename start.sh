#!/bin/bash
# start.sh - ALOHA-style background startup
# Usage: ./start.sh

# Get current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Start the robot in the background
# Redirects output to robot.log for troubleshooting
nohup python3 "$DIR/run_robot.py" > "$DIR/robot.log" 2>&1 &

echo "Robot started in background (PID: $!). Monitoring robot.log..."
tail -n 10 "$DIR/robot.log"
