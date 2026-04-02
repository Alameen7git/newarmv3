#!/bin/bash
# stop.sh - ALOHA-style safe shutdown
# Usage: ./stop.sh

# Find the PID of the run_robot.py process
PIDS=$(pgrep -f run_robot.py)

if [ -z "$PIDS" ]; then
    echo "Robot is not running."
    exit 0
fi

echo "Stopping robot (PIDs: $PIDS)..."
for pid in $PIDS; do
    kill -SIGINT $pid
done

echo "Waiting for robot to shutdown gracefully..."
sleep 2
echo "Done."
