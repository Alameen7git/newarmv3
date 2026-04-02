@echo off
REM start.bat - Windows-style background startup
REM Usage: start.bat

echo Starting GELLO Robot...
start "GELLO Robot" /B python run_robot.py > robot.log 2>&1
echo Robot started. Monitoring robot.log...
type robot.log
