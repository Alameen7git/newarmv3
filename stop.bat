@echo off
REM stop.bat - Windows-style safe shutdown
REM Usage: stop.bat

echo Stopping GELLO Robot...
taskkill /F /FI "WINDOWTITLE eq GELLO Robot"
echo Robot stopped.
