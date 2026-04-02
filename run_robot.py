#!/usr/bin/env python3
import sys
import os
from gravity_compensation import main

if __name__ == "__main__":
    # This script simply calls the refactored main() in gravity_compensation.py
    # which now handles persistence and hardware homing via CLI arguments.
    # 
    # Usage:
    #   python run_robot.py              <- Starts robot with stored config
    #   python run_robot.py --calibrate  <- Runs one-time calibration
    
    sys.exit(main())
