import os
import yaml
from pathlib import Path

DEFAULT_CONFIG_PATH = Path(__file__).parent / "config" / "calibration.yaml"

def load_calibration(path=DEFAULT_CONFIG_PATH):
    """Load calibration offsets from a YAML file."""
    if not os.path.exists(path):
        return None
    
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            return data.get("joint_offsets")
    except Exception as e:
        print(f"Warning: Failed to load calibration from {path}: {e}")
        return None

def save_calibration(offsets, path=DEFAULT_CONFIG_PATH):
    """Save calibration offsets to a YAML file."""
    # Ensure directory exists
    os.makedirs(os.path.dirname(path), exist_ok=True)
    
    data = {
        "joint_offsets": [float(x) for x in offsets],
        "units": "radians"
    }
    
    try:
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        print(f"Successfully saved calibration to {path}")
        return True
    except Exception as e:
        print(f"Error: Failed to save calibration to {path}: {e}")
        return False
