# 🤖 GELLO UR5 Gravity Compensation — Developer Edition

> **Professional, ALOHA-style gravity compensation for custom GELLO UR5 arms.**
> Features persistent calibration, CLI-first control, and Hardware Homing (EEPROM).

---

## 🚀 Quick Start (VS Code)

This repository is designed for **one-click operation** in VS Code:

1.  **Plug in** your U2D2 and power the arm.
2.  **Select** 🚀 `Start Robot` from the **Run & Debug** drop-down menu in VS Code.
3.  **Done!** The arm will now float.

---

## 🛠️ Calibration Workflow (ALOHA-Style)

You only need to calibrate **once**. After that, the robot "remembers" its position.

### 1. Perform Initial Calibration
Place the arm in the **Home Pose** (see [Mechanical Alignment](#mechanical-alignment)) and run:
```bash
python3 run_robot.py --calibrate
```
This saves your offsets to `config/calibration.yaml`.

### 2. Enable "Start Anywhere" (Hardware Homing)
To avoid needing a home pose on every power-up, bake the calibration into the motor EEPROM:
```bash
python3 run_robot.py --calibrate --write-to-hw
```
*Requires a power cycle after running.*

---

## 📜 CLI Commands & Scripts

| Command | Description |
| :--- | :--- |
| `./start.sh` | Starts the robot in the background (ALOHA style) |
| `./stop.sh` | Safely stops the robot and disables torque |
| `tail -f robot.log` | Monitor live telemetry and system health |

---

## 🔧 Hardware Configuration

### 📍 Mechanical Alignment (Home Pose)
Before calibrating, ensure the arm is physically aligned as follows:

- **Joint 1 (Base)**: 0° (Straight forward)
- **Joint 2 (Shoulder)**: -90° (**Straight UP**)
- **Joint 3 (Elbow)**: +90° (Forearm horizontal)
- **Joint 4-6**: -90°, -90°, 0° (Neutral wrist)

### ⚙️ Motor IDs
- **ID 1**: XL330 (Base)
- **ID 2-3**: **XM430-W350-T** (Upgraded Shoulder/Elbow) ⭐
- **ID 4-7**: XL330 (Wrist/Gripper)

---

## 📦 Installation

```bash
./install_deps.sh
```
*Requires `python3` and `pip`.*

---

## 🤝 Integration with LeRobot
This repository is fully compatible with the Hugging Face **LeRobot** ecosystem. 
To integrate:
1. Copy the offsets from `config/calibration.yaml`.
2. Paste them into your LeRobot `robot_config.yaml` under the `offsets` section.
3. Use the `joint_signs: [1, 1, -1, 1, 1, 1, 1]`.

---

## 📄 License
MIT License. Based on the GELLO software framework.
