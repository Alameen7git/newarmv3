#!/usr/bin/env python3
"""
Standalone Gravity Compensation for Custom GELLO UR5 Arm
=========================================================

A plug-and-play Python script for active gravity compensation on a
custom-redesigned GELLO UR5 leader arm.

Hardware Configuration:
  - Joint 1:    XL330-M288-T  (original)
  - Joint 2:    XM430-W350-T  (upgraded)
  - Joint 3:    XM430-W350-T  (upgraded)
  - Joint 4-6:  XL330-M288-T  (original)
  - Gripper:    XL330-M077-T  (original)

Dependencies:
  pip install dynamixel-sdk pin numpy

Usage:
  python gravity_compensation.py                     # Run with defaults
  python gravity_compensation.py --dry-run           # Test without torque
  python gravity_compensation.py --gain 0.3          # Custom gravity comp gain
  python gravity_compensation.py --port /dev/ttyUSB0 # Custom serial port

Author: Auto-generated for custom GELLO UR5 build
License: MIT
"""

import argparse
import os
import signal
import sys
import time
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Optional, Sequence, Tuple

import numpy as np

# ============================================================================
# CONFIGURATION — Edit these values to match your specific hardware setup
# ============================================================================

DEFAULT_CONFIG = {
    # ---- Serial / Hardware ----
    "port": "/dev/ttyUSB0",           # Serial port for U2D2 (Linux)
    # "port": "COM3",                 # Uncomment for Windows
    "baudrate": 57600,

    # ---- Motor IDs (in order from base to gripper) ----
    "motor_ids": [1, 2, 3, 4, 5, 6, 7],

    # ---- Servo types per joint (must match motor_ids order) ----
    "servo_types": [
        "XL330_M288_T",   # Joint 1 - Shoulder Pan
        "XM430_W350_T",   # Joint 2 - Shoulder Lift (UPGRADED)
        "XM430_W350_T",   # Joint 3 - Elbow (UPGRADED)
        "XL330_M288_T",   # Joint 4 - Wrist 1
        "XL330_M288_T",   # Joint 5 - Wrist 2
        "XL330_M288_T",   # Joint 6 - Wrist 3
        "XL330_M077_T",   # Gripper
    ],

    # ---- Joint sign convention (standard GELLO UR5) ----
    "joint_signs": [1, 1, -1, 1, 1, 1, 1],

    # ---- Number of arm joints (excluding gripper) ----
    "num_arm_joints": 6,

    # ---- URDF path (relative to this script or absolute) ----
    "urdf_path": "gello_ur5.urdf",

    # ---- Control loop frequency (Hz) ----
    "control_frequency_hz": 500,

    # ---- Gravity compensation gain (start low ~0.3, increase to ~0.8) ----
    "gravity_comp_gain": 0.6,

    # ---- Static friction compensation ----
    "friction_comp_gain": 0.0,         # Set >0 to enable (e.g., 0.15)
    "friction_enable_speed": 0.05,     # rad/s threshold

    # ---- Joint limit barrier (repulsive torques near limits) ----
    "joint_limits_max": [ 3.05, 3.05,  3.05,  3.05,  3.05,  3.05],
    "joint_limits_min": [-3.05, -3.05, -3.05, -3.05, -3.05, -3.05],
    "joint_limit_kp": 2.0,
    "joint_limit_kd": 0.1,

    # ---- Calibration position (put arm here before starting) ----
    # Standard GELLO UR5 home: straight up
    "calibration_joint_pos": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],

    # ---- Safety ----
    "dry_run": False,                  # True = read only, no torque output
}


# ============================================================================
# MOTOR CONSTANTS
# ============================================================================

# Torque-to-current ratio: mA per Nm
# These are derived from motor datasheets and the GELLO driver
TORQUE_TO_CURRENT = {
    "XL330_M288_T": 1158.73,          # From GELLO: XC330_T288_T mapping
    "XM430_W350_T": 561.80,           # ~1000/1.78 Nm/A at 12V
    "XL330_M077_T": 1158.73,          # Same family as M288
}

# Maximum safe current per motor type (mA)
# Capped below absolute max for safety margin
CURRENT_LIMITS = {
    "XL330_M288_T": 1100,             # Stall ~1193 mA
    "XM430_W350_T": 2000,             # Stall ~2300 mA at 12V
    "XL330_M077_T": 600,              # Lower torque variant
}


# ============================================================================
# DYNAMIXEL SDK CONSTANTS
# ============================================================================

ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_GOAL_CURRENT = 102
LEN_GOAL_CURRENT = 2
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
CURRENT_CONTROL_MODE = 0
POSITION_CONTROL_MODE = 3


# ============================================================================
# DYNAMIXEL INTERFACE
# ============================================================================

class DynamixelInterface:
    """Lightweight Dynamixel SDK wrapper for multi-motor communication."""

    def __init__(
        self,
        motor_ids: Sequence[int],
        servo_types: Sequence[str],
        port: str,
        baudrate: int = 57600,
    ):
        self.motor_ids = list(motor_ids)
        self.servo_types = list(servo_types)
        self.port = port
        self.baudrate = baudrate
        self.num_motors = len(motor_ids)

        # Build per-motor torque mapping arrays
        self.torque_to_current_map = np.array(
            [TORQUE_TO_CURRENT[s] for s in servo_types], dtype=float
        )
        self.current_limits_arr = np.array(
            [CURRENT_LIMITS[s] for s in servo_types], dtype=float
        )

        self._torque_enabled = False
        self._positions = None          # Latest read (raw int ticks)
        self._velocities = None         # Latest read (raw int units)
        self._lock = Lock()
        self._stop_event = Event()
        self._read_thread: Optional[Thread] = None

        # Will be set in connect()
        self._port_handler = None
        self._packet_handler = None
        self._sync_read = None
        self._sync_write_current = None

    def connect(self) -> None:
        """Open serial port and initialize sync read/write."""
        try:
            from dynamixel_sdk import (
                GroupSyncRead,
                GroupSyncWrite,
                PacketHandler,
                PortHandler,
            )
        except ImportError:
            print("ERROR: dynamixel_sdk not found.")
            print("Install with: pip install dynamixel-sdk")
            sys.exit(1)

        self._port_handler = PortHandler(self.port)
        self._packet_handler = PacketHandler(2.0)  # Protocol 2.0

        if not self._port_handler.openPort():
            raise RuntimeError(f"Failed to open port: {self.port}")
        if not self._port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set baudrate: {self.baudrate}")

        # Sync read: velocity (4 bytes) + position (4 bytes) = 8 bytes starting at velocity addr
        self._sync_read = GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            ADDR_PRESENT_VELOCITY,
            LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION,
        )
        for dxl_id in self.motor_ids:
            if not self._sync_read.addParam(dxl_id):
                raise RuntimeError(f"Failed to add sync read param for motor {dxl_id}")

        # Sync write for current commands
        self._sync_write_current = GroupSyncWrite(
            self._port_handler,
            self._packet_handler,
            ADDR_GOAL_CURRENT,
            LEN_GOAL_CURRENT,
        )

        print(f"Connected to {self.num_motors} Dynamixel servos on {self.port}")

        # Start background read thread
        self._start_read_thread()

    def set_operating_mode(self, mode: int) -> None:
        """Set operating mode for all motors. Must be called with torque disabled."""
        from dynamixel_sdk.robotis_def import COMM_SUCCESS

        with self._lock:
            for dxl_id in self.motor_ids:
                result, error = self._packet_handler.write1ByteTxRx(
                    self._port_handler, dxl_id, ADDR_OPERATING_MODE, mode
                )
                if result != COMM_SUCCESS or error != 0:
                    raise RuntimeError(
                        f"Failed to set operating mode for motor {dxl_id} "
                        f"(result={result}, error={error})"
                    )
        mode_name = {0: "Current Control", 3: "Position Control"}.get(mode, str(mode))
        print(f"Operating mode set to: {mode_name}")

    def enable_torque(self) -> None:
        """Enable torque on all motors."""
        self._set_torque(True)

    def disable_torque(self) -> None:
        """Disable torque on all motors."""
        self._set_torque(False)

    def _set_torque(self, enable: bool) -> None:
        from dynamixel_sdk.robotis_def import COMM_SUCCESS

        value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            for dxl_id in self.motor_ids:
                result, error = self._packet_handler.write1ByteTxRx(
                    self._port_handler, dxl_id, ADDR_TORQUE_ENABLE, value
                )
                if result != COMM_SUCCESS or error != 0:
                    print(f"WARNING: Torque {'enable' if enable else 'disable'} "
                          f"failed for motor {dxl_id}")
        self._torque_enabled = enable

    def read_positions_and_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get latest joint positions (rad) and velocities (rad/s)."""
        while self._positions is None or self._velocities is None:
            time.sleep(0.01)
        pos_rad = self._positions.copy() / 2048.0 * np.pi
        vel_rad_s = self._velocities.copy() * 0.229 * 2 * np.pi / 60
        return pos_rad, vel_rad_s

    def write_currents(self, currents_mA: np.ndarray) -> None:
        """Write current commands to all motors via sync write."""
        from dynamixel_sdk.robotis_def import (
            COMM_SUCCESS,
            DXL_HIBYTE,
            DXL_LOBYTE,
        )

        if not self._torque_enabled:
            return

        # Clamp to per-motor limits
        currents_clamped = np.clip(
            currents_mA, -self.current_limits_arr, self.current_limits_arr
        )

        with self._lock:
            for dxl_id, current in zip(self.motor_ids, currents_clamped):
                current_int = int(current)
                param = [DXL_LOBYTE(current_int), DXL_HIBYTE(current_int)]
                if not self._sync_write_current.addParam(dxl_id, param):
                    print(f"WARNING: Failed to add current param for motor {dxl_id}")
                    return

            result = self._sync_write_current.txPacket()
            if result != COMM_SUCCESS:
                print(f"WARNING: Sync write current failed (result={result})")
            self._sync_write_current.clearParam()

    def write_torques(self, torques_Nm: np.ndarray) -> None:
        """Convert torques (Nm) to currents (mA) and write."""
        currents = self.torque_to_current_map * torques_Nm
        self.write_currents(currents)

    def _start_read_thread(self) -> None:
        self._read_thread = Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def _read_loop(self) -> None:
        """Continuously read positions and velocities in background."""
        from dynamixel_sdk.robotis_def import COMM_SUCCESS

        while not self._stop_event.is_set():
            time.sleep(0.001)
            with self._lock:
                result = self._sync_read.txRxPacket()
                if result != COMM_SUCCESS:
                    continue

                positions = np.zeros(self.num_motors, dtype=np.int32)
                velocities = np.zeros(self.num_motors, dtype=np.int32)

                for i, dxl_id in enumerate(self.motor_ids):
                    # Read velocity
                    if self._sync_read.isAvailable(
                        dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY
                    ):
                        vel = self._sync_read.getData(
                            dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY
                        )
                        if vel > 0x7FFFFFFF:
                            vel -= 0x100000000
                        velocities[i] = vel

                    # Read position
                    if self._sync_read.isAvailable(
                        dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    ):
                        pos = self._sync_read.getData(
                            dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                        )
                        if pos > 0x7FFFFFFF:
                            pos -= 0x100000000
                        positions[i] = pos

                self._positions = positions
                self._velocities = velocities

    def close(self) -> None:
        """Stop read thread, disable torque, close port."""
        self._stop_event.set()
        if self._read_thread is not None:
            self._read_thread.join(timeout=2.0)
        if self._port_handler is not None:
            try:
                self.disable_torque()
            except Exception:
                pass
            self._port_handler.closePort()
        print("Dynamixel interface closed.")


# ============================================================================
# GRAVITY COMPENSATOR
# ============================================================================

class GravityCompensator:
    """
    Active gravity compensation for a custom GELLO UR5 arm.

    Uses Pinocchio (pin) for rigid-body inverse dynamics to compute
    the torques needed to counteract gravity, then sends corresponding
    current commands to the Dynamixel servos.
    """

    # Calibration search range: ±20π
    CALIBRATION_RANGE = 20
    CALIBRATION_STEPS = 81

    def __init__(self, config: dict):
        self.config = config
        self.running = False

        # Extract config
        self.num_arm_joints = config["num_arm_joints"]
        self.joint_signs = np.array(config["joint_signs"], dtype=float)
        self.calibration_pos = np.array(config["calibration_joint_pos"], dtype=float)
        self.gravity_comp_gain = config["gravity_comp_gain"]
        self.friction_comp_gain = config["friction_comp_gain"]
        self.friction_enable_speed = config["friction_enable_speed"]
        self.joint_limits_max = np.array(config["joint_limits_max"], dtype=float)
        self.joint_limits_min = np.array(config["joint_limits_min"], dtype=float)
        self.joint_limit_kp = config["joint_limit_kp"]
        self.joint_limit_kd = config["joint_limit_kd"]
        self.dt = 1.0 / config["control_frequency_hz"]
        self.dry_run = config["dry_run"]

        # State
        self.joint_offsets = np.zeros(len(config["motor_ids"]), dtype=float)
        self.tau_g = np.zeros(self.num_arm_joints, dtype=float)
        self.stiction_dither = np.ones(self.num_arm_joints, dtype=bool)

        # Initialize hardware
        self.dxl = DynamixelInterface(
            motor_ids=config["motor_ids"],
            servo_types=config["servo_types"],
            port=config["port"],
            baudrate=config["baudrate"],
        )

        # Initialize Pinocchio model
        self.pin_model = None
        self.pin_data = None

    def initialize(self) -> None:
        """Connect to hardware, load URDF, calibrate."""
        print("=" * 60)
        print("GELLO UR5 Gravity Compensation — Initializing")
        print("=" * 60)

        # Step 1: Connect to Dynamixel servos
        print("\n[1/4] Connecting to Dynamixel servos...")
        self.dxl.connect()

        # Step 2: Load URDF and build Pinocchio model
        print("\n[2/4] Loading URDF for inverse dynamics...")
        self._load_urdf()

        # Step 3: Calibrate joint offsets
        print("\n[3/4] Calibrating joint offsets...")
        self._calibrate()

        # Step 4: Configure servos for current control
        print("\n[4/4] Configuring servos...")
        if not self.dry_run:
            self.dxl.disable_torque()
            self.dxl.set_operating_mode(CURRENT_CONTROL_MODE)
            self.dxl.enable_torque()
            print("Servos set to CURRENT CONTROL mode with torque ENABLED")
        else:
            print("DRY RUN mode — torque will NOT be enabled")

        print("\n" + "=" * 60)
        print("Initialization complete!")
        print(f"  Gravity comp gain: {self.gravity_comp_gain}")
        print(f"  Control frequency: {1/self.dt:.0f} Hz")
        print(f"  Dry run: {self.dry_run}")
        print("=" * 60)

    def _load_urdf(self) -> None:
        """Load URDF and build Pinocchio model."""
        try:
            import pinocchio as pin
        except ImportError:
            print("ERROR: pinocchio (pin) not found.")
            print("Install with: pip install pin")
            sys.exit(1)

        urdf_path = self.config["urdf_path"]

        # Try relative to script first
        script_dir = Path(__file__).parent
        candidate = script_dir / urdf_path
        if candidate.exists():
            urdf_path = str(candidate)
        elif not Path(urdf_path).exists():
            raise FileNotFoundError(
                f"URDF file not found: {urdf_path}\n"
                f"Looked in: {candidate} and {Path(urdf_path).resolve()}"
            )

        print(f"  URDF: {urdf_path}")
        urdf_dir = str(Path(urdf_path).parent)
        self.pin_model, _, _ = pin.buildModelsFromUrdf(
            filename=str(urdf_path), package_dirs=urdf_dir
        )
        self.pin_data = self.pin_model.createData()
        print(f"  Model DOFs: {self.pin_model.nq}")

    def _calibrate(self) -> None:
        """
        Calibrate Dynamixel offsets by matching current raw positions
        to the expected calibration joint positions.

        The arm must be placed in the calibration pose before running.
        """
        # Warm up: take a few readings to stabilize
        for _ in range(10):
            self.dxl.read_positions_and_velocities()

        raw_pos, _ = self.dxl.read_positions_and_velocities()

        # Find offset for each arm joint
        offsets = []
        for i in range(self.num_arm_joints):
            best_offset = 0.0
            best_error = float("inf")

            for offset in np.linspace(
                -self.CALIBRATION_RANGE * np.pi,
                self.CALIBRATION_RANGE * np.pi,
                self.CALIBRATION_STEPS,
            ):
                joint_val = self.joint_signs[i] * (raw_pos[i] - offset)
                error = abs(joint_val - self.calibration_pos[i])
                if error < best_error:
                    best_error = error
                    best_offset = offset
            offsets.append(best_offset)

        # Gripper offset = raw reading (so gripper starts at 0)
        offsets.append(raw_pos[-1])
        self.joint_offsets = np.array(offsets, dtype=float)

        print(f"  Joint offsets: {[f'{x:.3f}' for x in self.joint_offsets]}")

        # Verify calibration
        arm_pos = self._get_arm_positions(raw_pos)
        error = np.abs(arm_pos - self.calibration_pos)
        print(f"  Calibration error: {[f'{x:.4f}' for x in error]}")
        max_error = np.max(error)
        if max_error > 0.3:
            print(f"  WARNING: Max calibration error is {max_error:.3f} rad")
            print("  Make sure the arm is in the calibration position!")

    def _get_arm_positions(self, raw_pos: np.ndarray) -> np.ndarray:
        """Apply offsets and signs to get arm joint positions."""
        n = self.num_arm_joints
        return (raw_pos[:n] - self.joint_offsets[:n]) * self.joint_signs[:n]

    def _get_arm_velocities(self, raw_vel: np.ndarray) -> np.ndarray:
        """Apply signs to get arm joint velocities."""
        n = self.num_arm_joints
        return raw_vel[:n] * self.joint_signs[:n]

    def get_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get calibrated arm joint positions and velocities."""
        raw_pos, raw_vel = self.dxl.read_positions_and_velocities()
        arm_pos = self._get_arm_positions(raw_pos)
        arm_vel = self._get_arm_velocities(raw_vel)
        return arm_pos, arm_vel

    def compute_gravity_torque(
        self, q: np.ndarray, dq: np.ndarray
    ) -> np.ndarray:
        """
        Compute gravity compensation torques using Pinocchio's RNEA.

        Recursive Newton-Euler Algorithm with zero acceleration gives us
        the torques needed to hold the arm stationary against gravity.
        """
        import pinocchio as pin

        self.tau_g = pin.rnea(
            self.pin_model,
            self.pin_data,
            q,       # joint positions
            dq,      # joint velocities (for Coriolis)
            np.zeros_like(dq),  # zero acceleration => gravity only
        )
        self.tau_g *= self.gravity_comp_gain
        return self.tau_g

    def compute_friction_torque(self, dq: np.ndarray) -> np.ndarray:
        """
        Static friction compensation using dithering.

        When joint velocity is near zero, apply small alternating torques
        to overcome stiction (static friction) in the servos/gears.
        """
        if self.friction_comp_gain <= 0:
            return np.zeros(self.num_arm_joints)

        tau_f = np.zeros(self.num_arm_joints)
        for i in range(self.num_arm_joints):
            if abs(dq[i]) < self.friction_enable_speed:
                if self.stiction_dither[i]:
                    tau_f[i] = self.friction_comp_gain * abs(self.tau_g[i])
                else:
                    tau_f[i] = -self.friction_comp_gain * abs(self.tau_g[i])
                self.stiction_dither[i] = not self.stiction_dither[i]
        return tau_f

    def compute_joint_limit_torque(
        self, q: np.ndarray, dq: np.ndarray
    ) -> np.ndarray:
        """
        Repulsive barrier torques near joint limits.

        Pushes the arm away from limits with a PD-like spring-damper.
        """
        tau_l = np.zeros(self.num_arm_joints)

        # Exceeded upper limit
        exceed_max = q > self.joint_limits_max
        if np.any(exceed_max):
            tau_l += (
                -self.joint_limit_kp * (q - self.joint_limits_max)
                - self.joint_limit_kd * dq
            ) * exceed_max

        # Exceeded lower limit
        exceed_min = q < self.joint_limits_min
        if np.any(exceed_min):
            tau_l += (
                -self.joint_limit_kp * (q - self.joint_limits_min)
                - self.joint_limit_kd * dq
            ) * exceed_min

        return tau_l

    def control_step(self) -> None:
        """Execute one step of the gravity compensation control loop."""
        # Read current state
        arm_pos, arm_vel = self.get_joint_states()

        # Compute torques
        torque_arm = np.zeros(self.num_arm_joints)

        # 1. Gravity compensation (main component)
        torque_arm += self.compute_gravity_torque(arm_pos, arm_vel)

        # 2. Friction compensation
        torque_arm += self.compute_friction_torque(arm_vel)

        # 3. Joint limit barriers
        torque_arm += self.compute_joint_limit_torque(arm_pos, arm_vel)

        if self.dry_run:
            # Just print what would be sent
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0

            if self._log_counter % 500 == 0:  # Print every ~1 second
                print(f"\n[DRY RUN] Positions (deg): "
                      f"{[f'{np.degrees(x):7.1f}' for x in arm_pos]}")
                print(f"[DRY RUN] Gravity τ (Nm):  "
                      f"{[f'{x:7.4f}' for x in self.tau_g]}")
                print(f"[DRY RUN] Total τ (Nm):    "
                      f"{[f'{x:7.4f}' for x in torque_arm]}")
                # Show what currents would be sent
                full_torque = np.append(torque_arm, 0.0)  # gripper = 0
                full_torque_signed = full_torque * self.joint_signs
                currents = self.dxl.torque_to_current_map * full_torque_signed
                print(f"[DRY RUN] Currents (mA):   "
                      f"{[f'{x:7.1f}' for x in currents]}")
        else:
            # Build full torque command (arm + gripper)
            full_torque = np.append(torque_arm, 0.0)  # gripper torque = 0
            # Apply joint signs for motor direction
            full_torque_signed = full_torque * self.joint_signs
            self.dxl.write_torques(full_torque_signed)

    def run(self) -> None:
        """Main control loop."""
        print(f"\nStarting gravity compensation at {1/self.dt:.0f} Hz")
        print("Press Ctrl+C to stop\n")

        self.running = True
        loop_count = 0
        overrun_count = 0

        try:
            while self.running:
                t0 = time.time()

                self.control_step()

                elapsed = time.time() - t0
                sleep_time = self.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    overrun_count += 1
                    if overrun_count % 100 == 1:
                        print(f"WARNING: Loop overrun by {-sleep_time*1000:.1f}ms "
                              f"(total overruns: {overrun_count})")

                loop_count += 1

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.shutdown()

        print(f"Ran {loop_count} control loops, {overrun_count} overruns")

    def shutdown(self) -> None:
        """Safely shutdown: zero torques, disable motors, close connection."""
        self.running = False
        print("\nShutting down...")

        if not self.dry_run:
            try:
                # Send zero torques first
                zero_torque = np.zeros(len(self.config["motor_ids"]))
                self.dxl.write_currents(zero_torque)
                time.sleep(0.05)
            except Exception as e:
                print(f"Warning during zero torque: {e}")

        self.dxl.close()
        print("Shutdown complete.")


# ============================================================================
# MAIN
# ============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Gravity Compensation for Custom GELLO UR5 Arm",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python gravity_compensation.py                          # Run with defaults
  python gravity_compensation.py --dry-run                # Test without torque
  python gravity_compensation.py --gain 0.3               # Low gain (safe start)
  python gravity_compensation.py --port /dev/ttyUSB0      # Specify port
  python gravity_compensation.py --port COM3              # Windows port
  python gravity_compensation.py --freq 200               # Lower control rate

Calibration:
  Before starting, place the arm in the calibration position:
  [0, -π/2, π/2, -π/2, -π/2, 0] (standard GELLO UR5 home)

Safety:
  Always start with --dry-run first to verify sensor readings.
  Then start with --gain 0.2 and gradually increase.
        """
    )
    parser.add_argument(
        "--port", "-p", type=str, default=None,
        help="Serial port (e.g., /dev/ttyUSB0 or COM3)"
    )
    parser.add_argument(
        "--gain", "-g", type=float, default=None,
        help="Gravity compensation gain (0.0 to 1.0, default: 0.6)"
    )
    parser.add_argument(
        "--freq", "-f", type=int, default=None,
        help="Control loop frequency in Hz (default: 500)"
    )
    parser.add_argument(
        "--dry-run", "-d", action="store_true",
        help="Dry run mode: read sensors only, no torque output"
    )
    parser.add_argument(
        "--urdf", "-u", type=str, default=None,
        help="Path to URDF file (default: gello_ur5.urdf)"
    )
    parser.add_argument(
        "--friction", type=float, default=None,
        help="Friction compensation gain (default: 0.0)"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    # Build config from defaults + CLI overrides
    config = DEFAULT_CONFIG.copy()
    if args.port is not None:
        config["port"] = args.port
    if args.gain is not None:
        config["gravity_comp_gain"] = args.gain
    if args.freq is not None:
        config["control_frequency_hz"] = args.freq
    if args.dry_run:
        config["dry_run"] = True
    if args.urdf is not None:
        config["urdf_path"] = args.urdf
    if args.friction is not None:
        config["friction_comp_gain"] = args.friction

    try:
        comp = GravityCompensator(config)

        # Setup signal handlers
        def signal_handler(signum, frame):
            print("\nReceived shutdown signal")
            comp.running = False

        signal.signal(signal.SIGINT, signal_handler)
        if hasattr(signal, 'SIGTERM'):
            signal.signal(signal.SIGTERM, signal_handler)

        comp.initialize()
        comp.run()

    except FileNotFoundError as e:
        print(f"\nERROR: {e}")
        return 1
    except RuntimeError as e:
        print(f"\nERROR: {e}")
        return 1
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
