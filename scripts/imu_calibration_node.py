#!/usr/bin/env python3
"""
imu_calibration_node.py — BNO085 IMU & Magnetometer Calibration Script

Interactive step-by-step calibration for BNO085 9-DOF IMU on Tomas_bot.
Guides the user through accelerometer, gyroscope, and magnetometer calibration
with clear serial instructions printed at each step.

Usage:
  ros2 run Tomas_bot imu_calibration_node.py

Or standalone:
  python3 scripts/imu_calibration_node.py --port /dev/arduino --baud 115200

The BNO085 uses Hillcrest's sensor fusion which performs dynamic calibration.
This script guides you through motions that help the BNO085's internal
calibration algorithms converge quickly and accurately.
"""

import sys
import time
import argparse
import threading
import serial


# ═══════════════════════════════════════════════════════════════════════════════
# Calibration step definitions
# ═══════════════════════════════════════════════════════════════════════════════

CALIBRATION_STEPS = [
    {
        "name": "GYROSCOPE CALIBRATION",
        "instructions": [
            "Place the robot on a FLAT, STABLE surface.",
            "Make sure the robot is completely STILL — no vibrations.",
            "Do NOT touch the robot during this step.",
            "The BNO085 will calibrate the gyroscope bias automatically.",
            "This takes approximately 5 seconds of stillness.",
        ],
        "duration": 8,
        "action": "still",
    },
    {
        "name": "ACCELEROMETER CALIBRATION — Position 1: Flat (normal position)",
        "instructions": [
            "Keep the robot in its NORMAL driving position.",
            "The BNO085 chip should be facing UP (Z-axis pointing toward ceiling).",
            "Hold still for 5 seconds.",
        ],
        "duration": 6,
        "action": "still",
    },
    {
        "name": "ACCELEROMETER CALIBRATION — Position 2: Nose UP",
        "instructions": [
            "Tilt the robot so the FRONT (LiDAR end) points UP at ~45°.",
            "Hold as steady as possible for 5 seconds.",
            "This helps the accelerometer calibrate the X-axis.",
        ],
        "duration": 6,
        "action": "still",
    },
    {
        "name": "ACCELEROMETER CALIBRATION — Position 3: Nose DOWN",
        "instructions": [
            "Tilt the robot so the FRONT (LiDAR end) points DOWN at ~45°.",
            "Hold steady for 5 seconds.",
        ],
        "duration": 6,
        "action": "still",
    },
    {
        "name": "ACCELEROMETER CALIBRATION — Position 4: Left side DOWN",
        "instructions": [
            "Tilt the robot so the LEFT side faces DOWN at ~45°.",
            "Hold steady for 5 seconds.",
            "This calibrates the Y-axis.",
        ],
        "duration": 6,
        "action": "still",
    },
    {
        "name": "ACCELEROMETER CALIBRATION — Position 5: Right side DOWN",
        "instructions": [
            "Tilt the robot so the RIGHT side faces DOWN at ~45°.",
            "Hold steady for 5 seconds.",
        ],
        "duration": 6,
        "action": "still",
    },
    {
        "name": "ACCELEROMETER CALIBRATION — Position 6: Upside down",
        "instructions": [
            "Carefully flip the robot UPSIDE DOWN (chip facing floor).",
            "Hold steady for 5 seconds.",
            "Then place the robot back in normal position.",
        ],
        "duration": 6,
        "action": "still",
    },
    {
        "name": "MAGNETOMETER CALIBRATION — Figure-8 Motion",
        "instructions": [
            "Place the robot back in NORMAL driving position on the floor.",
            "Slowly rotate the robot in a FIGURE-8 pattern:",
            "  → Turn the robot 360° CLOCKWISE (full rotation)",
            "  → Then turn 360° COUNTER-CLOCKWISE (full rotation)",
            "  → Repeat the figure-8 pattern 2-3 times.",
            "Move SLOWLY and SMOOTHLY — about 10 seconds per full rotation.",
            "IMPORTANT: Stay AWAY from motors, metal objects, and magnets.",
            "The BNO085 magnetometer needs exposure to Earth's field in all directions.",
        ],
        "duration": 45,
        "action": "figure8",
    },
    {
        "name": "MAGNETOMETER CALIBRATION — Tilt while Rotating",
        "instructions": [
            "Now combine rotation WITH gentle tilting:",
            "  → Rotate the robot CLOCKWISE 360° while gently rocking ±15° forward/back.",
            "  → Then rotate COUNTER-CLOCKWISE 360° with the same tilting.",
            "This gives the magnetometer data in the full 3D sphere.",
            "Total time: ~30 seconds.",
        ],
        "duration": 35,
        "action": "tilt_rotate",
    },
    {
        "name": "FINAL VERIFICATION — Static Hold",
        "instructions": [
            "Place the robot FLAT on the ground in normal driving position.",
            "Point the FRONT of the robot toward NORTH (or any known direction).",
            "Hold completely still for 10 seconds.",
            "The IMU readings will be checked for stability and accuracy.",
        ],
        "duration": 12,
        "action": "verify",
    },
]


def print_banner(text):
    width = max(60, len(text) + 4)
    print("\n" + "═" * width)
    print(f"  {text}")
    print("═" * width)


def print_step_header(step_num, total, name):
    print(f"\n{'─' * 60}")
    print(f"  STEP {step_num}/{total}: {name}")
    print(f"{'─' * 60}")


def countdown(seconds, label=""):
    for remaining in range(seconds, 0, -1):
        sys.stdout.write(f"\r  ⏱  {label}{remaining} seconds remaining...   ")
        sys.stdout.flush()
        time.sleep(1)
    sys.stdout.write(f"\r  ✓  {label}Complete!                              \n")
    sys.stdout.flush()


class IMUCalibrator:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.latest_imu = None
        self.imu_count = 0
        self.reader_thread = None

    def connect(self):
        print(f"\n  Connecting to Arduino on {self.port} @ {self.baud} baud...")
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.1)
            time.sleep(2.5)  # Wait for Arduino reset
            self.ser.reset_input_buffer()
            print(f"  ✓ Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"  ✗ Failed to connect: {e}")
            return False

    def start_reading(self):
        self.running = True
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

    def stop_reading(self):
        self.running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=2.0)

    def _read_loop(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('i '):
                        self.latest_imu = line
                        self.imu_count += 1
            except (serial.SerialException, UnicodeDecodeError):
                pass
            time.sleep(0.01)

    def get_imu_values(self):
        """Parse the latest IMU line into a dict."""
        if self.latest_imu is None:
            return None
        try:
            parts = self.latest_imu.split()
            if len(parts) != 11:
                return None
            return {
                'qw': float(parts[1]), 'qx': float(parts[2]),
                'qy': float(parts[3]), 'qz': float(parts[4]),
                'ax': float(parts[5]), 'ay': float(parts[6]), 'az': float(parts[7]),
                'gx': float(parts[8]), 'gy': float(parts[9]), 'gz': float(parts[10]),
            }
        except (ValueError, IndexError):
            return None

    def check_imu_health(self):
        """Verify IMU is sending data."""
        self.imu_count = 0
        time.sleep(2.0)
        if self.imu_count == 0:
            print("  ⚠  WARNING: No IMU data received!")
            print("     Check BNO085 wiring: VIN→5V, GND→GND, SDA→D2, SCL→D3")
            return False
        print(f"  ✓ IMU is active — received {self.imu_count} samples in 2 seconds")
        return True

    def verify_readings(self):
        """Collect readings for verification and display statistics."""
        samples = []
        print("  Collecting verification samples...")
        start = time.time()
        while time.time() - start < 5.0:
            vals = self.get_imu_values()
            if vals:
                samples.append(vals)
            time.sleep(0.05)

        if len(samples) < 10:
            print("  ⚠  Not enough IMU samples for verification")
            return

        # Compute averages
        avg = {k: sum(s[k] for s in samples) / len(samples) for k in samples[0]}

        # Quaternion magnitude (should be ~1.0)
        q_mag = (avg['qw']**2 + avg['qx']**2 + avg['qy']**2 + avg['qz']**2) ** 0.5

        # Gravity magnitude (should be ~9.81 m/s²)
        g_mag = (avg['ax']**2 + avg['ay']**2 + avg['az']**2) ** 0.5

        # Gyro drift (should be near zero when still)
        gyro_mag = (avg['gx']**2 + avg['gy']**2 + avg['gz']**2) ** 0.5

        print(f"\n  {'─' * 50}")
        print(f"  VERIFICATION RESULTS ({len(samples)} samples)")
        print(f"  {'─' * 50}")
        print(f"  Quaternion:     w={avg['qw']:.4f} x={avg['qx']:.4f} y={avg['qy']:.4f} z={avg['qz']:.4f}")
        print(f"  Quaternion norm: {q_mag:.4f} (should be ~1.0000)")
        print(f"  Accel (m/s²):   x={avg['ax']:.3f} y={avg['ay']:.3f} z={avg['az']:.3f}")
        print(f"  Gravity magnitude: {g_mag:.3f} m/s² (should be ~9.81)")
        print(f"  Gyro (rad/s):   x={avg['gx']:.4f} y={avg['gy']:.4f} z={avg['gz']:.4f}")
        print(f"  Gyro drift:     {gyro_mag:.4f} rad/s (should be <0.01)")

        # Health checks
        issues = []
        if abs(q_mag - 1.0) > 0.05:
            issues.append("Quaternion magnitude far from 1.0 — orientation may be unreliable")
        if abs(g_mag - 9.81) > 1.0:
            issues.append(f"Gravity magnitude {g_mag:.2f} differs from 9.81 — accelerometer may need recalibration")
        if gyro_mag > 0.05:
            issues.append(f"Gyroscope drift {gyro_mag:.4f} is high — ensure robot is completely still")

        if issues:
            print(f"\n  ⚠  ISSUES DETECTED:")
            for issue in issues:
                print(f"     • {issue}")
        else:
            print(f"\n  ✓ ALL CHECKS PASSED — IMU calibration looks good!")

    def close(self):
        self.stop_reading()
        if self.ser and self.ser.is_open:
            self.ser.close()

    def run_calibration(self):
        print_banner("BNO085 IMU CALIBRATION — Tomas_bot")
        print("  This script guides you through calibrating the BNO085 9-DOF IMU.")
        print("  The BNO085 performs dynamic self-calibration — you just need to")
        print("  move the robot through specific orientations and motions.")
        print(f"\n  Total steps: {len(CALIBRATION_STEPS)}")
        print("  Estimated time: ~3 minutes")

        if not self.connect():
            return False

        self.start_reading()

        # Check IMU is alive
        if not self.check_imu_health():
            self.close()
            return False

        input("\n  Press ENTER to begin calibration...")

        total = len(CALIBRATION_STEPS)
        for i, step in enumerate(CALIBRATION_STEPS, 1):
            print_step_header(i, total, step["name"])

            for instruction in step["instructions"]:
                print(f"    → {instruction}")

            input(f"\n  Press ENTER when ready for this step...")

            if step["action"] == "verify":
                countdown(step["duration"], "Verifying: ")
                self.verify_readings()
            else:
                countdown(step["duration"], "Hold: ")

            vals = self.get_imu_values()
            if vals:
                print(f"  📊 Current IMU: qw={vals['qw']:.3f} ax={vals['ax']:.2f} ay={vals['ay']:.2f} az={vals['az']:.2f} "
                      f"gx={vals['gx']:.3f} gy={vals['gy']:.3f} gz={vals['gz']:.3f}")

        print_banner("CALIBRATION COMPLETE")
        print("  The BNO085 calibration data is stored in the sensor's internal memory.")
        print("  The BNO085 refines calibration continuously during operation.")
        print("\n  Next steps:")
        print("    1. Run bringup:  ros2 launch Tomas_bot bringup_hardware.launch.py")
        print("    2. Check IMU:    ros2 topic echo /imu/data")
        print("    3. Check EKF:    ros2 topic echo /odom")
        print("    4. Start SLAM:   ros2 launch Tomas_bot slam_hardware.launch.py")

        self.close()
        return True


def main():
    parser = argparse.ArgumentParser(description='BNO085 IMU Calibration for Tomas_bot')
    parser.add_argument('--port', default='/dev/arduino', help='Serial port (default: /dev/arduino)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    args = parser.parse_args()

    calibrator = IMUCalibrator(args.port, args.baud)
    try:
        success = calibrator.run_calibration()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n  Calibration interrupted by user.")
        calibrator.close()
        sys.exit(1)


if __name__ == '__main__':
    main()
