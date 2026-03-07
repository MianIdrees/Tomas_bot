#!/usr/bin/env python3
"""
imu_check_node.py — BNO085 IMU Diagnostic & Health Check Script

Reads BNO085 IMU data from Arduino serial and displays real-time readings
for verifying that the IMU is working correctly. Shows:
  - Quaternion orientation (+ Euler angles)
  - Accelerometer (m/s²) with gravity magnitude
  - Gyroscope (rad/s)
  - Data rate and connection health

Usage:
  ros2 run Tomas_bot imu_check_node.py

Or standalone:
  python3 scripts/imu_check_node.py --port /dev/arduino --baud 115200
"""

import sys
import math
import time
import argparse
import threading
import serial


def quaternion_to_euler(qw, qx, qy, qz):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class IMUChecker:
    def __init__(self, port, baud, duration):
        self.port = port
        self.baud = baud
        self.duration = duration
        self.ser = None
        self.running = False
        self.reader_thread = None

        # Data tracking
        self.latest_imu = None
        self.sample_count = 0
        self.start_time = None
        self.last_print_time = 0

        # Statistics
        self.min_vals = {}
        self.max_vals = {}
        self.sum_vals = {}

    def connect(self):
        print(f"\n  Connecting to Arduino on {self.port} @ {self.baud} baud...")
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.1)
            time.sleep(2.5)
            self.ser.reset_input_buffer()
            print(f"  ✓ Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"  ✗ Failed to connect: {e}")
            return False

    def start_reading(self):
        self.running = True
        self.start_time = time.time()
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
                        self._process_imu_line(line)
                    elif line.startswith('e '):
                        pass  # Encoder data — ignore for IMU check
                    elif line and not line.startswith('d ') and not line.startswith('m '):
                        # Print any status/error messages from Arduino
                        print(f"  [Arduino] {line}")
            except (serial.SerialException, UnicodeDecodeError):
                pass
            time.sleep(0.005)

    def _process_imu_line(self, line):
        try:
            parts = line.split()
            if len(parts) != 11:
                return
            vals = {
                'qw': float(parts[1]), 'qx': float(parts[2]),
                'qy': float(parts[3]), 'qz': float(parts[4]),
                'ax': float(parts[5]), 'ay': float(parts[6]), 'az': float(parts[7]),
                'gx': float(parts[8]), 'gy': float(parts[9]), 'gz': float(parts[10]),
            }
            self.latest_imu = vals
            self.sample_count += 1

            # Update min/max/sum statistics
            for key, val in vals.items():
                if key not in self.min_vals:
                    self.min_vals[key] = val
                    self.max_vals[key] = val
                    self.sum_vals[key] = 0.0
                self.min_vals[key] = min(self.min_vals[key], val)
                self.max_vals[key] = max(self.max_vals[key], val)
                self.sum_vals[key] += val

        except (ValueError, IndexError):
            pass

    def display_loop(self):
        """Main display loop — prints formatted IMU data at ~2Hz."""
        print("\n" + "═" * 70)
        print("  BNO085 IMU Health Check — Live Data")
        print("  Press Ctrl+C to stop")
        print("═" * 70)

        while self.running:
            now = time.time()
            if now - self.last_print_time < 0.5:
                time.sleep(0.05)
                if self.duration > 0 and (now - self.start_time) >= self.duration:
                    self.running = False
                continue

            self.last_print_time = now
            elapsed = now - self.start_time

            if self.duration > 0 and elapsed >= self.duration:
                self.running = False
                continue

            if self.latest_imu is None:
                print(f"\r  ⏳ Waiting for IMU data... ({elapsed:.0f}s)", end='')
                continue

            v = self.latest_imu
            rate = self.sample_count / max(elapsed, 0.001)

            # Quaternion magnitude
            q_mag = (v['qw']**2 + v['qx']**2 + v['qy']**2 + v['qz']**2) ** 0.5

            # Euler angles
            roll, pitch, yaw = quaternion_to_euler(v['qw'], v['qx'], v['qy'], v['qz'])

            # Gravity magnitude
            g_mag = (v['ax']**2 + v['ay']**2 + v['az']**2) ** 0.5

            # Gyro magnitude
            gyro_mag = (v['gx']**2 + v['gy']**2 + v['gz']**2) ** 0.5

            # Clear screen and print
            print(f"\033[2J\033[H")  # Clear terminal screen
            print("═" * 70)
            print("  BNO085 IMU Health Check — Live Data")
            duration_str = f"  Duration: {self.duration}s" if self.duration > 0 else ""
            print(f"  Elapsed: {elapsed:.1f}s  |  Samples: {self.sample_count}  |  Rate: {rate:.1f} Hz{duration_str}")
            print("═" * 70)

            print(f"\n  ── Orientation (Quaternion) ──")
            print(f"     w: {v['qw']:+8.4f}   x: {v['qx']:+8.4f}   y: {v['qy']:+8.4f}   z: {v['qz']:+8.4f}")
            print(f"     Norm: {q_mag:.4f} {'✓' if abs(q_mag - 1.0) < 0.02 else '⚠ SHOULD BE ~1.0'}")

            print(f"\n  ── Orientation (Euler degrees) ──")
            print(f"     Roll: {roll:+8.2f}°   Pitch: {pitch:+8.2f}°   Yaw: {yaw:+8.2f}°")

            print(f"\n  ── Accelerometer (m/s²) ──")
            print(f"     X: {v['ax']:+8.3f}   Y: {v['ay']:+8.3f}   Z: {v['az']:+8.3f}")
            print(f"     |g| = {g_mag:.3f} m/s² {'✓' if abs(g_mag - 9.81) < 0.5 else '⚠ SHOULD BE ~9.81'}")

            print(f"\n  ── Gyroscope (rad/s) ──")
            print(f"     X: {v['gx']:+8.4f}   Y: {v['gy']:+8.4f}   Z: {v['gz']:+8.4f}")
            print(f"     |ω| = {gyro_mag:.4f} rad/s {'✓ (still)' if gyro_mag < 0.02 else '◉ rotating'}")

            # Health summary
            print(f"\n  ── Health ──")
            issues = 0
            if abs(q_mag - 1.0) > 0.05:
                print(f"     ⚠  Quaternion norm {q_mag:.4f} — should be 1.0")
                issues += 1
            if abs(g_mag - 9.81) > 1.5:
                print(f"     ⚠  Gravity magnitude {g_mag:.2f} — should be ~9.81 m/s²")
                issues += 1
            if rate < 10:
                print(f"     ⚠  Data rate {rate:.1f} Hz — expected ~20 Hz")
                issues += 1
            if issues == 0:
                print(f"     ✓  ALL OK — IMU readings look healthy")

            print(f"\n  Press Ctrl+C to stop")

    def print_summary(self):
        """Print final statistics summary."""
        elapsed = time.time() - self.start_time
        rate = self.sample_count / max(elapsed, 0.001)

        print("\n" + "═" * 70)
        print("  IMU CHECK SUMMARY")
        print("═" * 70)
        print(f"  Duration:     {elapsed:.1f} seconds")
        print(f"  Samples:      {self.sample_count}")
        print(f"  Average rate: {rate:.1f} Hz")

        if self.sample_count > 0 and self.min_vals:
            print(f"\n  ── Value Ranges (min / max) ──")
            for key in ['qw', 'qx', 'qy', 'qz']:
                avg = self.sum_vals[key] / self.sample_count
                print(f"     {key}: {self.min_vals[key]:+8.4f} / {self.max_vals[key]:+8.4f}  (avg: {avg:+8.4f})")
            for key in ['ax', 'ay', 'az']:
                avg = self.sum_vals[key] / self.sample_count
                print(f"     {key}: {self.min_vals[key]:+8.3f} / {self.max_vals[key]:+8.3f}  (avg: {avg:+8.3f})")
            for key in ['gx', 'gy', 'gz']:
                avg = self.sum_vals[key] / self.sample_count
                print(f"     {key}: {self.min_vals[key]:+8.4f} / {self.max_vals[key]:+8.4f}  (avg: {avg:+8.4f})")

            # Final health assessment
            avg_g = (self.sum_vals.get('ax', 0)**2 + self.sum_vals.get('ay', 0)**2 + self.sum_vals.get('az', 0)**2) ** 0.5 / self.sample_count
            print(f"\n  ── Final Assessment ──")
            if rate >= 15 and self.sample_count > 10:
                print(f"     ✓  IMU is working — {rate:.0f} Hz data rate is good")
            elif self.sample_count > 0:
                print(f"     ⚠  IMU rate is low ({rate:.1f} Hz) — check I²C connection")
            else:
                print(f"     ✗  No IMU data received — check BNO085 wiring")
        else:
            print(f"\n  ✗  No IMU data received!")
            print(f"     Check: VIN→5V, GND→GND, SDA→D2, SCL→D3")
            print(f"     Ensure Adafruit_BNO08x library is installed on Arduino")

        print("═" * 70)

    def close(self):
        self.stop_reading()
        if self.ser and self.ser.is_open:
            self.ser.close()

    def run(self):
        if not self.connect():
            return False

        self.start_reading()

        # Wait briefly for initial data
        time.sleep(1.0)

        try:
            self.display_loop()
        except KeyboardInterrupt:
            pass

        self.print_summary()
        self.close()
        return self.sample_count > 0


def main():
    parser = argparse.ArgumentParser(description='BNO085 IMU Health Check for Tomas_bot')
    parser.add_argument('--port', default='/dev/arduino', help='Serial port (default: /dev/arduino)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--duration', type=int, default=0, help='Duration in seconds (0 = run until Ctrl+C)')
    args = parser.parse_args()

    checker = IMUChecker(args.port, args.baud, args.duration)
    try:
        success = checker.run()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n  Interrupted.")
        checker.close()
        sys.exit(0)


if __name__ == '__main__':
    main()
