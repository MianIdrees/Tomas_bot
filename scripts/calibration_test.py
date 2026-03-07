#!/usr/bin/env python3
"""
Comprehensive movement test: measures calibration values and tests all motion types.
Compares expected vs actual, computes errors.
"""
import serial
import time
import math
import sys

PORT = '/dev/arduino'
BAUD = 115200
WHEEL_SEP = 0.181       # meters
WHEEL_RAD = 0.0345      # meters
TICKS_PER_REV = 528.0   # 11 PPR x 48:1
METERS_PER_TICK = (2.0 * math.pi * WHEEL_RAD) / TICKS_PER_REV

def open_serial():
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(2.5)
    ser.reset_input_buffer()
    return ser

def send_cmd(ser, cmd):
    ser.write(f'{cmd}\n'.encode('ascii'))

def read_encoders(ser, timeout=0.15):
    """Read latest encoder line within timeout. Returns (left, right) or None."""
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        while ser.in_waiting:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('e '):
                parts = line.split()
                if len(parts) == 3:
                    last = (int(parts[1]), int(parts[2]))
        time.sleep(0.01)
    return last

def flush_and_reset(ser):
    """Reset encoders and flush."""
    send_cmd(ser, 'r')
    time.sleep(0.3)
    ser.reset_input_buffer()

def measure_max_ticks(ser):
    """Run both motors at full PWM and measure ticks per 50ms interval.
    Uses total tick count over a fixed period for accuracy (not per-interval deltas).
    """
    print("\n" + "="*70)
    print("CALIBRATION: Measuring max ticks per 50ms at PWM 255")
    print("="*70)
    
    flush_and_reset(ser)
    
    # Ramp up to full speed using raw PWM ('w' bypasses PID)
    print("Ramping up to PWM 255 (raw mode)...")
    for pwm in [50, 100, 150, 200, 255]:
        send_cmd(ser, f'w {pwm} {pwm}')
        time.sleep(0.3)
    
    # Wait for steady state at full speed
    time.sleep(1.0)
    
    # Reset encoders NOW (after reaching steady state)
    flush_and_reset(ser)
    time.sleep(0.1)
    ser.reset_input_buffer()
    
    # Keep at full speed for exactly 3 seconds, measuring total ticks
    measure_duration = 3.0
    send_cmd(ser, 'w 255 255')
    start = time.time()
    while time.time() - start < measure_duration:
        send_cmd(ser, 'w 255 255')
        time.sleep(0.08)
    
    # Stop motors (use 'w 0 0' which also sets raw mode, then 'm 0 0' to re-enable PID)
    send_cmd(ser, 'w 0 0')
    time.sleep(0.5)
    send_cmd(ser, 'm 0 0')
    time.sleep(1.0)  # let motors fully stop
    
    # Read final encoder value — these are total ticks since the reset
    enc = read_encoders(ser, 0.5)
    if enc is None:
        send_cmd(ser, 'm 0 0')
        time.sleep(0.3)
        enc = read_encoders(ser, 0.5)
    
    send_cmd(ser, 'm 0 0')
    
    if enc is None:
        print("ERROR: Could not read encoder values!")
        return 40, 40
    
    total_left = abs(enc[0])
    total_right = abs(enc[1])
    
    # Number of 50ms intervals in the measurement period
    num_intervals = measure_duration / 0.050  # 60 intervals in 3 seconds
    
    left_max = total_left / num_intervals
    right_max = total_right / num_intervals
    
    print(f"  Measurement period: {measure_duration}s ({num_intervals:.0f} intervals)")
    print(f"  Total ticks: Left={total_left}, Right={total_right}")
    print(f"  Left  ticks/50ms: {left_max:.1f}")
    print(f"  Right ticks/50ms: {right_max:.1f}")
    print(f"  Left  RPM:  {left_max * 20 * 60 / TICKS_PER_REV:.0f}")
    print(f"  Right RPM:  {right_max * 20 * 60 / TICKS_PER_REV:.0f}")
    print(f"  Left  max speed:  {left_max * 20 * METERS_PER_TICK:.3f} m/s")
    print(f"  Right max speed:  {right_max * 20 * METERS_PER_TICK:.3f} m/s")
    
    return left_max, right_max


def run_motion_test(ser, name, left_pwm, right_pwm, duration,
                    expected_x, expected_y, expected_yaw_deg):
    """Run a single motion test, measure actual displacement."""
    print(f"\n{'─'*70}")
    print(f"TEST: {name}")
    print(f"  Command: m {left_pwm} {right_pwm} for {duration}s")
    print(f"  Expected: x={expected_x:.3f}m, y={expected_y:.3f}m, yaw={expected_yaw_deg:.1f}°")
    print(f"{'─'*70}")
    
    flush_and_reset(ser)
    time.sleep(0.3)
    
    # Enable debug for this test
    send_cmd(ser, 'd')
    time.sleep(0.1)
    
    # Collect PID debug data
    debug_lines = []
    
    # Run motion
    start = time.time()
    while time.time() - start < duration:
        send_cmd(ser, f'm {left_pwm} {right_pwm}')
        # Read any available data
        while ser.in_waiting:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('d '):
                debug_lines.append((time.time() - start, line))
        time.sleep(0.04)
    
    # Stop and wait for coast-down
    send_cmd(ser, 'm 0 0')
    time.sleep(1.0)
    
    # Read final encoder values
    enc = read_encoders(ser, 0.3)
    if enc is None:
        # Try harder
        send_cmd(ser, 'm 0 0')
        time.sleep(0.2)
        enc = read_encoders(ser, 0.5)
    
    if enc is None:
        print("  ERROR: Could not read final encoder values!")
        return None
    
    left_ticks_total = enc[0]
    right_ticks_total = enc[1]
    
    # Compute actual displacement from encoder ticks
    dist_left = left_ticks_total * METERS_PER_TICK
    dist_right = right_ticks_total * METERS_PER_TICK
    dist_center = (dist_right + dist_left) / 2.0
    delta_theta = (dist_right - dist_left) / WHEEL_SEP
    
    # Dead-reckoning position
    if abs(delta_theta) < 1e-6:
        actual_x = dist_center
        actual_y = 0.0
    else:
        radius = dist_center / delta_theta
        actual_x = radius * math.sin(delta_theta)
        actual_y = radius * (1 - math.cos(delta_theta))
    actual_yaw_deg = math.degrees(delta_theta)
    
    # Compute errors
    err_x = actual_x - expected_x
    err_y = actual_y - expected_y
    err_yaw = actual_yaw_deg - expected_yaw_deg
    dist_actual = math.sqrt(actual_x**2 + actual_y**2)
    dist_expected = math.sqrt(expected_x**2 + expected_y**2)
    
    print(f"\n  Raw ticks:  Left={left_ticks_total:+d}, Right={right_ticks_total:+d}")
    print(f"  Distances:  Left={dist_left:.4f}m, Right={dist_right:.4f}m")
    print(f"  Actual:     x={actual_x:.4f}m, y={actual_y:.4f}m, yaw={actual_yaw_deg:.2f}°")
    print(f"  Expected:   x={expected_x:.4f}m, y={expected_y:.4f}m, yaw={expected_yaw_deg:.2f}°")
    print(f"  Errors:     x={err_x:+.4f}m, y={err_y:+.4f}m, yaw={err_yaw:+.2f}°")
    if dist_expected > 0.01:
        pct = (dist_actual - dist_expected) / dist_expected * 100
        print(f"  Distance %: {pct:+.1f}%")
    if expected_yaw_deg != 0:
        yaw_pct = (actual_yaw_deg - expected_yaw_deg) / expected_yaw_deg * 100
        print(f"  Yaw %:      {yaw_pct:+.1f}%")
    
    # Show PID tracking quality from debug data
    if debug_lines:
        print(f"\n  PID samples ({len(debug_lines)} captured):")
        # Show a few representative samples
        step = max(1, len(debug_lines) // 5)
        for i in range(0, len(debug_lines), step):
            t, line = debug_lines[i]
            print(f"    t={t:.2f}s  {line}")
    
    return {
        'name': name,
        'left_ticks': left_ticks_total,
        'right_ticks': right_ticks_total,
        'actual_x': actual_x, 'actual_y': actual_y, 'actual_yaw': actual_yaw_deg,
        'expected_x': expected_x, 'expected_y': expected_y, 'expected_yaw': expected_yaw_deg,
        'err_x': err_x, 'err_y': err_y, 'err_yaw': err_yaw,
    }


def main():
    ser = open_serial()
    print("Connected to Arduino.")
    
    # Wait for IMU init
    time.sleep(1.0)
    ser.reset_input_buffer()
    
    # ── Step 1: Measure actual max ticks ──
    left_max, right_max = measure_max_ticks(ser)
    
    # Compute actual max speed
    max_speed_left = left_max * 20 * METERS_PER_TICK   # ticks/50ms * 20Hz * m/tick
    max_speed_right = right_max * 20 * METERS_PER_TICK
    slower_max = min(max_speed_left, max_speed_right)
    
    print(f"\n  >>> Firmware MAX_TICKS_PER_INTERVAL should be: {min(left_max, right_max):.0f}")
    print(f"  >>> LEFT_MAX_TICKS should be: {left_max:.0f}")
    print(f"  >>> RIGHT_MAX_TICKS should be: {right_max:.0f}")
    print(f"  >>> diff_drive_node max_motor_speed should be: {slower_max:.3f} m/s")
    
    # ── Step 2: Motion tests ──
    # Use measured max_ticks to compute correct PWM and expected values.
    # max_ticks = ticks per 50ms at PWM 255
    # At PWM p: ticks_per_50ms = (p/255) * max_ticks
    # Speed at PWM p: v = (p/255) * max_ticks * 20 * METERS_PER_TICK
    avg_max = (left_max + right_max) / 2.0
    max_speed = avg_max * 20 * METERS_PER_TICK  # m/s at PWM 255

    test_duration = 2.0  # seconds for each motion

    # Forward: target 0.15 m/s (or less if max_speed is low)
    fwd_speed = min(0.15, max_speed * 0.5)
    fwd_pwm = int(fwd_speed / max_speed * 255)
    fwd_expected_dist = fwd_speed * test_duration

    # Backward: target 0.10 m/s
    bwd_speed = min(0.10, max_speed * 0.3)
    bwd_pwm = -int(bwd_speed / max_speed * 255)
    bwd_expected_dist = bwd_speed * test_duration

    # Rotation: compute PWM for ~90° in rot_duration seconds
    rot_duration = 1.5
    desired_yaw_rad = math.pi / 2.0  # 90°
    # w = 2*v_wheel / WHEEL_SEP → v_wheel = w * WHEEL_SEP / 2
    w_needed = desired_yaw_rad / rot_duration  # rad/s
    v_wheel_needed = w_needed * WHEEL_SEP / 2.0  # m/s per wheel
    rot_pwm = max(35, int(v_wheel_needed / max_speed * 255))

    print(f"\n  Computed test parameters:")
    print(f"    Forward:  PWM={fwd_pwm}, target speed={fwd_speed:.3f} m/s, expected dist={fwd_expected_dist:.3f}m")
    print(f"    Backward: PWM={bwd_pwm}, target speed={bwd_speed:.3f} m/s, expected dist={bwd_expected_dist:.3f}m")
    print(f"    Rotation: PWM={rot_pwm}, v_wheel={v_wheel_needed:.3f} m/s, expected yaw=90°")
    
    print("\n\n" + "="*70)
    print("MOVEMENT TESTS")
    print("="*70)
    
    results = []
    
    time.sleep(1.0)
    
    # Test 1: Forward
    r = run_motion_test(ser, f"FORWARD ({fwd_speed:.2f} m/s × {test_duration}s)",
                        fwd_pwm, fwd_pwm, test_duration,
                        expected_x=fwd_expected_dist, expected_y=0.0, expected_yaw_deg=0.0)
    if r: results.append(r)
    time.sleep(2.0)
    
    # Test 2: Backward
    r = run_motion_test(ser, f"BACKWARD ({bwd_speed:.2f} m/s × {test_duration}s)",
                        bwd_pwm, bwd_pwm, test_duration,
                        expected_x=-bwd_expected_dist, expected_y=0.0, expected_yaw_deg=0.0)
    if r: results.append(r)
    time.sleep(2.0)
    
    # Test 3: Rotate Left (CCW) — left wheel backward, right wheel forward
    r = run_motion_test(ser, f"ROTATE LEFT (CCW) × {rot_duration}s",
                        -rot_pwm, rot_pwm, rot_duration,
                        expected_x=0.0, expected_y=0.0, expected_yaw_deg=90.0)
    if r: results.append(r)
    time.sleep(2.0)
    
    # Test 4: Rotate Right (CW) — left wheel forward, right wheel backward
    r = run_motion_test(ser, f"ROTATE RIGHT (CW) × {rot_duration}s",
                        rot_pwm, -rot_pwm, rot_duration,
                        expected_x=0.0, expected_y=0.0, expected_yaw_deg=-90.0)
    if r: results.append(r)
    time.sleep(1.0)
    
    # ── Summary ──
    print("\n\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print(f"{'Test':<35} {'Exp Yaw':>8} {'Act Yaw':>8} {'Yaw Err':>8} | {'Exp Dist':>8} {'Act Dist':>9} {'%Err':>6}")
    print("─"*70)
    for r in results:
        exp_dist = math.sqrt(r['expected_x']**2 + r['expected_y']**2)
        act_dist = math.sqrt(r['actual_x']**2 + r['actual_y']**2)
        dist_pct = ((act_dist - exp_dist) / exp_dist * 100) if exp_dist > 0.01 else 0
        print(f"{r['name']:<35} {r['expected_yaw']:>7.1f}° {r['actual_yaw']:>7.1f}° {r['err_yaw']:>+7.1f}° | "
              f"{exp_dist:>7.3f}m {act_dist:>8.3f}m {dist_pct:>+5.1f}%")
    
    # Diagnose key issues
    print(f"\n{'─'*70}")
    print("DIAGNOSIS:")
    
    # Check wheel speed asymmetry
    if results:
        fwd = results[0]
        if fwd['left_ticks'] != 0:
            ratio = fwd['right_ticks'] / fwd['left_ticks']
            print(f"  Wheel speed ratio (R/L): {ratio:.3f}  (should be ~1.00)")
            if abs(ratio - 1.0) > 0.05:
                print(f"  ⚠ Right motor is {(1-ratio)*100:+.1f}% different from left!")
    
    # Check if PID is achieving target
    for r in results:
        if 'FORWARD' in r['name']:
            exp = abs(r['expected_x'])
            act = abs(r['actual_x'])
            if exp > 0:
                pct = act / exp * 100
                print(f"  Forward: achieved {pct:.0f}% of expected distance")
                if pct < 80:
                    print(f"  ⚠ PID not reaching target! MAX_TICKS may need calibration")
    
    send_cmd(ser, 'm 0 0')
    ser.close()
    print("\nDone.")


if __name__ == '__main__':
    main()
