/* code updated
 * ============================================================================
 * Arduino Leonardo — Motor + Encoder + BNO085 IMU Controller for ROS2
 * ============================================================================
 * Target: Arduino Leonardo (ATmega32U4) built into LattePanda Alpha
 *
 * Motors: 130 RPM 12V DC motors with Quadrature Encoders
 *   - Encoder: 11 pulses per motor shaft rotation
 *   - Gear ratio: ~48:1 (typical for 130 RPM motors) — CALIBRATE THIS
 *   - Default ticks per output revolution: 11 × 48 = 528
 *   - Wheels: 69mm diameter (0.0345m radius)
 *   - Max speed: ~0.47 m/s at 12V no-load
 *
 * Motor Driver: L298N H-Bridge
 *   - Power: 6–12V input
 *
 * IMU: Adafruit BNO085 (9-DOF) via I²C
 *   - I²C Address: 0x4A (default)
 *   - Reports: Game Rotation Vector + Gyroscope + Accelerometer
 *   - Uses minimal SHTP I2C driver (NOT Adafruit library — saves ~1KB SRAM)
 *
 * Pin Assignments:
 *   L298N Motor Driver:
 *     ENA → D5  (PWM, left motor speed)
 *     IN1 → D7  (left motor direction)
 *     IN2 → D6  (left motor direction)
 *     IN3 → D10 (right motor direction)
 *     IN4 → D9  (right motor direction)
 *     ENB → D11 (PWM, right motor speed)
 *
 *   Left Motor Encoder (MOVED for I²C — see Integration Guide):
 *     Green  → D1  (Channel A — hardware interrupt INT3)
 *     Yellow → D0  (Channel B — direction sensing)
 *
 *   Right Motor Encoder:
 *     Yellow → A4  (Channel A — polled, no hardware interrupt)
 *     Green  → A5  (Channel B — direction sensing)
 *
 *   BNO085 IMU (I²C on D2/D3):
 *     VIN → 5V,  GND → GND
 *     SDA → D2,  SCL → D3
 *     INT → D4 (optional, data-ready interrupt)
 *
 * Serial Protocol (115200 baud, USB CDC):
 *   Commands FROM ROS2 (LattePanda → Leonardo):
 *     m <left_pwm> <right_pwm>\n
 *       Values: -255 to 255 (positive = forward)
 *
 *   Feedback TO ROS2 (Leonardo → LattePanda):
 *     e <left_ticks> <right_ticks>\n
 *       Cumulative signed encoder ticks, sent every 50ms (20 Hz)
 *     i <qw> <qx> <qy> <qz> <ax> <ay> <az> <gx> <gy> <gz>\n
 *       IMU data: quaternion + linear accel (m/s²) + gyro (rad/s), 50ms
 *
 *   Reset command:
 *     r\n  — Resets encoder counts to zero
 *
 *   Debug command:
 *     d\n  — Enables PID debug output for 5 seconds
 *
 * NOTE: On Arduino Leonardo, Serial = USB CDC (native USB).
 *       The built-in Leonardo on LattePanda Alpha connects directly
 *       via internal USB, no external USB-to-serial adapter needed.
 *
 * ============================================================================
 */

#include <Wire.h>

// Forward declaration for Arduino auto-prototype generation
struct MotorPID;

// ========================== MINIMAL BNO085 SHTP I2C DRIVER ==========================
// Replaces the Adafruit_BNO08x library to fit within ATmega32U4's 2.5KB SRAM.
// Only implements: reset, enable 3 reports, read 3 report types.
// SHTP = Sensor Hub Transport Protocol (Hillcrest/CEVA)

#define BNO085_I2C_ADDR  0x4A

// SHTP channel numbers
#define SHTP_CHAN_COMMAND   0
#define SHTP_CHAN_EXE       1
#define SHTP_CHAN_CONTROL   2
#define SHTP_CHAN_REPORTS   3

// SH2 report IDs
#define SH2_TIMESTAMP_REBASE       0xFB
#define SH2_SET_FEATURE_CMD        0xFD
#define SH2_ACCELEROMETER          0x01
#define SH2_GYROSCOPE_CALIBRATED   0x02
#define SH2_GAME_ROTATION_VECTOR   0x08

// Shared I2C read buffer (28 payload + 4 header = 32 = AVR Wire max)
#define BNO_BUF_SIZE  32
static uint8_t bno_buf[BNO_BUF_SIZE];
static uint8_t bno_seq[4];  // Per-channel SHTP sequence numbers

// Send an SHTP packet to the BNO085
static bool bno_sendPacket(uint8_t channel, const uint8_t *data, uint8_t dataLen) {
    uint16_t totalLen = dataLen + 4;
    Wire.beginTransmission(BNO085_I2C_ADDR);
    Wire.write((uint8_t)(totalLen & 0xFF));
    Wire.write((uint8_t)(totalLen >> 8));
    Wire.write(channel);
    Wire.write(bno_seq[channel]++);
    for (uint8_t i = 0; i < dataLen; i++) {
        Wire.write(data[i]);
    }
    return (Wire.endTransmission() == 0);
}

// Read one SHTP packet. Returns payload length on sensor report channel, 0 otherwise.
// Payload data is stored in bno_buf[0..N-1].
static uint8_t bno_readPacket() {
    uint8_t n = Wire.requestFrom((uint8_t)BNO085_I2C_ADDR, (uint8_t)BNO_BUF_SIZE);
    if (n < 4) {
        while (Wire.available()) Wire.read();
        return 0;
    }

    uint8_t len_lo  = Wire.read();
    uint8_t len_hi  = Wire.read();
    uint8_t channel = Wire.read();
    /*uint8_t seq =*/ Wire.read();

    uint16_t pktLen = ((uint16_t)(len_hi & 0x7F) << 8) | len_lo;
    if (pktLen < 5 || pktLen > 300) {
        while (Wire.available()) Wire.read();
        return 0;
    }

    uint8_t available = n - 4;
    uint16_t dataLen  = pktLen - 4;
    uint8_t toRead = (available < dataLen) ? available : (uint8_t)dataLen;
    if (toRead > BNO_BUF_SIZE) toRead = BNO_BUF_SIZE;

    for (uint8_t i = 0; i < toRead; i++) {
        bno_buf[i] = Wire.read();
    }
    while (Wire.available()) Wire.read();

    // Only return data for sensor report channel
    if (channel != SHTP_CHAN_REPORTS) return 0;
    return toRead;
}

// Enable a sensor report at a given interval (microseconds)
static void bno_enableReport(uint8_t reportId, uint32_t interval_us) {
    uint8_t cmd[17];
    memset(cmd, 0, 17);
    cmd[0] = SH2_SET_FEATURE_CMD;
    cmd[1] = reportId;
    // Bytes 2-4: flags + change sensitivity = 0
    cmd[5] = (uint8_t)(interval_us & 0xFF);
    cmd[6] = (uint8_t)((interval_us >> 8) & 0xFF);
    cmd[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    cmd[8] = (uint8_t)((interval_us >> 24) & 0xFF);
    // Bytes 9-16: batch interval + sensor config = 0
    bno_sendPacket(SHTP_CHAN_CONTROL, cmd, 17);
}

// ========================== PIN DEFINITIONS ==========================

// Left Motor (L298N)
#define LEFT_ENA   5    // PWM speed control
#define LEFT_IN1   7    // Direction
#define LEFT_IN2   6    // Direction

// Right Motor (L298N)
#define RIGHT_IN3  10   // Direction
#define RIGHT_IN4  9    // Direction
#define RIGHT_ENB  11   // PWM speed control

// Left Encoder — MOVED from D3/D2 to D1/D0 to free I²C bus
#define LEFT_ENC_A   1  // Green wire → INT3 on Leonardo (moved from D3)
#define LEFT_ENC_B   0  // Yellow wire → direction sensing (moved from D2)

// Right Encoder (polled — A4/A5 have no hardware interrupt on ATmega32U4)
#define RIGHT_ENC_A  A4  // Yellow wire → polled channel A
#define RIGHT_ENC_B  A5  // Green wire  → direction sensing

// BNO085 IMU (I²C on hardware SDA=D2, SCL=D3)
#define BNO085_INT_PIN  4   // Optional data-ready interrupt

// ========================== CONFIGURATION ==========================

#define SERIAL_BAUD         115200
#define PUBLISH_INTERVAL_MS    50   // Encoder data publish rate (20 Hz)
#define PID_INTERVAL_MS        50   // PID update rate
#define CMD_TIMEOUT_MS        500   // Stop motors if no command for 500ms
#define SERIAL_BUF_SIZE        32

// ========================== PID SPEED CONTROL ==========================
/*
 * PID controller equalizes wheel speeds using encoder feedback.
 * The 'm' command value (-255 to 255) is treated as a target speed.
 * PID adjusts actual PWM to match target speed.
 *
 * Calibration values — MUST BE MEASURED on actual motors:
 *   Run each motor at PWM=255 and count ticks per PID interval (50ms).
 *   Set LEFT_MAX_TICKS and RIGHT_MAX_TICKS accordingly.
 *   Start with conservative estimates and tune.
 */
#define MAX_TICKS_PER_INTERVAL  48    // Target scaling (measured at raw PWM 255, 3s avg)
#define LEFT_MAX_TICKS          48    // Measured: left motor ticks at raw PWM 255 per 50ms
#define RIGHT_MAX_TICKS         48    // Measured: right motor ticks at raw PWM 255 per 50ms

// Minimum PWM to overcome motor stiction and L298N voltage drop
// MUST MATCH diff_drive_node.py min_pwm parameter (tuned: 57)
// Heavy 2.4kg robot: dead zone measured at PWM 52, +5 safety margin
// The state machine relies on the Arduino PID to completely
// handle speeds at or above MIN_PWM, while sub-MIN speeds are
// handled by duty cycling in diff_drive_node.py
#define MIN_PWM  57

// PID gains — start conservative, tune on real hardware
#define KP   1.0
#define KI   0.8
#define KD   0.15
#define INTEGRAL_LIMIT  150.0

// Target ramping for smooth acceleration
#define RAMP_RATE  10.0f  // ticks/interval per PID cycle (fast response; PID handles smooth output)

// ========================== PID STATE ==========================

struct MotorPID {
    float target;          // Current (ramped) target ticks per interval
    float cmd_target;      // Commanded target (from serial)
    float integral;        // Accumulated error
    float prev_error;      // Previous error for derivative
    int   output_pwm;      // Actual PWM applied
};

MotorPID left_pid  = {0, 0, 0, 0, 0};
MotorPID right_pid = {0, 0, 0, 0, 0};

long prev_left_ticks  = 0;
long prev_right_ticks = 0;

int debug_countdown = 0;
bool raw_pwm_mode = false;  // 'w' command bypasses PID

// ========================== ENCODER STATE ==========================

// Left encoder — updated by hardware interrupt (INT3 on D1)
volatile long left_ticks  = 0;

// Right encoder — updated by Pin Change Interrupt (PCINT1 on A4)
// ATmega32U4: A4 = PF1 = PCINT1 (PCINT0 group covers PF0-PF7)
volatile long right_ticks = 0;

// ========================== TIMING ==========================

unsigned long last_publish_time = 0;
unsigned long last_pid_time     = 0;
unsigned long last_cmd_time     = 0;

// ========================== SERIAL BUFFER ==========================

char serial_buf[SERIAL_BUF_SIZE];
int  serial_idx = 0;

// ========================== BNO085 IMU STATE ==========================

bool imu_available = false;

// Latest IMU readings
float imu_qw = 1.0f, imu_qx = 0.0f, imu_qy = 0.0f, imu_qz = 0.0f;
float imu_ax = 0.0f, imu_ay = 0.0f, imu_az = 0.0f;
float imu_gx = 0.0f, imu_gy = 0.0f, imu_gz = 0.0f;

unsigned long last_imu_publish_time = 0;
#define IMU_PUBLISH_INTERVAL_MS  50  // 20 Hz, aligned with encoder rate

void initIMU() {
    // Check if BNO085 is on the I²C bus
    Wire.beginTransmission(BNO085_I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println(F("! BNO085 not detected on I2C 0x4A"));
        imu_available = false;
        return;
    }

    // Soft reset: send reset command on executable channel
    uint8_t resetCmd = 1;
    bno_sendPacket(SHTP_CHAN_EXE, &resetCmd, 1);
    delay(300);

    // Flush boot advertisement and initialization packets
    for (uint8_t i = 0; i < 20; i++) {
        bno_readPacket();
        delay(20);
    }

    Serial.println(F("BNO085 found on I2C 0x4A"));

    // Enable the 3 reports we need at 20Hz (50000 µs interval)
    bno_enableReport(SH2_GAME_ROTATION_VECTOR, 50000);
    delay(20);
    bno_enableReport(SH2_ACCELEROMETER, 50000);
    delay(20);
    bno_enableReport(SH2_GYROSCOPE_CALIBRATED, 50000);
    delay(20);

    imu_available = true;
    Serial.println(F("BNO085 reports enabled (RotVec+Accel+Gyro @20Hz)"));
}

void readIMU() {
    if (!imu_available) return;

    // Read up to 3 packets per call (one per report type at most)
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        uint8_t len = bno_readPacket();
        if (len == 0) break;

        // Parse sensor reports from payload
        uint8_t pos = 0;
        while (pos < len) {
            uint8_t id = bno_buf[pos];

            if (id == SH2_TIMESTAMP_REBASE && pos + 5 <= len) {
                pos += 5;  // Skip 5-byte timestamp rebase
            }
            else if (id == SH2_GAME_ROTATION_VECTOR && pos + 12 <= len) {
                // 4-byte header (id, seq, status, delay) + 8 bytes data
                int16_t qi = (int16_t)((uint16_t)bno_buf[pos+4] | ((uint16_t)bno_buf[pos+5] << 8));
                int16_t qj = (int16_t)((uint16_t)bno_buf[pos+6] | ((uint16_t)bno_buf[pos+7] << 8));
                int16_t qk = (int16_t)((uint16_t)bno_buf[pos+8] | ((uint16_t)bno_buf[pos+9] << 8));
                int16_t qr = (int16_t)((uint16_t)bno_buf[pos+10] | ((uint16_t)bno_buf[pos+11] << 8));
                imu_qx = qi / 16384.0f;  // Q14
                imu_qy = qj / 16384.0f;
                imu_qz = qk / 16384.0f;
                imu_qw = qr / 16384.0f;
                pos += 12;
            }
            else if (id == SH2_ACCELEROMETER && pos + 10 <= len) {
                // 4-byte header + 6 bytes data
                int16_t ax = (int16_t)((uint16_t)bno_buf[pos+4] | ((uint16_t)bno_buf[pos+5] << 8));
                int16_t ay = (int16_t)((uint16_t)bno_buf[pos+6] | ((uint16_t)bno_buf[pos+7] << 8));
                int16_t az = (int16_t)((uint16_t)bno_buf[pos+8] | ((uint16_t)bno_buf[pos+9] << 8));
                imu_ax = ax / 256.0f;  // Q8, m/s²
                imu_ay = ay / 256.0f;
                imu_az = az / 256.0f;
                pos += 10;
            }
            else if (id == SH2_GYROSCOPE_CALIBRATED && pos + 10 <= len) {
                // 4-byte header + 6 bytes gyro data (10 bytes total)
                // Note: Only the Uncalibrated Gyroscope (0x07) has bias fields (16 bytes).
                //       The Calibrated Gyroscope (0x02) is 10 bytes.
                int16_t gx = (int16_t)((uint16_t)bno_buf[pos+4] | ((uint16_t)bno_buf[pos+5] << 8));
                int16_t gy = (int16_t)((uint16_t)bno_buf[pos+6] | ((uint16_t)bno_buf[pos+7] << 8));
                int16_t gz = (int16_t)((uint16_t)bno_buf[pos+8] | ((uint16_t)bno_buf[pos+9] << 8));
                imu_gx = gx / 512.0f;  // Q9, rad/s
                imu_gy = gy / 512.0f;
                imu_gz = gz / 512.0f;
                pos += 10;
            }
            else {
                break;  // Unknown report or not enough data
            }
        }
    }
}

void publishIMU() {
    if (!imu_available) return;
    // Format: i <qw> <qx> <qy> <qz> <ax> <ay> <az> <gx> <gy> <gz>
    Serial.print(F("i "));
    Serial.print(imu_qw, 4); Serial.print(' ');
    Serial.print(imu_qx, 4); Serial.print(' ');
    Serial.print(imu_qy, 4); Serial.print(' ');
    Serial.print(imu_qz, 4); Serial.print(' ');
    Serial.print(imu_ax, 4); Serial.print(' ');
    Serial.print(imu_ay, 4); Serial.print(' ');
    Serial.print(imu_az, 4); Serial.print(' ');
    Serial.print(imu_gx, 4); Serial.print(' ');
    Serial.print(imu_gy, 4); Serial.print(' ');
    Serial.println(imu_gz, 4);
}

// ========================== LEFT ENCODER ISR ==========================
// Hardware interrupt on D1 (INT3 on Leonardo), RISING edge
// NOTE: Moved from D3/INT0 to D1/INT3 to free D2/D3 for I²C (BNO085)

void leftEncoderISR() {
    if (digitalRead(LEFT_ENC_B) == HIGH) {
        left_ticks--;
    } else {
        left_ticks++;
    }
}

// ========================== RIGHT ENCODER — TIMER1 ISR POLLING ==========================
// ATmega32U4 has NO Pin Change Interrupts on Port F (A4/A5).
// (PCINT0 group covers Port B only — different from ATmega328P!)
// Instead, we use Timer1 Compare Match A to poll A4 at ~2 kHz,
// fast enough to catch every encoder edge even during blocking I2C.
// Timer1 is free on Leonardo when pins 9/10 are used as digital outputs.

static volatile uint8_t right_enc_a_prev = 0;

ISR(TIMER1_COMPA_vect) {
    uint8_t current = PINF & _BV(PF1);  // Read A4 (PF1)
    if (current && !right_enc_a_prev) {
        // Rising edge — check A5 (PF0) for direction
        if (PINF & _BV(PF0)) {
            right_ticks--;
        } else {
            right_ticks++;
        }
    }
    right_enc_a_prev = current;
}

// ========================== MOTOR CONTROL ==========================

void setMotor(int pwm, int ena_pin, int in1_pin, int in2_pin) {
    if (pwm > 0) {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
        analogWrite(ena_pin, constrain(pwm, 0, 255));
    } else if (pwm < 0) {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
        analogWrite(ena_pin, constrain(-pwm, 0, 255));
    } else {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
        analogWrite(ena_pin, 0);
    }
}

void setMotors(int left_pwm, int right_pwm) {
    setMotor(left_pwm,  LEFT_ENA,  LEFT_IN1,  LEFT_IN2);
    setMotor(right_pwm, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4);
}

void stopMotors() {
    raw_pwm_mode = false;
    left_pid.target = 0;
    left_pid.cmd_target = 0;
    left_pid.integral = 0;
    left_pid.prev_error = 0;
    left_pid.output_pwm = 0;
    right_pid.target = 0;
    right_pid.cmd_target = 0;
    right_pid.integral = 0;
    right_pid.prev_error = 0;
    right_pid.output_pwm = 0;
    setMotors(0, 0);
}

// ========================== PID UPDATE ==========================

void updateMotorPID(MotorPID &pid, long delta_ticks, int motor_max_ticks,
                    int ena_pin, int in1_pin, int in2_pin, unsigned long dt_ms) {
    // Ramp target toward cmd_target
    bool is_ramping = false;
    if (pid.target < pid.cmd_target - 0.5f) {
        pid.target += RAMP_RATE;
        if (pid.target > pid.cmd_target) pid.target = pid.cmd_target;
        is_ramping = true;
    } else if (pid.target > pid.cmd_target + 0.5f) {
        pid.target -= RAMP_RATE;
        if (pid.target < pid.cmd_target) pid.target = pid.cmd_target;
        is_ramping = true;
    } else {
        pid.target = pid.cmd_target;
    }

    // During ramping, keep integral at zero to prevent accumulation
    if (is_ramping) {
        pid.integral = 0;
        pid.prev_error = 0;
    }

    // If target is zero, stop immediately
    if (pid.target > -0.5f && pid.target < 0.5f) {
        pid.output_pwm = 0;
        pid.integral = 0;
        pid.prev_error = 0;
        setMotor(0, ena_pin, in1_pin, in2_pin);
        return;
    }

    // Normalize measurement to nominal PID interval.
    // If readIMU() blocking extended this interval, scale ticks down so
    // the PID doesn't think the motor is going too fast.
    float actual = (float)delta_ticks;
    if (dt_ms > 0 && dt_ms != PID_INTERVAL_MS) {
        actual = actual * (float)PID_INTERVAL_MS / (float)dt_ms;
    }
    float error = pid.target - actual;

    // Derivative
    float derivative = error - pid.prev_error;
    pid.prev_error = error;

    // Per-motor feedforward
    float ff_ratio = pid.target / (float)motor_max_ticks;
    int feedforward = (int)(ff_ratio * 255.0f);

    // Tentative integral with anti-windup
    float new_integral = pid.integral + error;
    if (new_integral > INTEGRAL_LIMIT)  new_integral = INTEGRAL_LIMIT;
    if (new_integral < -INTEGRAL_LIMIT) new_integral = -INTEGRAL_LIMIT;

    // Compute output
    float correction = KP * error + KI * new_integral + KD * derivative;
    int raw_output = feedforward + (int)correction;

    // Anti-windup: only commit integral if output isn't saturated
    if ((raw_output <= 255 && raw_output >= -255) ||
        (error < 0 && pid.integral > 0) ||
        (error > 0 && pid.integral < 0)) {
        pid.integral = new_integral;
    }

    // Recompute with actual integral
    correction = KP * error + KI * pid.integral + KD * derivative;
    pid.output_pwm = feedforward + (int)correction;

    // Clamp PWM
    if (pid.output_pwm > 255)  pid.output_pwm = 255;
    if (pid.output_pwm < -255) pid.output_pwm = -255;

    // Dead-zone compensation: only boost to MIN_PWM when the PID is trying
    // to produce motion (feedforward >= MIN_PWM) or when undershooting (error > 0).
    // For very low targets where feedforward < MIN_PWM, let the PID output
    // drop to 0 when overshooting — this creates natural duty-cycling between
    // 0 and MIN_PWM that averages to the correct speed.
    if (pid.output_pwm > 0 && pid.output_pwm < MIN_PWM) {
        if (feedforward >= MIN_PWM || error > 0) {
            pid.output_pwm = MIN_PWM;
        } else {
            pid.output_pwm = 0;
        }
    }
    if (pid.output_pwm < 0 && pid.output_pwm > -MIN_PWM) {
        if (feedforward <= -MIN_PWM || error < 0) {
            pid.output_pwm = -MIN_PWM;
        } else {
            pid.output_pwm = 0;
        }
    }

    setMotor(pid.output_pwm, ena_pin, in1_pin, in2_pin);
}

void updatePID() {
    if (raw_pwm_mode) return;  // Skip PID in raw PWM mode

    // Compute actual elapsed time since last PID update.
    // readIMU() I2C blocking can stretch intervals beyond 50ms.
    unsigned long now = millis();
    unsigned long dt_ms = now - last_pid_time;
    last_pid_time = now;

    noInterrupts();
    long l = left_ticks;
    long r = right_ticks;
    interrupts();

    long delta_left  = l - prev_left_ticks;
    long delta_right = r - prev_right_ticks;
    prev_left_ticks  = l;
    prev_right_ticks = r;

    updateMotorPID(left_pid,  delta_left,  LEFT_MAX_TICKS,
                   LEFT_ENA,  LEFT_IN1,  LEFT_IN2, dt_ms);
    updateMotorPID(right_pid, delta_right, RIGHT_MAX_TICKS,
                   RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, dt_ms);

    // Debug output
    if (debug_countdown > 0) {
        debug_countdown--;
        Serial.print(F("d L:"));
        Serial.print(left_pid.target, 1);
        Serial.print(',');
        Serial.print(delta_left);
        Serial.print(',');
        Serial.print(left_pid.integral, 1);
        Serial.print(',');
        Serial.print(left_pid.output_pwm);
        Serial.print(F(" R:"));
        Serial.print(right_pid.target, 1);
        Serial.print(',');
        Serial.print(delta_right);
        Serial.print(',');
        Serial.print(right_pid.integral, 1);
        Serial.print(',');
        Serial.println(right_pid.output_pwm);
    }
}

// ========================== SERIAL COMMAND PARSING ==========================

void processCommand(const char* cmd) {
    if (cmd[0] == 'm') {
        raw_pwm_mode = false;
        int left_pwm = 0, right_pwm = 0;
        if (sscanf(cmd + 1, "%d %d", &left_pwm, &right_pwm) == 2) {
            left_pwm  = constrain(left_pwm,  -255, 255);
            right_pwm = constrain(right_pwm, -255, 255);
            left_pid.cmd_target  = (left_pwm  / 255.0f) * MAX_TICKS_PER_INTERVAL;
            right_pid.cmd_target = (right_pwm / 255.0f) * MAX_TICKS_PER_INTERVAL;
            last_cmd_time = millis();
        }
    } else if (cmd[0] == 'w') {
        // Raw PWM mode — bypass PID, drive motors directly (for calibration)
        int left_pwm = 0, right_pwm = 0;
        if (sscanf(cmd + 1, "%d %d", &left_pwm, &right_pwm) == 2) {
            raw_pwm_mode = true;
            left_pwm  = constrain(left_pwm,  -255, 255);
            right_pwm = constrain(right_pwm, -255, 255);
            setMotors(left_pwm, right_pwm);
            last_cmd_time = millis();
        }
    } else if (cmd[0] == 'r') {
        noInterrupts();
        left_ticks  = 0;
        right_ticks = 0;
        interrupts();
        prev_left_ticks  = 0;
        prev_right_ticks = 0;
        left_pid.integral = 0;
        left_pid.prev_error = 0;
        right_pid.integral = 0;
        right_pid.prev_error = 0;
        Serial.println(F("r ok"));
    } else if (cmd[0] == 'd') {
        debug_countdown = 100;
        Serial.println(F("d on"));
    }
}

void readSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serial_idx > 0) {
                serial_buf[serial_idx] = '\0';
                processCommand(serial_buf);
                serial_idx = 0;
            }
        } else if (serial_idx < SERIAL_BUF_SIZE - 1) {
            serial_buf[serial_idx++] = c;
        }
    }
}

// ========================== PUBLISH ENCODER DATA ==========================

void publishEncoders() {
    long l, r;
    noInterrupts();
    l = left_ticks;
    r = right_ticks;
    interrupts();

    Serial.print(F("e "));
    Serial.print(l);
    Serial.print(' ');
    Serial.println(r);
}

// ========================== SETUP & LOOP ==========================

void setup() {
    Serial.begin(SERIAL_BAUD);

    // On Leonardo, wait for USB CDC serial connection
    // This ensures the first messages aren't lost
    while (!Serial) {
        ; // Wait for serial port to connect (Leonardo-specific)
    }

    // Motor pins
    pinMode(LEFT_ENA,  OUTPUT);
    pinMode(LEFT_IN1,  OUTPUT);
    pinMode(LEFT_IN2,  OUTPUT);
    pinMode(RIGHT_ENB, OUTPUT);
    pinMode(RIGHT_IN3, OUTPUT);
    pinMode(RIGHT_IN4, OUTPUT);

    // Encoder pins
    pinMode(LEFT_ENC_A,  INPUT_PULLUP);
    pinMode(LEFT_ENC_B,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    // Initialize right encoder previous state
    right_enc_a_prev = (PINF & _BV(PF1)) ? 1 : 0;

    // Attach hardware interrupt for left encoder (D1 = INT3 on Leonardo)
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);

    // Setup Timer1 CTC mode for right encoder polling at ~8 kHz
    // ATmega32U4 @ 16MHz: prescaler=8 → 2MHz tick, OCR1A=249 → 8kHz ISR
    noInterrupts();
    TCCR1A = 0;                        // CTC mode (WGM12 in TCCR1B)
    TCCR1B = _BV(WGM12) | _BV(CS11);  // CTC + prescaler /8
    OCR1A  = 249;                      // 16MHz/8/250 = 8kHz
    TIMSK1 = _BV(OCIE1A);             // Enable Compare Match A interrupt
    interrupts();

    stopMotors();

    // Initialize BNO085 IMU over I²C
    Wire.begin();
    initIMU();

    Serial.println(F("Leonardo motor+IMU controller ready (PID + BNO085)"));
}

void loop() {
    // Both encoders are now interrupt-driven:
    //   Left:  hardware INT3 on D1
    //   Right: Timer1 ISR polling A4 at 2kHz
    // No manual polling needed — loop can safely do blocking I2C reads.

    // Process incoming serial commands
    readSerial();

    // Safety: stop motors if no command received recently
    if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
        stopMotors();
    }

    // PID speed control update
    if (millis() - last_pid_time >= PID_INTERVAL_MS) {
        updatePID();  // updatePID() sets last_pid_time internally
    }

    // Publish encoder ticks
    if (millis() - last_publish_time >= PUBLISH_INTERVAL_MS) {
        publishEncoders();
        last_publish_time = millis();
    }

    // Read and publish IMU data — only when it's time to publish.
    if (millis() - last_imu_publish_time >= IMU_PUBLISH_INTERVAL_MS) {
        readIMU();
        publishIMU();
        last_imu_publish_time = millis();
    }
}
