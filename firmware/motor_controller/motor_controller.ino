/*
 * ============================================================================
 * Arduino Leonardo — Dual Motor + Encoder Controller for ROS2 Differential Drive
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
 * Pin Assignments:
 *   L298N Motor Driver:
 *     ENA → D5  (PWM, left motor speed)
 *     IN1 → D7  (left motor direction)
 *     IN2 → D6  (left motor direction)
 *     IN3 → D10 (right motor direction)
 *     IN4 → D9  (right motor direction)
 *     ENB → D11 (PWM, right motor speed)
 *
 *   Left Motor Encoder:
 *     Green  → D3  (Channel A — hardware interrupt INT0)
 *     Yellow → D2  (Channel B — direction sensing)
 *
 *   Right Motor Encoder:
 *     Yellow → A4  (Channel A — polled, no hardware interrupt)
 *     Green  → A5  (Channel B — direction sensing)
 *
 * Serial Protocol (115200 baud, USB CDC):
 *   Commands FROM ROS2 (LattePanda → Leonardo):
 *     m <left_pwm> <right_pwm>\n
 *       Values: -255 to 255 (positive = forward)
 *
 *   Feedback TO ROS2 (Leonardo → LattePanda):
 *     e <left_ticks> <right_ticks>\n
 *       Cumulative signed encoder ticks, sent every 50ms (20 Hz)
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

// ========================== PIN DEFINITIONS ==========================

// Left Motor (L298N)
#define LEFT_ENA   5    // PWM speed control
#define LEFT_IN1   7    // Direction
#define LEFT_IN2   6    // Direction

// Right Motor (L298N)
#define RIGHT_IN3  10   // Direction
#define RIGHT_IN4  9    // Direction
#define RIGHT_ENB  11   // PWM speed control

// Left Encoder (hardware interrupt capable on Leonardo)
#define LEFT_ENC_A   3  // Green wire → INT0 on Leonardo
#define LEFT_ENC_B   2  // Yellow wire → direction sensing

// Right Encoder (polled — A4/A5 have no hardware interrupt on ATmega32U4)
#define RIGHT_ENC_A  A4  // Yellow wire → polled channel A
#define RIGHT_ENC_B  A5  // Green wire  → direction sensing

// ========================== CONFIGURATION ==========================

#define SERIAL_BAUD         115200
#define PUBLISH_INTERVAL_MS    50   // Encoder data publish rate (20 Hz)
#define PID_INTERVAL_MS        50   // PID update rate
#define CMD_TIMEOUT_MS        500   // Stop motors if no command for 500ms
#define SERIAL_BUF_SIZE        64

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
#define MAX_TICKS_PER_INTERVAL  40    // Target scaling (use slower motor's value)
#define LEFT_MAX_TICKS          40    // Measured: left motor ticks at PWM 255 per interval
#define RIGHT_MAX_TICKS         40    // Measured: right motor ticks at PWM 255 per interval

// Minimum PWM to overcome motor stiction and L298N voltage drop
#define MIN_PWM  35

// PID gains — start conservative, tune on real hardware
#define KP   0.8
#define KI   1.0
#define KD   0.05
#define INTEGRAL_LIMIT  150.0

// Target ramping for smooth acceleration
#define RAMP_RATE  5.0f   // ticks/interval per PID cycle

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

// ========================== ENCODER STATE ==========================

// Left encoder — updated by hardware interrupt
volatile long left_ticks  = 0;

// Right encoder — updated by polling in loop()
volatile long right_ticks = 0;
byte right_enc_a_last = LOW;  // Previous state of right encoder channel A

// ========================== TIMING ==========================

unsigned long last_publish_time = 0;
unsigned long last_pid_time     = 0;
unsigned long last_cmd_time     = 0;

// ========================== SERIAL BUFFER ==========================

char serial_buf[SERIAL_BUF_SIZE];
int  serial_idx = 0;

// ========================== LEFT ENCODER ISR ==========================
// Hardware interrupt on D3 (INT0 on Leonardo), RISING edge

void leftEncoderISR() {
    if (digitalRead(LEFT_ENC_B) == HIGH) {
        left_ticks++;
    } else {
        left_ticks--;
    }
}

// ========================== RIGHT ENCODER POLLING ==========================
// Called every loop() iteration — checks for RISING edge on A4

void pollRightEncoder() {
    byte current = digitalRead(RIGHT_ENC_A);
    if (current == HIGH && right_enc_a_last == LOW) {
        // Rising edge detected
        if (digitalRead(RIGHT_ENC_B) == HIGH) {
            right_ticks++;
        } else {
            right_ticks--;
        }
    }
    right_enc_a_last = current;
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
                    int ena_pin, int in1_pin, int in2_pin) {
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

    // Compute error
    float actual = (float)delta_ticks;
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

    // Dead-zone compensation
    if (pid.output_pwm > 0 && pid.output_pwm < MIN_PWM) pid.output_pwm = MIN_PWM;
    if (pid.output_pwm < 0 && pid.output_pwm > -MIN_PWM) pid.output_pwm = -MIN_PWM;

    setMotor(pid.output_pwm, ena_pin, in1_pin, in2_pin);
}

void updatePID() {
    noInterrupts();
    long l = left_ticks;
    long r = right_ticks;
    interrupts();

    long delta_left  = l - prev_left_ticks;
    long delta_right = r - prev_right_ticks;
    prev_left_ticks  = l;
    prev_right_ticks = r;

    updateMotorPID(left_pid,  delta_left,  LEFT_MAX_TICKS,
                   LEFT_ENA,  LEFT_IN1,  LEFT_IN2);
    updateMotorPID(right_pid, delta_right, RIGHT_MAX_TICKS,
                   RIGHT_ENB, RIGHT_IN3, RIGHT_IN4);

    // Debug output
    if (debug_countdown > 0) {
        debug_countdown--;
        Serial.print("d L:");
        Serial.print(left_pid.target, 1);
        Serial.print(",");
        Serial.print(delta_left);
        Serial.print(",");
        Serial.print(left_pid.integral, 1);
        Serial.print(",");
        Serial.print(left_pid.output_pwm);
        Serial.print(" R:");
        Serial.print(right_pid.target, 1);
        Serial.print(",");
        Serial.print(delta_right);
        Serial.print(",");
        Serial.print(right_pid.integral, 1);
        Serial.print(",");
        Serial.println(right_pid.output_pwm);
    }
}

// ========================== SERIAL COMMAND PARSING ==========================

void processCommand(const char* cmd) {
    if (cmd[0] == 'm') {
        int left_pwm = 0, right_pwm = 0;
        if (sscanf(cmd + 1, "%d %d", &left_pwm, &right_pwm) == 2) {
            left_pwm  = constrain(left_pwm,  -255, 255);
            right_pwm = constrain(right_pwm, -255, 255);
            left_pid.cmd_target  = (left_pwm  / 255.0f) * MAX_TICKS_PER_INTERVAL;
            right_pid.cmd_target = (right_pwm / 255.0f) * MAX_TICKS_PER_INTERVAL;
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
        Serial.println("r ok");
    } else if (cmd[0] == 'd') {
        debug_countdown = 100;
        Serial.println("d on");
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

    Serial.print("e ");
    Serial.print(l);
    Serial.print(" ");
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

    // Initialize right encoder last state for polling
    right_enc_a_last = digitalRead(RIGHT_ENC_A);

    // Attach hardware interrupt for left encoder (D3 = INT0 on Leonardo)
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);

    // NOTE: Right encoder on A4/A5 does not support hardware interrupts
    // on ATmega32U4 (Leonardo). It is polled in loop() instead.

    stopMotors();

    Serial.println("Leonardo motor controller ready (PID + polling)");
}

void loop() {
    // Poll right encoder every loop iteration for reliable detection
    pollRightEncoder();

    // Process incoming serial commands
    readSerial();

    // Safety: stop motors if no command received recently
    if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
        stopMotors();
    }

    // PID speed control update
    if (millis() - last_pid_time >= PID_INTERVAL_MS) {
        updatePID();
        last_pid_time = millis();
    }

    // Publish encoder ticks
    if (millis() - last_publish_time >= PUBLISH_INTERVAL_MS) {
        publishEncoders();
        last_publish_time = millis();
    }
}
