#include <PID_v1.h>

/* ------------ PINS ------------ */
#define LEFT_ENC_A 2
#define LEFT_ENC_B 4
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 11

#define LEFT_PWM 9
#define LEFT_IN1 8
#define LEFT_IN2 6
#define RIGHT_PWM 10
#define RIGHT_IN1 7
#define RIGHT_IN2 5

/* ------------ PARAMETERS ------------ */
volatile long left_ticks = 0;
volatile long right_ticks = 0;
// long prev_left_ticks = 0;
// long prev_right_ticks = 0;
double left_vel_filt = 0, right_vel_filt = 0;

const float CPR = 800.0;
const unsigned long FEEDBACK_PERIOD_MS = 50;  // Control loops at 20 Hz (T = 1/f)
const float DT = FEEDBACK_PERIOD_MS / 1000.0;  //dt

// Safety Watchdog (10 seconds for manual testing)
// Some problems with ROS2 ==> Automatic robot stops
unsigned long last_cmd_time = 0;
const unsigned long TIMEOUT_MS = 10000; 

/* ------------ PID CONFIG ------------ */
double left_setpoint = 0, left_input = 0, left_output = 0;
//     Desired            Actual          Output(PWM)
double right_setpoint = 0, right_input = 0, right_output = 0;

// Increased Kp for bench testing. Added small Ki to help reach speed.
double Kp = 8.0;
double Ki = 0.5; // For wheel velocity ==> 0.1 to 0.8
double Kd = 0.2; // larger ki ==> Unstable robot

PID leftPID(&left_input, &left_output, &left_setpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&right_input, &right_output, &right_setpoint, Kp, Ki, Kd, DIRECT);

/* ------------ INTERRUPTS ------------ */
// Interrupts Function Declaration
void leftEncoderISR() { (digitalRead(LEFT_ENC_B) == HIGH) ? left_ticks++ : left_ticks--; }
void rightEncoderISR() { (digitalRead(RIGHT_ENC_B) == HIGH) ? right_ticks++ : right_ticks--; }

/* ------------ MOTOR HELPER ------------ */
void driveMotor(double pid_output, int pwm_pin, int in1, int in2) {
  // If setpoint(Desired) is 0, stop completely
  // When let go of key ==> clear PID memory
  if (abs(left_setpoint) < 0.01 && abs(right_setpoint) < 0.01) {
    leftPID.SetMode(MANUAL);
    rightPID.SetMode(MANUAL);

    left_output = 0;
    right_output = 0;

    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);
  }


  // Add a minimum PWM offset (Deadzone compensation)
  // If PID output is 10, and MIN_PWM is 40, actual PWM becomes 50.
  int MIN_PWM = 20; // Thresold ==> Below which motors will not run
  int final_pwm = 0;

  if (pid_output > 0) final_pwm = MIN_PWM + (int)pid_output;    
  else if (pid_output < 0) final_pwm = -MIN_PWM + (int)pid_output;

  final_pwm = constrain(final_pwm, -255, 255);  //limitimg values

  if (final_pwm >= 0) {
    // Forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Reverse
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm_pin, abs(final_pwm));
}

void setup() {
  Serial.begin(115200); // High BR ==> low latency ROS2 communication
  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);

  // Accurate encoder counting even at high speed
  // Call Interrups at RISING Edge (LOW --> HIGH)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  leftPID.SetMode(AUTOMATIC);
  // Prevents saturation at ±255
  leftPID.SetOutputLimits(-200, 200); // Leave room for MIN_PWM offset
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(-200, 200);
  // 50 ms (at 20 Hz)
  leftPID.SetSampleTime(FEEDBACK_PERIOD_MS);
  rightPID.SetSampleTime(FEEDBACK_PERIOD_MS);
}

/* ------------ CLEAN ROS2 LOOP ------------ */
void loop() {
  // 1. READ COMMAND FROM ROS2 (Format: V left_rad right_rad)
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'V') {
      left_setpoint = Serial.parseFloat();
      right_setpoint = Serial.parseFloat();
      // Number of milliseconds since Arduino powered ON
      last_cmd_time = millis();
    }
  }

  // 2. SAFETY WATCHDOG
  // If no ROS cmds ==> stop robot
  // Check every 10 seconds
  // Current - Previous
  if (millis() - last_cmd_time > TIMEOUT_MS) {
    left_setpoint = 0;
    right_setpoint = 0;
  }

  static unsigned long last_feedback = 0;
  if (millis() - last_feedback >= FEEDBACK_PERIOD_MS) {
    last_feedback = millis();

    // 3. GET ACTUAL VELOCITY FOR PID
    noInterrupts();
    long curr_left = left_ticks;
    long curr_right = right_ticks;
    interrupts();

    // Delts ==> revolutions ==> rad ==> rad/sec
    // ROS compatible Units
    // left_input = ((curr_left - prev_left_ticks) / CPR) * 2.0 * PI / DT;
    // right_input = ((curr_right - prev_right_ticks) / CPR) * 2.0 * PI / DT;

    // prev_left_ticks = curr_left;
    // prev_right_ticks = curr_right;
    double alpha = 0.7; // 0.6–0.8 works well

    left_input = alpha * left_vel_filt +
                (1 - alpha) * left_input;

    right_input = alpha * right_vel_filt +
                  (1 - alpha) * right_input;

    left_vel_filt = left_input;
    right_vel_filt = right_input;


    // 4. COMPUTE PID
    // Calculates required PWM correction
    leftPID.Compute();
    rightPID.Compute();

    // 5. ACTUATE MOTORS
    driveMotor(left_output, LEFT_PWM, LEFT_IN1, LEFT_IN2);
    driveMotor(right_output, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2);

    // 6. RAW ROS2 FEEDBACK (Format: left_rad,right_rad)
    // No extra text, just the numbers and a comma.
    // Wheel positions in rad
    Serial.print((curr_left / CPR) * 2.0 * PI, 4);
    Serial.print(",");
    Serial.println((curr_right / CPR) * 2.0 * PI, 4);
  }
}