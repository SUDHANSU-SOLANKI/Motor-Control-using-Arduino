// This is main PID code 
// Change PID parameters once whole real model is created

#include <PID_v1.h>

#include <PID_v1.h>

/* ------------ ENCODER PINS ------------ */
#define LEFT_ENC_A   2      
#define LEFT_ENC_B   4
#define RIGHT_ENC_A  3      
#define RIGHT_ENC_B  11

/* ------------ L298N MOTOR PINS ------------ */
#define LEFT_PWM    9       
#define LEFT_IN1    8       
#define LEFT_IN2    6       
#define RIGHT_PWM   10      
#define RIGHT_IN1   7       
#define RIGHT_IN2   5      

/* ------------ PARAMETERS ------------ */
volatile long left_ticks  = 0;
volatile long right_ticks = 0;
long prev_left_ticks  = 0;
long prev_right_ticks = 0;

const float CPR = 800.0;              
const unsigned long FEEDBACK_PERIOD_MS = 50; 
const float DT = FEEDBACK_PERIOD_MS / 1000.0;

/* ------------ PID VARIABLES ------------ */
// double is required by the PID library
double left_setpoint = 0, left_input = 0, left_output = 0;
double right_setpoint = 0, right_input = 0, right_output = 0;

// Tuning Parameters: Start small!
// Kp: Reactive power, Ki: Steady state error, Kd: Smoothness
double Kp = 2.5, Ki = 10.0, Kd = 0.1; 

PID leftPID(&left_input, &left_output, &left_setpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&right_input, &right_output, &right_setpoint, Kp, Ki, Kd, DIRECT);

/* ------------ ENCODER ISR ------------ */
void leftEncoderISR() {
  (digitalRead(LEFT_ENC_B) == HIGH) ? left_ticks++ : left_ticks--;
}

void rightEncoderISR() {
  (digitalRead(RIGHT_ENC_B) == HIGH) ? right_ticks++ : right_ticks--;
}

/* ------------ MOTOR DRIVE ------------ */
void driveMotor(int pwm, int pwm_pin, int in1, int in2) {
  // PWM comes from PID output (-255 to 255)
  if (pwm >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm_pin, abs(pwm));
}

void setup() {
  Serial.begin(115200);
  
  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // Initialize PID
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(-255, 255);
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(-255, 255);
  
  // High sample rate for smooth control
  leftPID.SetSampleTime(FEEDBACK_PERIOD_MS);
  rightPID.SetSampleTime(FEEDBACK_PERIOD_MS);
}

void loop() {
  // 1. Process Serial Commands from ROS2 Hardware Interface
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'V') {
      left_setpoint = Serial.parseFloat();  // Target rad/s from ROS
      right_setpoint = Serial.parseFloat();
    }
  }

  static unsigned long last_feedback = 0;
  if (millis() - last_feedback >= FEEDBACK_PERIOD_MS) {
    last_feedback = millis();

    // 2. Calculate Actual Velocity (Input for PID)
    noInterrupts();
    long curr_left = left_ticks;
    long curr_right = right_ticks;
    interrupts();

    // Actual Velocity in rad/s: (Delta_Ticks / CPR) * 2PI / DT
    left_input = ((curr_left - prev_left_ticks) / CPR) * 2.0 * PI / DT;
    right_input = ((curr_right - prev_right_ticks) / CPR) * 2.0 * PI / DT;

    prev_left_ticks = curr_left;
    prev_right_ticks = curr_right;

    // 3. Compute PID
    leftPID.Compute();
    rightPID.Compute();

    // 4. Actuate Motors
    driveMotor(left_output, LEFT_PWM, LEFT_IN1, LEFT_IN2);
    driveMotor(right_output, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2);

    // 5. Output for ROS2 Read Loop (Radians)
    float left_pos_rad = (curr_left / CPR) * 2.0 * PI;
    float right_pos_rad = (curr_right / CPR) * 2.0 * PI;
    Serial.print(left_pos_rad, 4);
    Serial.print(",");
    Serial.println(right_pos_rad, 4);
  }
}