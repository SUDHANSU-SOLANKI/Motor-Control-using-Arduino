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

const float CPR = 800.0;              // VERIFY THIS VALUE!
const unsigned long FEEDBACK_PERIOD_MS = 50; 
const float DT = FEEDBACK_PERIOD_MS / 1000.0;

/* ------------ PID PARAMETERS ------------ */

// LEFT MOTOR PID
double Kp_left  = 1.2;
double Ki_left  = 2.0;
double Kd_left  = 0.05;

// RIGHT MOTOR PID
double Kp_right = 1.2;
double Ki_right = 2.0;
double Kd_right = 0.05;

/* ------------ PID VARIABLES ------------ */
double left_setpoint = 0, left_input = 0, left_output = 0;
double right_setpoint = 0, right_input = 0, right_output = 0;

PID leftPID(&left_input, &left_output, &left_setpoint,
            Kp_left, Ki_left, Kd_left, DIRECT);

PID rightPID(&right_input, &right_output, &right_setpoint,
             Kp_right, Ki_right, Kd_right, DIRECT);

/* ------------ SAFETY ------------ */
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 500;   // ms

/* ------------ ENCODER ISR ------------ */
void leftEncoderISR() {
  (digitalRead(LEFT_ENC_B) == HIGH) ? left_ticks++ : left_ticks--;
}

void rightEncoderISR() {
  (digitalRead(RIGHT_ENC_B) == HIGH) ? right_ticks++ : right_ticks--;
}

/* ------------ MOTOR DRIVE ------------ */
void driveMotor(double pwm, int pwm_pin, int in1, int in2) {

  int min_pwm = 40;  // Adjust after testing

  int pwm_int = (int)pwm;

  if (abs(pwm_int) > 0 && abs(pwm_int) < min_pwm) {
    pwm_int = (pwm_int > 0) ? min_pwm : -min_pwm;
  }

  if (pwm_int >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  analogWrite(pwm_pin, abs(pwm_int));
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(-255, 255);
  leftPID.SetSampleTime(FEEDBACK_PERIOD_MS);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(-255, 255);
  rightPID.SetSampleTime(FEEDBACK_PERIOD_MS);
}

void loop() {

  /* ------------ RECEIVE ROS COMMAND ------------ */
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'V') {
      left_setpoint = Serial.parseFloat();
      right_setpoint = Serial.parseFloat();
      last_cmd_time = millis();
    }
  }

  /* ------------ SAFETY TIMEOUT ------------ */
  if (millis() - last_cmd_time > CMD_TIMEOUT) {
    left_setpoint = 0;
    right_setpoint = 0;
  }

  /* ------------ FEEDBACK LOOP ------------ */
  static unsigned long last_feedback = 0;

  if (millis() - last_feedback >= FEEDBACK_PERIOD_MS) {

    last_feedback = millis();

    noInterrupts();
    long curr_left = left_ticks;
    long curr_right = right_ticks;
    interrupts();

    // Velocity calculation (rad/s)
    left_input  = ((float)(curr_left - prev_left_ticks) / CPR) * 2.0 * PI / DT;
    right_input = ((float)(curr_right - prev_right_ticks) / CPR) * 2.0 * PI / DT;

    prev_left_ticks = curr_left;
    prev_right_ticks = curr_right;

    // Compute PID
    leftPID.Compute();
    rightPID.Compute();

    // Drive Motors
    driveMotor(left_output, LEFT_PWM, LEFT_IN1, LEFT_IN2);
    driveMotor(right_output, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2);

    // Send position (radians) back to ROS
    float left_pos_rad = ((float)curr_left / CPR) * 2.0 * PI;
    float right_pos_rad = ((float)curr_right / CPR) * 2.0 * PI;

    Serial.print(left_pos_rad, 4);
    Serial.print(",");
    Serial.println(right_pos_rad, 4);
  }
}
