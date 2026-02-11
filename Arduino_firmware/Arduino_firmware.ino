/* =====================================================
   ROS2-Control Ready Arduino Firmware
   L298N + Quadrature Encoders (FIXED PIN CONFLICT)
   ===================================================== */

// -------- ENCODER PINS --------
#define LEFT_ENC_A   2
#define LEFT_ENC_B   4
#define RIGHT_ENC_A  3
#define RIGHT_ENC_B  11

// -------- L298N MOTOR PINS --------
// LEFT MOTOR
#define LEFT_PWM    9
#define LEFT_IN1    8
#define LEFT_IN2    6

// RIGHT MOTOR
#define RIGHT_PWM   10
#define RIGHT_IN1   7
#define RIGHT_IN2   5   // FIXED

volatile long left_ticks  = 0;
volatile long right_ticks = 0;

const float MAX_RAD_PER_SEC = 10.0;
const unsigned long FEEDBACK_PERIOD_MS = 50;

// -------- ENCODER ISR --------
void leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH)
    left_ticks++;
  else
    left_ticks--;
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH)
    right_ticks++;
  else
    right_ticks--;
}

// -------- MOTOR CONTROL --------
void setMotor(float rad_per_sec, int pwm_pin, int in1, int in2) {
  rad_per_sec = constrain(rad_per_sec, -MAX_RAD_PER_SEC, MAX_RAD_PER_SEC);

  int pwm = map(abs(rad_per_sec) * 100,
                0, MAX_RAD_PER_SEC * 100,
                0, 255);

  if (rad_per_sec >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  analogWrite(pwm_pin, pwm);
}

// -------- SERIAL --------
void processSerial() {
  if (!Serial.available()) return;

  char cmd = Serial.read();
  if (cmd == 'V') {
    float v_left  = Serial.parseFloat();
    float v_right = Serial.parseFloat();

    setMotor(v_left,  LEFT_PWM,  LEFT_IN1,  LEFT_IN2);
    setMotor(v_right, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2);
  }
}

// -------- SETUP --------
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  Serial.println("READY");
}

// -------- LOOP --------
void loop() {
  processSerial();

  static unsigned long last_feedback = 0;
  if (millis() - last_feedback >= FEEDBACK_PERIOD_MS) {
    last_feedback = millis();

    Serial.print("E ");
    Serial.print(left_ticks);
    Serial.print(" ");
    Serial.println(right_ticks);
  }
}
