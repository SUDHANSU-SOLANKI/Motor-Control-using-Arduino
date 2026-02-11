/* =====================================================
   FINAL Arduino Firmware
   L298N + Quadrature Encoders + RPM Feedback
   ROS2-Control Ready
   ===================================================== */

/* ------------ ENCODER PINS ------------ */
#define LEFT_ENC_A   2      // Interrupt
#define LEFT_ENC_B   4
#define RIGHT_ENC_A  3      // Interrupt
#define RIGHT_ENC_B  11

/* ------------ L298N MOTOR PINS ------------ */
// LEFT MOTOR
#define LEFT_PWM    9       // ENA (remove jumper)
#define LEFT_IN1    8       // IN1
#define LEFT_IN2    6       // IN2

// RIGHT MOTOR
#define RIGHT_PWM   10      // ENB (remove jumper)
#define RIGHT_IN1   7       // IN3
#define RIGHT_IN2   5      // IN4

/* ------------ PARAMETERS ------------ */
volatile long left_ticks  = 0;
volatile long right_ticks = 0;

long prev_left_ticks  = 0;
long prev_right_ticks = 0;

const float CPR = 800.0;              // Encoder counts per wheel revolution
const float MAX_RAD_PER_SEC = 10.0;    // Safety clamp
const unsigned long FEEDBACK_PERIOD_MS = 50; // 20 Hz

/* ------------ ENCODER ISR ------------ */
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

/* ------------ MOTOR CONTROL ------------ */
void setMotor(float rad_per_sec, int pwm_pin, int in1, int in2) {
  rad_per_sec = constrain(rad_per_sec, -MAX_RAD_PER_SEC, MAX_RAD_PER_SEC);

  int pwm = map(abs(rad_per_sec) * 100,
                0, MAX_RAD_PER_SEC * 100,
                0, 255);

  // Direction fixed so +ve command = forward
  if (rad_per_sec >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  analogWrite(pwm_pin, pwm);
}

/* ------------ SERIAL COMMAND ------------ */
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

/* ------------ SETUP ------------ */
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  // Encoder pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Motor pins
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),
                  leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A),
                  rightEncoderISR, RISING);

  Serial.println("READY");
}

/* ------------ LOOP ------------ */
// void loop() {
//   processSerial();

//   static unsigned long last_feedback = 0;
//   if (millis() - last_feedback >= FEEDBACK_PERIOD_MS) {
//     last_feedback = millis();

//     // Copy tick counts safely
//     long curr_left  = left_ticks;
//     long curr_right = right_ticks;

//     // Delta ticks
//     long delta_left  = curr_left  - prev_left_ticks;
//     long delta_right = curr_right - prev_right_ticks;

//     prev_left_ticks  = curr_left;
//     prev_right_ticks = curr_right;

//     float dt = FEEDBACK_PERIOD_MS / 1000.0;

//     // RPM calculation
//     float left_rpm  = (delta_left  / CPR) * (60.0 / dt);
//     float right_rpm = (delta_right / CPR) * (60.0 / dt);

//     // SERIAL OUTPUT
//     // Encoder ticks (for ROS)
//     Serial.print("E ");
//     Serial.print(curr_left);
//     Serial.print(" ");
//     Serial.println(curr_right);

//     // RPM (for debugging)
//     Serial.print("RPM ");
//     Serial.print(left_rpm);
//     Serial.print(" ");
//     Serial.println(right_rpm);
//   }
// }

/* ------------ REFINED LOOP ------------ */
void loop() {
  processSerial();

  static unsigned long last_feedback = 0;
  if (millis() - last_feedback >= FEEDBACK_PERIOD_MS) {
    last_feedback = millis();

    // 1. ATOMIC READ: Disable interrupts for a microsecond to copy values safely
    noInterrupts();
    long curr_left  = left_ticks;
    long curr_right = right_ticks;
    interrupts();

    // 2. CONVERT TO RADIANS (What ROS2 expects)
    float left_pos_rad  = (curr_left  / CPR) * 2.0 * PI;
    float right_pos_rad = (curr_right / CPR) * 2.0 * PI;

    // 3. CLEAN SERIAL OUTPUT (Easy for C++ ReadLine to parse)
    // Format: "left_rad,right_rad"
    Serial.print(left_pos_rad, 4); // 4 decimal places
    Serial.print(",");
    Serial.println(right_pos_rad, 4);
  }
}
