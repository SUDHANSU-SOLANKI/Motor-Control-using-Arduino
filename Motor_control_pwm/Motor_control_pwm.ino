/* =========================================================
   Encoder-based Motor Control (Arduino)
   Motor: DC Encoder Motor (Robokits)
   Measured Wheel_CPR = 800 counts per output shaft revolution
   ========================================================= */

// ---------- ENCODER PINS ----------
#define ENC_A 2        // Interrupt pin
#define ENC_B 3

// ---------- MOTOR DRIVER PINS ----------
#define IN1 8
#define IN2 9
#define PWM 5

// ---------- CONSTANTS ----------
const float WHEEL_CPR = 800.0;   // Measured by hand rotation
const unsigned long SAMPLE_TIME = 1000; // ms (1 second)

// ---------- VARIABLES ----------
volatile long encoder_count = 0;

long last_count = 0;
unsigned long last_time = 0;

float target_rpm = 150.0;   // Desired speed
int pwm_value = 100;        // Initial PWM (0–255)

// =========================================================
// ENCODER INTERRUPT SERVICE ROUTINE
// =========================================================
void encoderISR() {
  if (digitalRead(ENC_B) == HIGH)
    encoder_count--;
  else
    encoder_count++;
}

// =========================================================
// SETUP
// =========================================================
void setup() {

  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Motor driver pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  // Motor direction (FORWARD)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  Serial.begin(9600);
  Serial.println("Encoder Motor Control Started");

  last_time = millis();
}

// =========================================================
// LOOP
// =========================================================
void loop() {

  unsigned long now = millis();

  if (now - last_time >= SAMPLE_TIME) {

    // Read encoder count
    long current_count = encoder_count;
    long delta_count = current_count - last_count;

    // Calculate RPM
    float rpm = (delta_count / WHEEL_CPR) * 60.0;

    // Simple speed control (NO PID)
    if (rpm < target_rpm)
      pwm_value += 5;
    else if (rpm > target_rpm)
      pwm_value -= 5;

    pwm_value = constrain(pwm_value, 0, 255);

    // Apply PWM
    analogWrite(PWM, pwm_value);

    // Print data
    Serial.print("Counts/sec: ");
    Serial.print(delta_count);
    Serial.print(" | RPM: ");
    Serial.print(rpm);
    Serial.print(" | PWM: ");
    Serial.println(pwm_value);

    // Update values
    last_count = current_count;
    last_time = now;
  }
}
