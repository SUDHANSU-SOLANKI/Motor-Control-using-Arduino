volatile long encoder_count = 0;

const int ENC_A = 2;
const int ENC_B = 3;

const float WHEEL_CPR = 800.0;
unsigned long last_time = 0;
long last_count = 0;

void setup() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  Serial.begin(9600);
  last_time = millis();
}

void loop() {
  unsigned long current_time = millis();

  // run speed calculation every 1 second
  if (current_time - last_time >= 1000) {

    long current_count = encoder_count;
    long delta_count = current_count - last_count;

    float rpm = (delta_count / WHEEL_CPR) * 60.0;

    Serial.print("Counts in 1s: ");
    Serial.print(delta_count);
    Serial.print(" | RPM: ");
    Serial.println(rpm);

    last_count = current_count;
    last_time = current_time;
  }
}

// interrupt routine
void encoderISR() {
  if (digitalRead(ENC_B) == HIGH)
    encoder_count++;
  else
    encoder_count--;
}
