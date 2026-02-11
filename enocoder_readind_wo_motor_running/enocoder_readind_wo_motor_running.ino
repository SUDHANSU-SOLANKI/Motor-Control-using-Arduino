volatile long encoderCount = 0;

void encoderISR() {
  encoderCount++;
}

void setup() {
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING);
  Serial.begin(9600);
}

void loop() {
  Serial.println(encoderCount);
  delay(500);
}
