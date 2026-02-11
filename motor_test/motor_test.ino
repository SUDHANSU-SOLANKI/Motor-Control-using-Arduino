int IN1 = 8;
int IN2 = 9;
int PWM = 5;   // MUST be a PWM pin

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
}

void loop() {
 
  for(int i=0;i<=255;i++){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, i);   // speed (0–255)
    delay(1);

  }
  for(int i=255;i>=0;i--){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(PWM, i);   // speed (0–255)
    delay(1);

  }
  analogWrite(PWM, 0);
  delay(3000);
}
