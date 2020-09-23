# 과제제출
#define PIN_LED 7

void setup() {
  pinMode(7, OUTPUT);
}

void loop() {
    digitalWrite(7, HIGH);
    delay(200);
    digitalWrite(7, LOW);
    delay(200);    
    digitalWrite(7, HIGH);
    delay(200);
}
