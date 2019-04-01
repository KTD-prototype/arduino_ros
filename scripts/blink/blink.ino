void setup() {
  pinMode(11, OUTPUT);  //A0
}

void loop() {
    digitalWrite(11,HIGH);
    delay(100);
    digitalWrite(11,LOW);
    delay(100);
}
