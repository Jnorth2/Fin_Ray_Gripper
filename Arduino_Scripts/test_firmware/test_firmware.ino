void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial) {
    delay(10);
  }
  Serial.println("here");
  Serial1.begin(115200);
  while(!Serial1){
    Serial.println("no uart");
  }
  Serial.println("at 2");
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
    Serial.println(c);
  }
  delay(5);
}
