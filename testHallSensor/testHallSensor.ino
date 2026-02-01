void setup() {
  Serial.begin(9600);
  Serial.println("setup");

}

void loop() {
  // put your main code here, to run repeatedly:
  float val = analogRead(A3);
  Serial.print(val);
  Serial.println();
  delay(1000);
}
