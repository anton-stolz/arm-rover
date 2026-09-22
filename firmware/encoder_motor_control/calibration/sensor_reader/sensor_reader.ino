void setup() {
  Serial.begin(115200);
}

void loop() {
  int val0 = analogRead(A0);
  int val1 = analogRead(A1);
  int val2 = analogRead(A2);
  int val3 = analogRead(A3);
  
  unsigned long t = millis();
  
  Serial.print(t);
  Serial.print(",");
  Serial.print(val0);
  Serial.print(",");
  Serial.print(val1);
  Serial.print(",");
  Serial.print(val2);
  Serial.print(",");
  Serial.println(val3);
  
  delay(5);
}
