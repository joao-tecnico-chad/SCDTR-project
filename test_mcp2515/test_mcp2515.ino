// Quick ADC pin test
void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  Serial.println("Reading GP26 (pin 31) and GP27 (pin 32)...");
}

void loop() {
  int a0 = analogRead(26);
  int a1 = analogRead(27);
  Serial.print("GP26="); Serial.print(a0);
  Serial.print(" GP27="); Serial.println(a1);
  delay(500);
}
