void setup() {
  Serial.begin(9600); // Set baud rate to 9600
  Serial.println("Arduino ready.");
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Read data until newline
    Serial.print("Received: ");
    Serial.println(data); // Echo received data back
  }
}
