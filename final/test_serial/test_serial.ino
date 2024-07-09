#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600); // Set baud rate to 9600
  
  // Serial.setTimeout(10000);
  // Serial.println("Arduino ready.");
  myservo.attach(9); 
  myservo.write(0);
  delay(1000);
}

void loop() {
  if (Serial.available() > 0) {
    Serial.print(Serial.available());
    String data = Serial.readStringUntil('\n'); // Read data until newline
    Serial.print("Received: ");
    Serial.println(data); // Echo received data back
    if (data == "forward") {
      for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
      for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
    }
    Serial.read(); // To clear the serial buffer

  }
  delay(10);
}
