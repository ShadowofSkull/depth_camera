#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
float target;
char dir;

void setup() {
  Serial.begin(115200); // Set baud rate to 9600
  
  // Serial.setTimeout(10000);
  // Serial.println("Arduino ready.");
  myservo.attach(9); 
  myservo.write(0);
  delay(1000);
}

void loop() {
  if (Serial.available() > 0) {
    Serial.print(Serial.available());
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("F")) {
      // Move forward
      target = command.substring(1).toFloat(); // Extract distance from command
      dir = 'F';
      // Serial.println("Front");
    } else if (command.startsWith("B")) {
      // Move backward
      target = command.substring(1).toFloat(); // Extract distance from command
      dir = 'B';
            // Serial.println("Back");
    } else if (command.startsWith("L")) {
      // Move left
      target = command.substring(1).toFloat(); // Extract distance from command
      dir = 'L';
            // Serial.println("Left");
    } else if (command.startsWith("R")) {
      // Move right
      target = command.substring(1).toFloat(); // Extract distance from command
      dir = 'R';
            // Serial.println("Right");
    }
   
    Serial.print("Received: ");
    Serial.print(dir); // Echo received data back
    Serial.println(target);
    if (dir == 'B' && target == 200) {
      for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
      for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
      Serial.println(1);
    }
    Serial.read(); // To clear the serial buffer

  }
  delay(10);
}
