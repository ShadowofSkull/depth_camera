#include <Servo.h>

Servo servo1;
Servo servo2; 
Servo servoH;

// constants won't change
const int SERVO1_PIN = 3; // Arduino pin connected to Servo Motor's pin
const int SERVO2_PIN = 9; // Arduino pin connected to Servo Motor's pin
const int SERVOH_PIN = 12;

// int currentNum = 0;

int state = 0;
String inString = "";

Servo servo; // create servo object to control a servo

void setup() {
  Serial.begin(115200);       // initialize serial port
  while (!Serial) {;} // wait for serial to connect
  Serial.setTimeout(2000); // increase timeout time for reading string
  servo1.attach(SERVO1_PIN);  // attaches the servo on pin 9 to the servo object
  servo2.attach(SERVO2_PIN);
  servoH.attach(SERVOH_PIN);
  servo1.write(70);
  servo2.write(120);
  servoH.write(6);
  pinMode(13, OUTPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
}

void loop() {
  // Check if either pin 6 or pin 7 is LOW
  if (digitalRead(6) == LOW) {
    state = 0;
    digitalWrite(13, HIGH);
    servo1.write(95); // rotate servo motor to 95 degree
    servo2.write(85);
    delay(100);
    // Serial.println("CLOSING");
    servoHcommand(1);
    // Serial.println("Turn Backward");

    // Stay in this loop until the state changes
    while (state == 1) {
      if (Serial.available() > 0) {
        inString = Serial.readStringUntil('\n');

        if (inString == "forward") {
          servo1.write(70); // rotate servo motor to 70 degree
          servo2.write(120);
          // Serial.println("Opening");
          delay(1000);
          servo1.write(95); // rotate servo motor to 95 degree
          servo2.write(85);
          delay(100);
          servoHcommand(0);
          
          // Serial.println("Turn Forward");
          state = 0; // Exit the loop after handling the command
          inString = "";
        }
      }
    }
  } else {
    // Only execute this part if the state is not 1
    servo1.write(70);
    servo2.write(120);
    // Serial.println("OPENING2");
  }
}

void servoHcommand(int con) {
  int newNom = 0;
  int currentNum = 0;

  if (con == 0) {
    currentNum = 135;
    newNom = 6;
  } else if (con == 1) {
    currentNum = 6;
    newNom = 135;
  }

  // if (newNom >= 6 && newNom <= 135) {
  if (currentNum < newNom) {
    for (int i = currentNum; i <= newNom; i++) {
      servoH.write(i);
      Serial.println("Turn 180");
      delay(50);
    }
  } else {
    for (int i = currentNum; i >= newNom; i--) {
      servoH.write(i);
      Serial.println("Turn back");
      delay(50);
    }
  }
  // }
  // currentNum = newNom;
}
sudo apt-get install npm.