#include <Servo.h>

Servo servo1;
Servo servo2; 
Servo servoH;

// constants won't change
const int SERVO1_PIN = 3; // Arduino pin connected to Servo Motor's pin
const int SERVO2_PIN = 9; // Arduino pin connected to Servo Motor's pin
const int SERVOH_PIN = 12;

// int currentNum = 0;

bool isFront = true;
String inString = "";

Servo servo; // create servo object to control a servo

void setup() {
  Serial.begin(115200);       // initialize serial port
  while (!Serial) {;} // wait for serial to connect
 
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
//close grip
//servo1.write(95); 
//servo2.write(85);
// open grip
//servo1.write(70); 
//servo2.write(120);
// flip forward
//servoHcommand(0);
// flip backward
//servoHcommand(1);

void loop() {
  // Check if either pin 6 or pin 7 is LOW
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    switch (command.charAt(0)){
        case 'j':
        servo1.write(95); 
        servo2.write(85);
        break;
        case 'k':
        servo1.write(70); 
        servo2.write(120);
        break;
        case 'l':
        servoHcommand(0);
        break;
        case ';':
        servoHcommand(1);
        break;
    } 
    delay(500);

  }
}

void servoHcommand(int con) {
  int newNom;
  int currentNum;
  int high = servoH.read();
  Serial.println(high);
  if (con == 0 && high == 135) {
    currentNum = 135;
    newNom = 6;
    
  } else if (con == 1 && high == 6) {
    currentNum = 6;
    newNom = 135;
    
  } else if (con == 0 && high == 6){
    currentNum = 6;
    newNom = 6;
  } else if (con == 1 && high == 135){
    currentNum = 135;
    newNom = 135;
  }
  
  if (currentNum < newNom) {
    for (int i = currentNum; i <= newNom; i++) {
      servoH.write(i);
      // Serial.println("Turn 180");
      delay(50);
    }
  } else {
    for (int i = currentNum; i >= newNom; i--) {
      servoH.write(i);
      // Serial.println("Turn back");
      delay(50);
    }
  }

}
