#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include "CytronMotorDriver.h"
#include <math.h>

#define ENCA1 18 // Encoder 1 (LB)
#define ENCB1 14 // Encoder 1
#define ENCA2 19  // Encoder 2 (LF)
#define ENCB2 15  // Encoder 2
#define ENCA3 3 // Encoder 3 (RF)
#define ENCB3 17 // Encoder 3
#define ENCA4 2 // Encoder 4 (RB)
#define ENCB4 16 // Encoder 4

volatile int posi1 = 0, posi2 = 0, posi3 = 0, posi4 = 0; // Positions
int pos1, pos2, pos3, pos4;
long prevT = 0;
float eprev1 = 0, eprev2 = 0, eprev3 = 0, eprev4 = 0;
// float eintegral1 = 0, eintegral2 = 0, eintegral3 = 0, eintegral4 = 0;
float u1 = 0, u2 = 0, u3 = 0, u4 = 0;
float target;
float target1 = 0, target2 = 0, target3 = 0, target4 = 0;
char dir;
int count = 0;
int e1, e2, e3, e4;

CytronMD motor1(PWM_DIR, 4, 5); // Motor 1 (LB)
CytronMD motor2(PWM_DIR, 6, 7); // Motor 2 (LF)
CytronMD motor3(PWM_DIR, 8, 9); // Motor 3 (RF)
CytronMD motor4(PWM_DIR, 10, 11); // Motor 4 (RB)

// Define states
enum MotorState {
  RUNNING,
  STOPPED
};

MotorState motorState = STOPPED;

void setup() {
  Serial.begin(115200);
  while (!Serial) {;} // wait for serial to connect
  // Serial.setTimeout(2000); // increase timeout time for reading string
  // Set encoder pins as inputs with pull-up resistors
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);
  pinMode(ENCA3, INPUT_PULLUP);
  pinMode(ENCB3, INPUT_PULLUP);
  pinMode(ENCA4, INPUT_PULLUP);
  pinMode(ENCB4, INPUT_PULLUP);
  
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);

}

void loop() {
  // Read serial input and update target if motorState is stopped to avoid interrupting unfinish movement
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    Serial.println(command);
    
    int speed = 100;

    switch (command.charAt(0)){
        case 'w':
        motor1.setSpeed(speed); // Motor 1 runs forward
        motor2.setSpeed(speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        motor3.setSpeed(speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        motor4.setSpeed(speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        
        break;
        case 's':
        //-------------------------------------------------- need calibrate
        motor1.setSpeed(-speed); // Motor 1 runs forward
        motor2.setSpeed(-speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        motor3.setSpeed(-speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        motor4.setSpeed(-speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        //--------------------------------------------------------------------
        break;
        case 'a':
        //-------------------------------------------------- need calibrate
        motor1.setSpeed(speed); // Motor 1 runs forward
        motor2.setSpeed(-speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        motor3.setSpeed(speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        motor4.setSpeed(-speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        //--------------------------------------------------------------------
        break;
        case 'd':
        //-------------------------------------------------- need calibrate
        motor1.setSpeed(-speed); // Motor 1 runs forward
        motor2.setSpeed(speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        motor3.setSpeed(-speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        motor4.setSpeed(speed); // Motor 2 and Motor 3 and Motor 4 runs forward
        //--------------------------------------------------------------------
        break;
    } 
    delay(500);
  }
  else {
    motor1.setSpeed(0); // Motor 1 runs forward
    motor2.setSpeed(0); // Motor 2 and Motor 3 and Motor 4 runs forward
    motor3.setSpeed(0); // Motor 2 and Motor 3 and Motor 4 runs forward
    motor4.setSpeed(0); // Motor 2 and Motor 3 and Motor 4 runs forward
  }

  // Delay to stabilize the loop
//  delay(500);

}
