#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include "CytronMotorDriver.h"
#include <math.h>

#include <ros.h>
#include <astra_camera/MotorControl.h>
#include <astra_camera/GripperControl.h>


#define ENCA1 18 // Encoder 1 (LB)
#define ENCB1 17 // Encoder 1
#define ENCA2 2  // Encoder 2 (LF)
#define ENCB2 3  // Encoder 2
#define ENCA3 19 // Encoder 3 (RF)
#define ENCB3 20 // Encoder 3
#define ENCA4 21 // Encoder 4 (RB)
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
int x, z, x_distance, z_distance;
char x_direction, z_direction;

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
// ROS NodeHandle
ros::NodeHandle nh;


// Callback functions for ROS subscribers
void motorControlCallback(const astra_camera::MotorControl &msg) {
    x = msg.x;
    z = msg.z;
    if (x < 0){
      x_direction = "l";
    }
    else{
      x_direction = "r";
    }
    x_distance = x;
    z_distance = z;
}

void gripperControlCallback(const astra_camera::GripperControl &msg) {
  z_direction = msg.flip;
  if (z_direction == "forward"){
    z_direction = 'f';
  }else {
    z_direction = 'b';
  }

}

// ROS Subscribers
ros::Subscriber<astra_camera::MotorControl> motorControlSub("motor_control", motorControlCallback);
ros::Subscriber<astra_camera::GripperControl> gripperControlSub("gripper_control", gripperControlCallback);


void setup() {
  Serial.begin(115200);

  // Set encoder pins as inputs with pull-up resistors
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);
  pinMode(ENCA3, INPUT_PULLUP);
  pinMode(ENCB3, INPUT_PULLUP);
  pinMode(ENCA4, INPUT_PULLUP);
  pinMode(ENCB4, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA3), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA4), readEncoder4, RISING);
  
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);

  // delay(10);
  pos1 = 0;
  pos2 = 0;
  pos3 = 0;
  pos4 = 0;

  // Initialize ROS node handle and subscribers
  nh.initNode();
  nh.subscribe(motorControlSub);
  nh.subscribe(gripperControlSub);
}

void movementControl(char direction, int distance) {
    // Read serial input and update target
  if (Serial.available()) {
    char command = direction;
    // target = command.toFloat() * 6 / PI;  // Update target position
    // target = command.toFloat();  // Update target position
    if (command == 'f') {
      // Move forward
      target = distance; // Extract distance from command
      dir = 'F';
      Serial.println("Front");
    } else if (command == 'b') {
      // Move backward
      target = distance; // Extract distance from command
      dir = 'B';
            Serial.println("Back");
    } else if (command == 'l') {
      // Move left
      target = distance; // Extract distance from command
      dir = 'L';
            Serial.println("Left");
    } else if (command == 'r') {
      // Move right
      target = distance; // Extract distance from command
      dir = 'R';
            Serial.println("Right");
    }
    // Reset motor state to RUNNING when new target is set
    motorState = RUNNING; 
  }

  // PID constants - motor 1 and motor 2 and motor 4
  float kp = 8.0;
  float kd = 1.0;
  float ki = 0.1;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

 ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = posi1;
    pos2 = posi2;
    pos3 = posi3;
    pos4 = posi4;
  }
  
  switch (dir) {
    case 'F':
      target1 = target;
      target2 = target;
      target3 = target;
      target4 = target;
      break;
    case 'B':
      target1 = -target;
      target2 = -target;
      target3 = -target;
      target4 = -target;
      break;
    case 'L':
      target1 = target;
      target2 = -target;
      target3 = target;
      target4 = -target;
      Serial.println("LeftTurn");
      // delay(3000);
      break;
    case 'R':
      target1 = -target;
      target2 = target;
      target3 = -target;
      target4 = target;
      break;
  }
  
  e1 = target1 - pos1;
  e2 = target2 - pos2;
  e3 = target3 - pos3;
  e4 = target4 - pos4;

  // PID calculations - adjust according to your control requirements
  u1 = kp * e1;
  u2 = kp * e2;
  u3 = kp * e3;
  u4 = kp * e4;

  // Limit control signals to prevent excessive motor speed
  if (u1 > 75) u1 = 75;
  else if (u1 < -75) u1 = -75;
  if (u2 > 75) u2 = 75;
  else if (u2 < -75) u2 = -75;
  if (u3 > 75) u3 = 75;
  else if (u3 < -75) u3 = -75;
  if (u4 > 75) u4 = 75;
  else if (u4 < -75) u4 = -75;

  // Set motor speeds based on current state
  if (motorState == RUNNING) {    
    switch (dir){
    case 'F':
    motor1.setSpeed(u1+3); // Motor 1 runs forward
    motor2.setSpeed(u2+3); // Motor 2 and Motor 3 and Motor 4 runs forward
    motor3.setSpeed(u3+1); // Motor 2 and Motor 3 and Motor 4 runs forward
    motor4.setSpeed(u4+8); // Motor 2 and Motor 3 and Motor 4 runs forward
    break;
    case 'B':
    //-------------------------------------------------- need calibrate
    motor1.setSpeed(u1-1); // Motor 1 runs forward
    motor2.setSpeed(u2-5); // Motor 2 and Motor 3 and Motor 4 runs forward
    motor3.setSpeed(u3-5); // Motor 2 and Motor 3 and Motor 4 runs forward
    motor4.setSpeed(u4-3); // Motor 2 and Motor 3 and Motor 4 runs forward
    //--------------------------------------------------------------------
    break;
    case 'L':
    //-------------------------------------------------- need calibrate
    motor1.setSpeed(u1); // Motor 1 runs forward
    motor2.setSpeed(u2-4); // Motor 2 and Motor 3 and Motor 4 runs forward
    motor3.setSpeed(u3+1); // Motor 2 and Motor 3 and Motor 4 runs forward
    motor4.setSpeed(u4-2); // Motor 2 and Motor 3 and Motor 4 runs forward
    //--------------------------------------------------------------------
    break;
    case 'R':
    //-------------------------------------------------- need calibrate
    motor1.setSpeed(u1); // Motor 1 runs forward
    motor2.setSpeed(u2+2); // Motor 2 and Motor 3 and Motor 4 runs forward
    motor3.setSpeed(u3+1); // Motor 2 and Motor 3 and Motor 4 runs forward
    motor4.setSpeed(u4-1); // Motor 2 and Motor 3 and Motor 4 runs forward
    //--------------------------------------------------------------------
    break;
    } 

  }
  else {
    motor1.setSpeed(0); // Stop Motor 1
    motor2.setSpeed(0); // Stop Motor 2
    motor3.setSpeed(0); // Stop Motor 3
    motor4.setSpeed(0); // Stop Motor 4
  }

  // Store previous error
  eprev1 = e1;
  eprev2 = e2;
  eprev3 = e3;
  eprev4 = e4;

if ((abs(e2) <= 2 && abs(e3) <= 2) || (abs(e1) <= 2 && abs(e4) <= 2)) {
    // If all close to target, stop motors
    Serial.println("Stop");
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(0);
    // Reset position values
    pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
    posi1 = 0, posi2 = 0, posi3 = 0, posi4 = 0;
    u1 = 0, u2 = 0, u3 = 0, u4 = 0;
    prevT = 0;
    eprev1 = 0, eprev2 = 0, eprev3 = 0, eprev4 = 0;
    delay(3000);
    // Reset motor state to STOPPED
    motorState = STOPPED;
  }

  // Print debug information
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos1);
  Serial.print(" ");
  Serial.print(pos2);
  Serial.print(" ");  
  Serial.print(pos3);
  Serial.print(" ");
  Serial.print(pos4);
  Serial.print(" ");
  Serial.print(target1);
  Serial.print(" ");
  Serial.print(target2);
  Serial.print(" ");  
  Serial.print(target3);
  Serial.print(" ");
  Serial.print(target4);
  Serial.print(" ");
  Serial.print(e1);
  Serial.print(" ");
  Serial.print(e2);
  Serial.print(" ");  
  Serial.print(e3);
  Serial.print(" ");
  Serial.println(e4);

  // Delay to stabilize the loop
  delay(10);

}

void loop() {  
  movementControl(x_direction, x_distance);
  movementControl(z_direction, z_distance);
  // Spin ROS node
  nh.spinOnce();
}


void readEncoder1() {
  if (digitalRead(ENCB1) == LOW) {
    posi1++;
  } else {
    posi1--;
  }
}

void readEncoder2() {
  if (digitalRead(ENCB2) == HIGH) {
    posi2++;
  } else {
    posi2--;
  }
}

void readEncoder3() {
  if (digitalRead(ENCB3) == HIGH) {
    posi3--;
  } else {
    posi3++;
  }
}

void readEncoder4() {
  if (digitalRead(ENCB4) == HIGH) {
    posi4--;
  } else {
    posi4++;
  }
}
