#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <util/atomic.h>
#include "CytronMotorDriver.h"
#include <math.h>

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

// ROS node handle
ros::NodeHandle nh;

// ROS topic to receive Twist commands (linear and angular velocities)
void twistCallback(const geometry_msgs::Twist& twist_msg) {
  float linearX = twist_msg.linear.x;
  float angularZ = twist_msg.angular.z;

  // Convert Twist commands to motor commands
  if (linearX > 0) {
    // Move forward
    target = linearX * 0.367; // Adjust as needed for your robot's movement characteristics
    dir = 'F';
  } else if (linearX < 0) {
    // Move backward
    target = -linearX * 0.367; // Adjust as needed
    dir = 'B';
  } else if (angularZ > 0) {
    // Rotate left
    target = angularZ * 0.367; // Adjust as needed
    dir = 'L';
  } else if (angularZ < 0) {
    // Rotate right
    target = -angularZ * 0.367; // Adjust as needed
    dir = 'R';
  }

  // Reset motor state to RUNNING when new target is set
  motorState = RUNNING;
}

// ROS subscriber for Twist commands
ros::Subscriber<geometry_msgs::Twist> twistSub("/cmd_vel", &twistCallback);

void setup() {
  Serial.begin(115200);
  while (!Serial) {;} // Wait for serial to connect

  nh.getHardware()->setBaud(115200);
  nh.initNode();

  // Set up encoders
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

  pos1 = 0;
  pos2 = 0;
  pos3 = 0;
  pos4 = 0;

  nh.subscribe(twistSub);
}

void loop() {
  nh.spinOnce();

  // PID constants - adjust according to your needs
  float kp = 10.0;
  float kd = 1.0;
  float ki = 0.1;

  // Time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
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
    motor1.setSpeed(u1);
    motor2.setSpeed(u2);
    motor3.setSpeed(u3);
    motor4.setSpeed(u4);
  } else {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(0);
  }

  // Store previous error
  eprev1 = e1;
  eprev2 = e2;
  eprev3 = e3;
  eprev4 = e4;

  // If all close to target, stop motors
  if ((abs(e2) <= 2 && abs(e3) <= 2) || (abs(e1) <= 2 && abs(e4) <= 2)) {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(0);
    pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
    posi1 = 0, posi2 = 0, posi3 = 0, posi4 = 0;
    u1 = 0, u2 = 0, u3 = 0, u4 = 0;
    prevT = 0;
    eprev1 = 0, eprev2 = 0, eprev3 = 0, eprev4 = 0;
    motorState = STOPPED;
  }

  delay(10);
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
