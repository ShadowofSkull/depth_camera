#include <ros.h>
#include <std_msgs/Float32.h>
#include <astra_camera/MotorControl.h>  // Replace 'astra_camera' with your actual ROS package name

#include <util/atomic.h> // For the ATOMIC_BLOCK macro
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
float eintegral1 = 0, eintegral2 = 0, eintegral3 = 0, eintegral4 = 0;
float u1 = 0, u2 = 0, u3 = 0, u4 = 0;
float target1, target2, target3, target4;

CytronMD motor1(PWM_DIR, 4, 5); // Motor 1 (Left Back)
CytronMD motor2(PWM_DIR, 6, 7); // Motor 2 (Left Front)
CytronMD motor3(PWM_DIR, 8, 9); // Motor 3 (Right Front)
CytronMD motor4(PWM_DIR, 10, 11); // Motor 4 (Right Back)

ros::NodeHandle nh;

// Define callback function to receive motor control messages
void motorControlCallback(const astra_camera::MotorControl& msg) {
  int16_t x = msg.x;  // Horizontal values
  int16_t z = msg.z;  // Depth values

  float speed = 255.0; // Assuming max speed of 255
  float timeToRun = abs(z) / speed; // Time to run the motor based on distance and speed

  if (z > 0) {
    // Move forward
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor3.setSpeed(speed);
    motor4.setSpeed(speed);
  } else if (z < 0) {
    // Move backward
    motor1.setSpeed(-speed);
    motor2.setSpeed(-speed);
    motor3.setSpeed(-speed);
    motor4.setSpeed(-speed);
  } else if (x > 0) {
    // Move right
    motor1.setSpeed(-speed);
    motor2.setSpeed(speed);
    motor3.setSpeed(speed);
    motor4.setSpeed(-speed);
  } else if (x < 0) {
    // Move left
    motor1.setSpeed(speed);
    motor2.setSpeed(-speed);
    motor3.setSpeed(-speed);
    motor4.setSpeed(speed);
  } else if (x > 0 && z > 0) {
    // Move diagonally top-right
    motor1.setSpeed(0);
    motor2.setSpeed(speed);
    motor3.setSpeed(speed);
    motor4.setSpeed(0);
  } else if (x < 0 && z > 0) {
    // Move diagonally top-left
    motor1.setSpeed(speed);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(speed);
  } else if (x > 0 && z < 0) {
    // Move diagonally bottom-right
    motor1.setSpeed(-speed);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(-speed);
  } else if (x < 0 && z < 0) {
    // Move diagonally bottom-left
    motor1.setSpeed(0);
    motor2.setSpeed(-speed);
    motor3.setSpeed(-speed);
    motor4.setSpeed(0);
  }

  delay(timeToRun * 1000); // Convert to milliseconds

  // Stop the motors after moving
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}

ros::Subscriber<astra_camera::MotorControl> sub("MotorControl", motorControlCallback);

void setup() {
  Serial.begin(115200);
  nh.initNode();
  nh.subscribe(sub);

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
  delay(10);
  pos1 = 0;
  pos2 = 0;
  pos3 = 0;
  pos4 = 0;
}

void loop() {
  nh.spinOnce();
  delay(10); // Small delay to prevent overwhelming the CPU
}

void readEncoder1() {
  if (digitalRead(ENCB1) == HIGH) {
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
    posi3++;
  } else {
    posi3--;
  }
}

void readEncoder4() {
  if (digitalRead(ENCB4) == HIGH) {
    posi4++;
  } else {
    posi4--;
  }
}
