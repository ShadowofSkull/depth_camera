#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include "CytronMotorDriver.h"

// Define pins for Motor 1
#define ENCA_M1 2
#define ENCB_M1 3
#define PWM_M1 4
#define DIR_M1 5

// Define pins for Motor 2
#define ENCA_M2 8
#define ENCB_M2 9
#define PWM_M2 6
#define DIR_M2 7

// Define pins for Motor 3
#define ENCA_M3 12
#define ENCB_M3 13
#define PWM_M3 10
#define DIR_M3 11

// Define pins for Motor 4
#define ENCA_M4 16
#define ENCB_M4 17
#define PWM_M4 14
#define DIR_M4 15

// Motor instances
CytronMD motor1(PWM_M1, DIR_M1);
CytronMD motor2(PWM_M2, DIR_M2);
CytronMD motor3(PWM_M3, DIR_M3);
CytronMD motor4(PWM_M4, DIR_M4);

volatile int posi1 = 0; // Position for Motor 1
volatile int posi2 = 0; // Position for Motor 2
volatile int posi3 = 0; // Position for Motor 3
volatile int posi4 = 0; // Position for Motor 4

int pos1 = 0; // Actual position for Motor 1
int pos2 = 0; // Actual position for Motor 2
int pos3 = 0; // Actual position for Motor 3
int pos4 = 0; // Actual position for Motor 4

long prevT1 = 0; // Previous time for Motor 1
long prevT2 = 0; // Previous time for Motor 2
long prevT3 = 0; // Previous time for Motor 3
long prevT4 = 0; // Previous time for Motor 4

float eprev1 = 0; // Previous error for Motor 1
float eprev2 = 0; // Previous error for Motor 2
float eprev3 = 0; // Previous error for Motor 3
float eprev4 = 0; // Previous error for Motor 4

float eintegral1 = 0; // Integral error for Motor 1
float eintegral2 = 0; // Integral error for Motor 2
float eintegral3 = 0; // Integral error for Motor 3
float eintegral4 = 0; // Integral error for Motor 4

ros::NodeHandle nh;

int targets[4] = {0, 0, 0, 0}; // Target positions for the motors

void targetCallback(const std_msgs::Int32MultiArray& msg) {
  if (msg.data_length == 4) {
    targets[0] = msg.data[0];
    targets[1] = msg.data[1];
    targets[2] = msg.data[2];
    targets[3] = msg.data[3];
  }
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("silo_path", &targetCallback);

void setup() {
  Serial.begin(115200);

  // ROS setup
  nh.initNode();
  nh.subscribe(sub);

  // Encoder pins setup for Motor 1
  pinMode(ENCA_M1, INPUT_PULLUP);
  pinMode(ENCB_M1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_M1), readEncoder1, RISING);

  // Encoder pins setup for Motor 2
  pinMode(ENCA_M2, INPUT_PULLUP);
  pinMode(ENCB_M2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2), readEncoder2, RISING);

  // Encoder pins setup for Motor 3
  pinMode(ENCA_M3, INPUT_PULLUP);
  pinMode(ENCB_M3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_M3), readEncoder3, RISING);

  // Encoder pins setup for Motor 4
  pinMode(ENCA_M4, INPUT_PULLUP);
  pinMode(ENCB_M4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_M4), readEncoder4, RISING);

  Serial.println("Target positions");
}

void loop() {
  nh.spinOnce();

  // Motor 1 control
  float kp1 = 10.0; // PID constants for Motor 1
  float kd1 = 1.0;
  float ki1 = 0.1;
  controlMotor(motor1, PWM_M1, DIR_M1, ENCA_M1, posi1, pos1, prevT1, eprev1, eintegral1, targets[0], kp1, kd1, ki1);

  // Motor 2 control
  float kp2 = 8.0; // PID constants for Motor 2
  float kd2 = 0.8;
  float ki2 = 0.05;
  controlMotor(motor2, PWM_M2, DIR_M2, ENCA_M2, posi2, pos2, prevT2, eprev2, eintegral2, targets[1], kp2, kd2, ki2);

  // Motor 3 control
  float kp3 = 12.0; // PID constants for Motor 3
  float kd3 = 1.2;
  float ki3 = 0.08;
  controlMotor(motor3, PWM_M3, DIR_M3, ENCA_M3, posi3, pos3, prevT3, eprev3, eintegral3, targets[2], kp3, kd3, ki3);

  // Motor 4 control
  float kp4 = 9.0; // PID constants for Motor 4
  float kd4 = 0.9;
  float ki4 = 0.06;
  controlMotor(motor4, PWM_M4, DIR_M4, ENCA_M4, posi4, pos4, prevT4, eprev4, eintegral4, targets[3], kp4, kd4, ki4);

  delay(10); // Delay to stabilize loop
}

void controlMotor(CytronMD &motor, int pwmPin, int dirPin, int encAPin, volatile int &posi, int &pos, long &prevT, float &eprev, float &eintegral, int target, float kp, float kd, float ki) {
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  int e = target - pos;
  float dedt = (e - eprev) / deltaT;
  eintegral += e * deltaT;

  float u = kp * e + kd * dedt + ki * eintegral;

  if (u > 255) u = 255;
  else if (u < -255) u = -255;

  motor.setSpeed(-1 * u); // Motor runs forward (adjust as per your motor direction configuration)

  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.println(u);
}

void readEncoder1() {
  if (digitalRead(ENCB_M1) == HIGH) {
    posi1++;
  } else {
    posi1--;
  }
}

void readEncoder2() {
  if (digitalRead(ENCB_M2) == HIGH) {
    posi2++;
  } else {
    posi2--;
  }
}

void readEncoder3() {
  if (digitalRead(ENCB_M3) == HIGH) {
    posi3++;
  } else {
    posi3--;
  }
}

void readEncoder4() {
  if (digitalRead(ENCB_M4) == HIGH) {
    posi4++;
  } else {
    posi4--;
  }
}
