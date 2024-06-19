#include <ros.h>
#include <std_msgs/Float32.h>
#include <astra_camera/MotorControl.h>  // Replace 'your_package_name' with your actual ROS package name

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
void motorControlCallback(const astra_camera::SiloPath& msg) {
  target1 = msg.motor1_speed;  // Set target speed for motor 1
  target2 = msg.motor2_speed;  // Set target speed for motor 2
  target3 = msg.motor3_speed;  // Set target speed for motor 3
  target4 = msg.motor4_speed;  // Set target speed for motor 4
}

ros::Subscriber<astra_camera::SiloPath> sub("check_the_published_topic", motorControlCallback);

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

  // PID constants
  float kp = 10.0;
  float kd = 1.0;
  float ki = 0.1;
  float ki23 = 0.5;

  // Time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = posi1;
    pos2 = posi2;
    pos3 = posi3;
    pos4 = posi4;
  }
  
  // Error calculation
  int e1 = pos1 - target1;
  int e2 = target2 - pos2;
  int e3 = pos3 - target3;
  int e4 = pos4 - target4;

  // Derivative
  float dedt1 = (e1 - eprev1) / deltaT;
  float dedt2 = (e2 - eprev2) / deltaT;
  float dedt3 = (e3 - eprev3) / deltaT;
  float dedt4 = (e4 - eprev4) / deltaT;

  // Integral
  eintegral1 += e1 * deltaT;
  eintegral2 += e2 * deltaT;
  eintegral3 += e3 * deltaT;
  eintegral4 += e4 * deltaT;

  // Control signal
  u1 = kp * e1 ;
  u2 = kp * e2 ;
  u3 = kp * e3 ;
  u4 = kp * e4 ;

  // Clamp control signals to motor limits
  if(u1 > 255) u1 = 255;
  else if(u1 < -255) u1 = -255;
  if(u2 > 255) u2 = 255;
  else if(u2 < -255) u2 = -255;  
  if(u3 > 255) u3 = 255;
  else if(u3 < -255) u3 = -255;  
  if(u4 > 255) u4 = 255;
  else if(u4 < -255) u4 = -255;

  motor1.setSpeed(u1);
  motor2.setSpeed(u2);
  motor3.setSpeed(u3);
  motor4.setSpeed(u4);

  // Store previous error
  eprev1 = e1;
  eprev2 = e2;
  eprev3 = e3;
  eprev4 = e4;

  delay(10);
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
    posi4--;
  }
}
