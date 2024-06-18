#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <util/atomic.h>
#include "CytronMotorDriver.h"
#include <ros.h>
#include <std_msgs/String.h>

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE

volatile int posi = 0; // specify posi as volatile
int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
float u = 0;

CytronMD motor(PWM_DIR, 4, 5); // PWM = Pin 4, DIR = Pin 5.

ros::NodeHandle nh;

void command_cb(const std_msgs::String& cmd_msg) {
  String command = cmd_msg.data.c_str();
  
  if (command == "forward") {
    motor.setSpeed(255); // Max speed forward
  } else if (command == "backward") {
    motor.setSpeed(-255); // Max speed backward
  } else if (command == "left") {
    motor.setSpeed(128); // Half speed forward (example for turning left)
  } else if (command == "right") {
    motor.setSpeed(-128); // Half speed backward (example for turning right)
  } else {
    motor.setSpeed(0); // Stop
  }
  digitalWrite(13, HIGH - digitalRead(13)); // Toggle LED
}

ros::Subscriber<std_msgs::String> sub("motor_command", command_cb);

void setup() {
  Serial.begin(115200);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  motor.attach();
}

void loop() {
  nh.spinOnce();
  delay(1);
}

void readEncoder() {
  if (digitalRead(ENCB) == HIGH) {
    posi++;
  } else {
    posi--;
  }
}
