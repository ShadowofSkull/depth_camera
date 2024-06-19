#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include "CytronMotorDriver.h"
#include <math.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#define ENCA1 18 // Encoder 1 (LB)
#define ENCB1 17 // Encoder 1
#define ENCA2 2 // Encoder 2 (LF)
#define ENCB2 3 // Encoder 2
#define ENCA3 19 // Encoder 3 (RF)
#define ENCB3 20 // Encoder 3
#define ENCA4 21 // Encoder 4 (RB)
#define ENCB4 16 // Encoder 4

volatile int posi1 = 0, posi2 = 0, posi3 = 0, posi4 = 0; // Positions
int pos1, pos2, pos3, pos4;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
float u = 0;
float target = 0;

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
    int16_t x = msg.x;
    int16_t z = msg.z;

    // Handle the x and z values as needed
    // For this example, we just print them
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(", z: ");
    Serial.println(z);
}

void gripperControlCallback(const astra_camera::GripperControl &msg) {
    if (msg.flip == "forward") {
        // Set motors to move forward
        motorState = RUNNING;
        target = 1000 ; // Set appropriate target for moving forward
    } else if (msg.flip == "backward") {
        // Set motors to move backward
        motorState = RUNNING;
        target = -1000 ; // Set appropriate target for moving backward
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
    delay(10);

    pos1 = 0;
    pos2 = 0;
    pos3 = 0;
    pos4 = 0;

    // Initialize ROS node handle and subscribers
    nh.initNode();
    nh.subscribe(motorControlSub);
    nh.subscribe(gripperControlSub);
}

void loop() {
    // Read serial input and update target
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        target = command.toFloat() * 6 / PI; // Update target position
        Serial.println(target);

        // Reset motor state to RUNNING when new target is set
        motorState = RUNNING;
    }

    // PID constants - motor 1 and motor 2 and motor 4
    float kp = 10.0;
    float kd = 1.0;
    float ki = 0.1;

    // time difference
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;

    // Read the position in an atomic block to avoid a potential
    // misread if the interrupt coincides with this code running
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pos1 = posi1;
        pos2 = posi2;
        pos3 = posi3;
        pos4 = posi4;
    }

    int pos = (pos1 + pos2 + pos3 + pos4) / 4;

    // error
    int e = target - pos; // motor 1 and motor 3 and motor 4

    // derivative
    float dedt = (e - eprev) / deltaT;

    // integral
    eintegral += e * deltaT;

    // control signal
    float u = kp * e;

    // Limit control signal to prevent excessive motor speed
    if (u > 75) u = 75;
    else if (u < -75) u = -75;

    // Set motor speeds based on current state
    if (motorState == RUNNING) {
        motor1.setSpeed(u); // Motor 1 runs forward
        motor2.setSpeed(u); // Motor 2 and Motor 3 and Motor 4 runs forward
        motor3.setSpeed(u); // Motor 2 and Motor 3 and Motor 4 runs forward
        motor4.setSpeed(u); // Motor 2 and Motor 3 and Motor 4 runs forward
    } else {
        motor1.setSpeed(0); // Stop Motor 1
        motor2.setSpeed(0); // Stop Motor 2
        motor3.setSpeed(0); // Stop Motor 3
        motor4.setSpeed(0); // Stop Motor 4
    }

    // Store previous error
    eprev = e;

    // Print debug information
    Serial.print(target);
    Serial.print(" ");
    Serial.print(pos);
    Serial.print(" ");
    Serial.print(pos1);
    Serial.print(" ");
    Serial.print(pos2);
    Serial.print(" ");
    Serial.print(pos3);
    Serial.print(" ");
    Serial.print(pos4);
    Serial.print(" ");
    Serial.println(u);

    // Check if any motor has reached the target
    if (motorState == RUNNING) {
        if (pos1 == target || pos2 == target || pos3 == target || pos4 == target) {
            // Stop all motors
            motor1.setSpeed(0);
            motor2.setSpeed(0);
            motor3.setSpeed(0);
            motor4.setSpeed(0);

            // Set state to STOPPED
            motorState = STOP
