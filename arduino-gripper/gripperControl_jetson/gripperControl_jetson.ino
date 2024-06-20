#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <astra_camera/GripperControl.h> // Include the custom message header

ros::NodeHandle nh;

Servo servo1; // servo on pin 3
Servo servo2; // servo on pin 9
Servo servoH; // servo on pin 12

std_msgs::String ballInGripper_msg;
ros::Publisher ballInGripper_pub("BallInGripper", &ballInGripper_msg);

// Callback function for gripper control
void gripperControlCallback(const astra_camera::GripperControl& cmd_msg) {
  String gripCommand = cmd_msg.grip;
  String flipCommand = cmd_msg.flip;

  if (gripCommand == "o") {
    servo1.write(85); // Open position for servo1
    servo2.write(95); // Open position for servo2
  } else if (gripCommand == "c") {
    servo1.write(155); // Close position for servo1
    servo2.write(25);  // Close position for servo2
  }

  if (flipCommand == "forward") {
    servoH.write(180); // Flip position for servoH
  } else if (flipCommand == "backward") {
    servoH.write(0);   // Back position for servoH
  }

  digitalWrite(13, HIGH - digitalRead(13)); // Toggle LED
}

ros::Subscriber<astra_camera::GripperControl> gripperControlSub("gripper_control", gripperControlCallback);


const int irPin = A0; // IR sensor connected to analog pin A0

void setup() {
  pinMode(13, OUTPUT);
  // init node to use rosmsg
  nh.initNode();
  nh.subscribe(gripperControlSub);
  nh.advertise(ballInGripper_pub);

  servo1.attach(3);  // attach servo1 to pin 3
  servo2.attach(9);  // attach servo2 to pin 9
  servoH.attach(12); // attach servoH to pin 12

  servo1.write(75);  // Initial position for servo1
  servo2.write(105); // Initial position for servo2 
  servoH.write(0);   // Initial position for servoH (backward)

  pinMode(irPin, INPUT);
}

void loop() {
  nh.spinOnce();

  // Read IR sensor
  int irValue = analogRead(irPin);

  // Check if ball is detected
  if (irValue < 500) { // Adjust this threshold based on your IR sensor reading
    ballInGripper_msg.data = "y";
  } else {
    ballInGripper_msg.data = "n";
  }

  // Publish ball detection status
  ballInGripper_pub.publish(&ballInGripper_msg);

  delay(100); // Adjust delay as needed
}
