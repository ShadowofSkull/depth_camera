#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from astra_camera.msg import MotorControl, GripperControl

def publish_motor_control(pub_motor, motor1_speed, motor2_speed, motor3_speed, motor4_speed):
    motor_control_msg = MotorControl()
    motor_control_msg.motor1_speed = motor1_speed
    motor_control_msg.motor2_speed = motor2_speed
    motor_control_msg.motor3_speed = motor3_speed
    motor_control_msg.motor4_speed = motor4_speed
    pub_motor.publish(motor_control_msg)

def publish_gripper_control(pub_gripper, state):
    gripper_control_msg = GripperControl()
    gripper_control_msg.state = state
    pub_gripper.publish(gripper_control_msg)

def controller_node():
    rospy.init_node('controller_node')

    pub_motor = rospy.Publisher('motor_control', MotorControl, queue_size=10)
    pub_gripper = rospy.Publisher('gripper_control', GripperControl, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Example commands for testing
        # Replace these with actual logic or inputs
        publish_motor_control(pub_motor, 100, 100, 100, 100)  # Move forward
        rospy.sleep(1)
        publish_motor_control(pub_motor, -100, -100, -100, -100)  # Move backward
        rospy.sleep(1)
        publish_motor_control(pub_motor, 100, -100, 100, -100)  # Turn left
        rospy.sleep(1)
        publish_motor_control(pub_motor, -100, 100, -100, 100)  # Turn right
        rospy.sleep(1)

        publish_gripper_control(pub_gripper, "o")  # Open gripper
        rospy.sleep(1)
        publish_gripper_control(pub_gripper, "c")  # Close gripper
        rospy.sleep(1)
        publish_gripper_control(pub_gripper, "flip")  # Flip gripper
        rospy.sleep(1)
        publish_gripper_control(pub_gripper, "back")  # Move gripper back
        rospy.sleep(1)

        rate.sleep()

if __name__ == '__main__':
    try:
        controller_node()
    except rospy.ROSInterruptException:
        pass

