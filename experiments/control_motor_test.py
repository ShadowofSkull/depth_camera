#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def send_motor_command(command):
    pub = rospy.Publisher('motor_command', String, queue_size=10)
    rospy.init_node('motor_commander', anonymous=True)
    
    msg = String()
    msg.data = command
    rospy.loginfo(f"Publishing: {msg.data}")
    pub.publish(msg)
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            command = input("Enter command (forward, backward, left, right): ").strip().lower()
            if command in ["forward", "backward", "left", "right"]:
                send_motor_command(command)
            else:
                print("Invalid command. Please enter 'forward', 'backward', 'left', or 'right'.")
    except rospy.ROSInterruptException:
        pass

