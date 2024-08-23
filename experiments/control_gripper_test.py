#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def send_servo_command(command):
    pub = rospy.Publisher('servo', String, queue_size=10)
    rospy.init_node('servo_commander', anonymous=True)
    
    msg = String()
    msg.data = command
    rospy.loginfo(f"Publishing: {msg.data}")
    pub.publish(msg)
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            command = input("Enter command (o, c, flip, back): ").strip()
            if command in ["o", "c", "flip", "back"]:
                send_servo_command(command)
            else:
                print("Invalid command. Please enter 'o', 'c', 'flip', or 'back'.")
    except rospy.ROSInterruptException:
        pass

