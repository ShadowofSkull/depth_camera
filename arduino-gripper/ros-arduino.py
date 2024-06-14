#!/usr/bin/env python3

import rospy
from custom_msgs.msg import GripperControl 
import serial

# type lsusb -v , to get the port number of your usb on the device. 
ARDUINO_PORT = '/dev/ttyS0' #port on wei wen's computer for arduino 
BAUD_RATE = 9600 

arduino_serial = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)

def gripper_control_callback(msg):
    command = msg.command
    
    arduino_serial.write(command.encode()) #writes the command to the arduino port, sending the command to it to o (open), or c(close)

def gripper_control_node():
    rospy.init_node('gripper_control_node', anonymous=True)
    rospy.Subscriber('/gripper_control', GripperControl, gripper_control_callback)
    rospy.spin()

def getDepth(): 
    #'/camera/depth/image_raw'
    # get the values from this topic, and based on the conditions, if met, send o/c command to the gripper_control topic
    rospy.spin()

if __name__ == '__main__':
    try:
        gripper_control_node()
    except rospy.ROSInterruptException:
        pass
