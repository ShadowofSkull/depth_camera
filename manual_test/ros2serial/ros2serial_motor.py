#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
import time

def serial_node():
    rospy.init_node('serial_node', anonymous=True)
    serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')  # Modify as per your setup
    serial_baud = rospy.get_param('~serial_baud', 115200)  # Match with Arduino serial baudrate
    pub = rospy.Publisher('motor_commands', String, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    try:
        ser = serial.Serial(serial_port, serial_baud, timeout=1)
        rospy.loginfo(f'Serial port connected: {serial_port}')
    except serial.SerialException as e:
        rospy.logerr(f'Failed to connect to {serial_port}: {e}')
        return

    while not rospy.is_shutdown():
        # Publish motor commands
        command = input("Enter command (F/B/L/R distance): ")
        ser.write(command.encode('utf-8') + b'\n')
        rospy.loginfo(f'Sending command: {command}')
        
        # Read serial response if needed
        # response = ser.readline().decode('utf-8').strip()
        # rospy.loginfo(f'Received: {response}')
        
        rate.sleep()

    ser.close()

if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass
