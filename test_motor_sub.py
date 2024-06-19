#!/usr/bin/env python

import rospy
from astra_camera.msg import SiloPath
import math

def calculate_path():
    pub = rospy.Publisher('silo_path', SiloPath, queue_size=10)
    rospy.init_node('path_calculator', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        msg = SiloPath()
        
        # write algorithm to determine the target motor movements to move forward, backwards, left, right and diagonally to reach the silo. 
        
        # sample target 
        msg.motor1_target = int(500 * math.sin(rospy.get_time()))
        msg.motor2_target = int(800 * math.cos(rospy.get_time()))
        msg.motor3_target = int(1000 * math.sin(rospy.get_time()))
        msg.motor4_target = int(600 * math.cos(rospy.get_time()))
        
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        calculate_path()
    except rospy.ROSInterruptException:
        pass

