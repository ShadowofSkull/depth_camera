#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray

def publish_matrix(matrix):
    pub = rospy.Publisher('silo_matrix', Int32MultiArray, queue_size=10)
    rospy.init_node('matrix_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        msg = Int32MultiArray()
        msg.data = sum(matrix, [])  # Flatten the matrix
        rospy.loginfo(f"Publishing matrix: {msg.data}")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Example matrix (5 silos with 3 rows each)
        # 0 - empty, 1 - team ball, 2 - opponent ball
        matrix = [
            [1, 1, 1],
            [1, 1, 0],
            [2, 1, 0],
            [2, 2, 0],
            [1, 0, 0]
        ]
        publish_matrix(matrix)
    except rospy.ROSInterruptException:
        pass
