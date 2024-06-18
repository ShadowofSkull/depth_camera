#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Int32

def callback(data):
    matrix = [data.data[i:i+3] for i in range(0, len(data.data), 3)]
    rospy.loginfo(f"Received matrix: {matrix}")

    # Determine best silo to place the ball based on priorities
    best_silo = -1
    priority_1 = []
    priority_2 = []
    priority_3 = []

    for i, silo in enumerate(matrix):
        if silo[1] == 2:  # Priority 1
            priority_1.append(i)
        elif silo[1] == 1:  # Priority 2
            priority_2.append(i)
        elif silo[0] == 0:  # Priority 3
            priority_3.append(i)

    if priority_1:
        best_silo = priority_1[0]  # Choose first silo with opponentBall in second row
    elif priority_2:
        best_silo = priority_2[0]  # Choose first silo with teamBall in second row
    elif priority_3:
        best_silo = priority_3[0]  # Choose first empty silo

    rospy.loginfo(f"Best silo to place the ball: {best_silo}")
    pub.publish(best_silo)

    # Check for V Goal condition
    check_v_goal(matrix)

def check_v_goal(matrix):
    team_color = 1  # Assuming 1 represents team color
    v_goal_count = 0

    for silo in matrix:
        if silo[2] == team_color and silo.count(team_color) >= 2:
            v_goal_count += 1

    if v_goal_count >= 3:
        rospy.loginfo("V Goal achieved! Team wins!")
        # Additional actions for V Goal can be added here

def listener():
    rospy.init_node('silo_decider', anonymous=True)
    rospy.Subscriber('silo_matrix', Int32MultiArray, callback)
    global pub
    pub = rospy.Publisher('best_silo', Int32, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

