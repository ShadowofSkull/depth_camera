#!/usr/bin/env python

import rospy
from astra_camera.msg import SiloMatrix, BestSilo

# Define constants for ball types
EMPTY = 0
TEAM_BALL = 1
OPPONENT_BALL = 2
PURPLE_BALL = 3

def callback(data):
    # Convert received data into a matrix format for easier processing
    matrix = [
        data.silo1,
        data.silo2,
        data.silo3,
        data.silo4,
        data.silo5
    ]

    rospy.loginfo(f"Received matrix: {matrix}")

    # Determine the best silo to place the ball based on priorities
    best_silo = -1
    priority_1 = []
    priority_2 = []
    priority_3 = []

    for i, silo in enumerate(matrix):
        # Check if silo is already filled
        if silo.count(EMPTY) == 0:
            continue  # Skip silos that are already filled

        # Determine priorities based on ball types
        if silo[1] == OPPONENT_BALL:
            priority_1.append(i)  # Priority 1: Opponent ball in second position
        elif silo[1] == TEAM_BALL:
            priority_2.append(i)  # Priority 2: Team ball in second position
        elif silo[0] == EMPTY:
            priority_3.append(i)  # Priority 3: Empty silo

    if priority_1:
        best_silo = priority_1[0]
    elif priority_2:
        best_silo = priority_2[0]
    elif priority_3:
        best_silo = priority_3[0]

    rospy.loginfo(f"Best silo to place the ball: {best_silo}")

    # Publish the best silo index
    best_silo_msg = BestSilo()
    best_silo_msg.index = best_silo # need to assign the index to the msg, its variable name should be same as the variable in the msg index
    best_silo_pub.publish(best_silo_msg)

    # Check for V Goal condition
    check_v_goal(matrix)

def check_v_goal(matrix):
    team_color = TEAM_BALL
    v_goal_count = 0

    for silo in matrix:
        if silo.count(EMPTY) == 0:
            continue  # Skip silos that are fully filled

        if silo[2] == team_color and silo.count(team_color) >= 1:
            v_goal_count += 1

    if v_goal_count >= 3:
        rospy.loginfo("V Goal achieved! Team wins!")
        # Additional actions for V Goal can be added here

def listener():
    global best_silo_pub
    rospy.init_node('silo_decider', anonymous=True)

    # Initialize the publisher
    best_silo_pub = rospy.Publisher('best_silo', BestSilo, queue_size=10)
    
    rospy.Subscriber('silo_matrix', SiloMatrix, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

