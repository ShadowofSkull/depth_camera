#!/usr/bin/env python

import rospy
from astra_camera.msg import SiloMatrix

def callback(data):    
    siloInfos = []
    for silo in data.siloMatrix.split(", "):
        print(silo)
        indiSilo = [int(silo[i]) for i in range(len(silo))]
        siloInfos.append(indiSilo)

    print(siloInfos)
    
    #rospy.loginfo("Received data: %s", data.siloMatrix)
    # Process the data as needed
    # For example, you can convert it back to a 2D array form:
    #array_2d = data.data


def subscriber():
    rospy.init_node('silo_matrix_subscriber', anonymous=True)
    rospy.Subscriber('silo_matrix', SiloMatrix, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass

