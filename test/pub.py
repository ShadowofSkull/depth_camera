
import rospy
from std_msgs.msg import Int16MultiArray



rospy.init_node("pub2")
rate = rospy.Rate(30)
arr = Int16MultiArray()

pub = rospy.Publisher("test2", Int16MultiArray, queue_size=10)
while not rospy.is_shutdown():
    tmp = []
    for i in range(10):
        tmp.append(i)
    arr.data = tmp
    pub.publish(arr)
    rate.sleep()