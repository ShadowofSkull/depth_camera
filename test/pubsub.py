import rospy
from std_msgs.msg import Int16MultiArray


def callback(data):
    subArr = data.data
    tmp = []
    for i in subArr:
        tmp.append(i)
    arr.data = tmp
    pub.publish(arr)
    rate.sleep()


rospy.init_node("pub")
rate = rospy.Rate(30)
arr = Int16MultiArray()
pub = rospy.Publisher("test", Int16MultiArray, queue_size=10)
sub = rospy.Subscriber("test2", Int16MultiArray, queue_size=10, callback=callback)
rospy.spin()
