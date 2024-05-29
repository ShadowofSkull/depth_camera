
import rospy
from std_msgs.msg import Int16MultiArray
def callback(data):
    arr = data.data
    print(arr)
    
    for i in arr:
        print(i)

rospy.init_node("sub")
sub = rospy.Subscriber("test", Int16MultiArray, callback=callback, queue_size=10)
rate = rospy.Rate(30)
while not rospy.is_shutdown():
    
    rate.sleep()