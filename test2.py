import rospy
from geometry_msgs.msg import Point

def action_callback(msg):
    layer = int(msg.x)
    col = int(msg.y)
    ball = int(msg.z)
    # Add your code here to place the ball at the specified position
    print(f"Place ball {ball} at layer {layer}, column {col}")

def action_listener():
    rospy.init_node('action_listener', anonymous=True)
    rospy.Subscriber('/robot/place_ball', Point, action_callback)
    rospy.spin()

if __name__ == '__main__':
    action_listener()

