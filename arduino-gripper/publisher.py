

import rospy
from astra_camera.msg import GripperControl

def gripper_command_publisher():
    rospy.init_node('gripper_command_publisher', anonymous=True)
    gripper_control_pub = rospy.Publisher('/gripper_control', GripperControl, queue_size=10)

    rate = rospy.Rate(10)  

    while not rospy.is_shutdown():
        command = input("Enter gripper command (o/c): ")

        gripper_control_msg = GripperControl()
        gripper_control_msg.grip = command
        gripper_control_msg.flip = command

        gripper_control_pub.publish(gripper_control_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        gripper_command_publisher()
    except rospy.ROSInterruptException:
        pass
