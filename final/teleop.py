#!/usr/bin/env python

from __future__ import print_function

import rospy
from std_msgs.msg import String
import sys, select, termios, tty

msg = """
Reading from the keyboard !
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >


anything else : stop

q/z : increase/decrease max speeds by 10%

CTRL-C to quit
"""

moveBindings = {
    "i": (1, 1, 1, 1),
    "o": (1, -1, 1, -1),
    "j": (-1, 1, 1, -1),
    "l": (1, -1, -1, 1),
    "u": (1, 1, -1, -1),
    ",": (-1, -1, -1, -1),
    ".": (-1, 1, -1, 1),
    "m": (-1, -1, 1, 1),
    "O": (1, -1, 1, -1),
    "I": (1, 1, 1, 1),
    "J": (-1, 1, 1, -1),
    "L": (1, -1, -1, 1),
    "U": (1, 1, -1, -1),
    "<": (-1, -1, -1, -1),
    ">": (-1, 1, -1, 1),
    "M": (-1, -1, 1, 1),
}

speedBindings = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed):
    return "currently:\tspeed %s " % (speed)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    validKeys = ["w", "a", "s", "d", "j", "k", "l", ";"]
    rospy.init_node("vel_Publisher")
    pub = rospy.Publisher("keyboard", String, queue_size=1)

    # publ = rospy.Publisher('/open_base/left_joint_velocity_controller/command', Float64, queue_size=1)
    # pubb = rospy.Publisher('/open_base/back_joint_velocity_controller/command', Float64, queue_size=1)
    # pubr = rospy.Publisher('/open_base/right_joint_velocity_controller/command', Float64, queue_size=1)
    # pubf = rospy.Publisher('/open_base/front_joint_velocity_controller/command', Float64, queue_size=1)  # New publisher for the fourth wheel

    try:
        print(msg)
        while 1:
            key = getKey()
            if key in validKeys:
                pub.publish(key)
            if key == "\x03":
                break

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
