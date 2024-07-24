#!/usr/bin/env python

from __future__ import print_function

import rospy
from std_msgs.msg import Float64
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
    'i': (1, 1, 1, 1),
    'o': (1, -1, 1, -1),
    'j': (-1, 1, 1, -1),
    'l': (1, -1, -1, 1),
    'u': (1, 1, -1, -1),
    ',': (-1, -1, -1, -1),
    '.': (-1, 1, -1, 1),
    'm': (-1, -1, 1, 1),
    'O': (1, -1, 1, -1),
    'I': (1, 1, 1, 1),
    'J': (-1, 1, 1, -1),
    'L': (1, -1, -1, 1),
    'U': (1, 1, -1, -1),
    '<': (-1, -1, -1, -1),
    '>': (-1, 1, -1, 1),
    'M': (-1, -1, 1, 1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
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

    rospy.init_node('vel_Publisher')
    publ = rospy.Publisher('/open_base/left_joint_velocity_controller/command', Float64, queue_size=1)
    pubb = rospy.Publisher('/open_base/back_joint_velocity_controller/command', Float64, queue_size=1)
    pubr = rospy.Publisher('/open_base/right_joint_velocity_controller/command', Float64, queue_size=1)
    pubf = rospy.Publisher('/open_base/front_joint_velocity_controller/command', Float64, queue_size=1)  # New publisher for the fourth wheel

    speed = 1.0
    x = 0
    y = 0
    z = 0
    w = 0
    status = 0

    try:
        print(msg)
        print(vels(speed))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                w = moveBindings[key][3]  # Velocity for the fourth wheel
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                print(vels(speed))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                w = 0
                if (key == '\x03'):
                    break

            vell = Float64()
            velb = Float64()
            velr = Float64()
            velf = Float64()

            vell.data = x * speed
            velb.data = y * speed
            velr.data = z * speed
            velf.data = w * speed

            publ.publish(vell)
            pubb.publish(velb)
            pubr.publish(velr)
            pubf.publish(velf)

    except Exception as e:
        print(e)

    finally:
        vell = Float64()
        velb = Float64()
        velr = Float64()
        velf = Float64()

        vell.data = 0.0
        velb.data = 0.0
        velr.data = 0.0
        velf.data = 0.0

        publ.publish(vell)
        pubb.publish(velb)
        pubr.publish(velr)
        pubf.publish(velf)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)