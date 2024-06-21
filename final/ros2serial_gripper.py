import serial
import time
import rospy
from astra_camera.msg import MotorControl, GripperControl

# this code is to acquired rosmsg in python and publish it by serial to arduino
# keep trying to connect to serial port
ser1 = None
while ser1 == None:
    try:
        # make sure to change the port to the correct port
        ser1 = serial.Serial("/dev/ttyUSB0", 115200, timeout=1.0)
    except:
        print("error")
        # retry every 1 sec
        time.sleep(1)
ser1.reset_input_buffer()
print("Serial OK")

armState = ""
clawState = ""
action = ""


def grip_cb(msg):
    print("cb")
    global armState, clawState
    clawState = msg.grip
    armState = msg.flip
    armState = armState[0]
    armState = armState.capitalize()


rospy.init_node("pub2gripper")
rospy.Subscriber("gripper_control", GripperControl, callback=grip_cb)

try:
    # rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        if armState != "" and clawState != "":
            try:
                print("writing")
                print(armState)
                print(clawState)
                action = clawState
                ser1.write(action.encode("utf=8"))
                time.sleep(2)
                action = armState
                ser1.write(action.encode("utf=8"))

            except Exception as e:
                print("Fail", e)
        time.sleep(3)

except KeyboardInterrupt:
    print("Close Serial Communication")
    ser1.close()
