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
print("gripper serial launched")
armState = ""
def grip_cb(msg):
    print("cb")
    global armState
    armState = msg.flip


rospy.init_node("pub2gripper")
rospy.Subscriber("gripper_control", GripperControl, callback=grip_cb)

try:

    while True:
    
        try:
            print("writing")
            
            ser1.write(armState.encode())
            print(f"armState:{armState}")
            # Reset state
            armState = ""
        except Exception as e:
            print("Fail", e)
        time.sleep(1)

except KeyboardInterrupt:
    print("Close Serial Communication")
    ser1.close()
