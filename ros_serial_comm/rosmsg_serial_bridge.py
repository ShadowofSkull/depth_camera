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
        ser1 = serial.Serial('/dev/ttyACM0', 115200, timeout = 1.0)
    except:
        print("error")
        # retry every 1 sec
        time.sleep(1)
ser1.reset_input_buffer()
print("Serial OK")
x = 0
z = 0
armState = ""
direction_x = ""
direction_z = ""
distance = ""

def motor_cb(msg):
    print("callback")
    global x, z
    x = msg.x
    z = msg.z
    

def grip_cb(msg):
    print("grip")
    global armState
    armState = msg.flip
    armState = armState[0]
    armState = armState.capitalize()

rospy.init_node("pub_xz_to_serial")
rospy.Subscriber("motor_control", MotorControl, callback=motor_cb)
rospy.Subscriber("gripper_control", GripperControl, callback=grip_cb)

try:
    # rate = rospy.Rate(0.1)
    
    while not rospy.is_shutdown():
        if x != 0 and z != 0:
            
            try:
                print("writing")
                print(x, z)
                print(armState)
                if x < 0:
                    direction_x = "L"
                else:
                    direction_x = "R"
                distance =  direction_x + str(x)
                # write for x direction movement first
                ser1.write(distance.encode('utf=8'))
                # delay for x movement to finish
                time.sleep(5)
                if armState != "":
                    # carry out z movement
                    distance = armState + str(z)
                    ser1.write(distance.encode('utf=8'))
            except Exception as e:
                print("Fail", e)
        # rate.sleep()
        time.sleep(5)

except KeyboardInterrupt:
    print("Close Serial Communication")
    ser1.close()