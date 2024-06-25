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
        ser1 = serial.Serial("/dev/ttyACM0", 115200, timeout=1.0)
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
    # print("callback")
    global x, z
    x = msg.x
    z = msg.z




def grip_cb(msg):
    # print("grip")
    global armState
    armState = msg.flip
    if armState != "":
        armState = armState[0]
    armState = armState.capitalize()


rospy.init_node("pub_xz_to_serial")
rospy.Subscriber("motor_control", MotorControl, callback=motor_cb)
rospy.Subscriber("gripper_control", GripperControl, callback=grip_cb)
# bro pls rmb add \n
try:
    # Hardcode for area1 to 3 movement
    ser1.write("R4375\n".encode())
    print(r"R4375\n")
    time.sleep(5)
    ser1.write("B2800\n".encode())
    time.sleep(5)

    ser1.write("R2750\n".encode())
    time.sleep(5)

    while True:
        print(f"x{x},z {z}")

        try:
            print("writing")
            print(x, z)
            print(armState)
            if x < 0:
                direction_x = "L"
                x = x * -1
            else:
                direction_x = "R"
            distance = direction_x + str(x) + "\n"
            print(distance)
            # write for x direction movement first
            ser1.write(distance.encode())
            # delay for x movement to finish
            time.sleep(5)
            if armState != "":
                # carry out z movement
                distance = armState + str(z) + "\n"
                print(distance)
                ser1.write(distance.encode())
            time.sleep(5)
        except Exception as e:
            print("Fail", e)
        time.sleep(1)

except KeyboardInterrupt:
    print("Close Serial Communication")
    ser1.close()
