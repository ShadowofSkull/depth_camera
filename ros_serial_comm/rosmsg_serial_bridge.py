import serial
import time
import rospy
from astra_camera.msg import MotorControl, GripperControl

ser1 = None
while ser1 == None:
    try:
        ser1 = serial.Serial('/dev/ttyACM0', 115200, timeout = 1.0)
    except:
        print("error")
        time.sleep(1)
ser1.reset_input_buffer()
print("Serial OK")
x = ""
z = ""
distance = ""
armState = ""
def motor_cb(msg):
    print("callback")
    global x, z, distance, armState
    x = str(msg.x)
    z = str(msg.z)
    distance = armState + z
    print(f"distance:{distance}")

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
                ser1.write(distance.encode('utf=8'))
            except Exception as e:
                print("Fail", e)
        # rate.sleep()
        time.sleep(5)
#             ser2.flush()
#         if distance:
#             while ser2.in_waiting <= 0:
#                 time.sleep(0.01)
#                 response = ser2.readline().decode('utf-8').rstrip()
#                 print(response)
#             line = ser1.readline ().decode("utf=8").rstrip()
#             print(line)        
#         print("Send message to Arduino 2")
#         ser2.write(line.encode('utf-8'))
#         while ser2.in_waiting <= 0:
#             time.sleep(0.01)
#         response = ser2.readline().decode('utf-8').rstrip()
#         print(response)
        
except KeyboardInterrupt:
    print("Close Serial Communication")
    ser1.close()