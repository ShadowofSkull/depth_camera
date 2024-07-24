import serial
import time
import rospy
from astra_camera.msg import MotorControl, GripperControl
import threading
# use match case once integrating keyboard, sync motor state is a must
# receive wasd then decide here what command to send F100, R10
# Attempt to connect to the serial port
ser1 = None
while ser1 is None:
    try:
        ser1 = serial.Serial("/dev/ttyACM0", 115200, timeout=1.0)
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)

ser1.reset_input_buffer()
print("Serial OK")
print("Motor serial launched")
time.sleep(3)
print("after 3 sec delay")

time.sleep(3)
print("delay end")
# Global variables
x = 0
z = 0
armState = ""
direction_x = ""
distance = ""
motorState = "1" # stopped since motor not running at the start

# Lock for thread safety
lock = threading.Lock()

# Callback functions
def motor_cb(msg):
    global x, z
    with lock:
        x = msg.x
        z = msg.z

def grip_cb(msg):
    global armState
    with lock:
        armState = msg.flip
        if armState:
            armState = armState[0].capitalize()


def main_loop():
    global x, z, armState, direction_x, distance, motorState
    

    while True:
        print("start")
        with lock:
            current_x = x
            current_z = z
            current_armState = armState
            x = 0 # Reset variable
            z = 0
            armState = ""
        if current_x == 0 and current_z == 0 and current_armState:
            continue
        

        try:
            print("Writing commands to serial")
            print(f"x: {current_x}, z: {current_z}, armState: {current_armState}")

            distance = current_armState + str(current_z) + "\n"
            print(distance)
            ser1.write(distance.encode('utf-8'))
            time.sleep(1)
            # print(motorState)
            # Verify z axis movement
            # while ser1.in_waiting == 0:
            #     print("in wait")
            #     time.sleep(0.1)

            # # if ser1.in_waiting > 0:
            # line = ser1.read_until(b"\n").decode("utf-8").rstrip()  # Decode the received data
            # print(f"Received z move: {line}")  # Debug print
            # Update motorState
            while ser1.in_waiting == 0:
                print("in wait")
                time.sleep(0.1)

            # if ser1.in_waiting > 0:
            line = ser1.read_until(b"\n").decode("utf-8").rstrip()  # Decode the received data
            print(f"Received motorstate start 2 should be 0: {line}")  # Debug print
            motorState = line
            print(f"0 properly assigned to motorState: {motorState}")
            
            while motorState == "0":
                print(motorState)
                print("wait till motor stop")
                # Update motorState
                if ser1.in_waiting > 0:
                    line = ser1.read_until(b"\n").decode("utf-8").rstrip()  # Decode the received data
                    print(f"Received motorstate stop 2 should be 1: {line}")  # Debug print
                    motorState = line
                    print(f"motor is 1: {motorState}")
            print("end")
                  
        except Exception as e:
            print(f"Failed to send command: {e}")
        time.sleep(5)

if __name__ == "__main__":
    try:
        # Initialize ROS node and subscribers
        rospy.init_node("pub_xz_to_serial")
        rospy.Subscriber("motor_control", MotorControl, callback=motor_cb)
        rospy.Subscriber("gripper_control", GripperControl, callback=grip_cb)

        main_thread = threading.Thread(target=main_loop)
        main_thread.start()
        rospy.spin()
    except KeyboardInterrupt:
        print("Closing Serial Communication")
        ser1.close()
    finally:
        if ser1:
            ser1.close()
        main_thread.join()
        print("Serial port closed.")