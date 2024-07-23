import serial
import time
import rospy
from astra_camera.msg import MotorControl, GripperControl
import threading

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
    
    # Hardcode initial movements
    # ser1.write("R4375\n".encode())
    # time.sleep(5)
    # ser1.write("B2800\n".encode())
    # time.sleep(5)
    # ser1.write("R2750\n".encode())
    # time.sleep(5)

    while True:
        with lock:
            current_x = x
            current_z = z
            current_armState = armState
            x = 0 # Reset variable
            z = 0
            armState = ""
        if current_x == 0 and current_z == 0 and armState:
            continue
        

        try:
            print("Writing commands to serial")
            print(f"x: {current_x}, z: {current_z}, armState: {current_armState}")

            # Determine direction for x-axis movement
            if current_x < 0:
                direction_x = "L"
                current_x = abs(current_x)
            else:
                direction_x = "R"
            # Update motorState (arduino side need modification to work with this)
            # if ser1.in_waiting > 0:
            #     line = ser1.readline().decode("utf-8").rstrip()  # Decode the received data
            #     print(f"Received motorstate 1: {line}")  # Debug print
            #     motorState = line
            # Only send move commands if the motor stopped so skip the command if not
            # 1 means stop 0 is running
            # if motorState == "0":
            #     print("in skip")
            #     time.sleep(1)
            #     continue

            # Send x direction movement command
            distance = direction_x + str(current_x) + "\n"
            # print(f"Distance command: {distance}")
            print(f"distance {distance}")
            ser1.write(distance.encode("utf-8"))
            time.sleep(1)
            # Verify x axis movement
            if ser1.in_waiting > 0:
                line = ser1.read_until(b"\n").decode("utf-8").rstrip()  # Decode the received data
                print(f"Received x move: {line}")  # Debug print
            # Update motorState
            if ser1.in_waiting > 0:
                line = ser1.read_until(b"\n").decode("utf-8").rstrip()  # Decode the received data
                print(f"Received motorstate start should be 0: {line}")  # Debug print
                motorState = line
                print(f"Motor state should be 0: {motorState}")
            # Wait till motor stop to proceed
            while motorState == "0":
                print(motorState)
                print("wait till motor stop")
                # Update motorState
                if ser1.in_waiting > 0:
                    line = ser1.read_until(b"\n").decode("utf-8").rstrip()  # Decode the received data
                    print(f"Received motorstate stop should be 1: {line}")  # Debug print
                    motorState = line
            # If armState is set, send z direction movement command
            if current_armState:
                # print(f"Distance command: {distance}")

                # ser1.reset_input_buffer()
                distance = current_armState + str(current_z) + "\n"
                print(distance)
                ser1.write(distance.encode('utf-8'))
                time.sleep(1)
                # print(motorState)
                # Verify z axis movement
                if ser1.in_waiting > 0:
                    line = ser1.read_until(b"\n").decode("utf-8").rstrip()  # Decode the received data
                    print(f"Received z move: {line}")  # Debug print
                # Update motorState
                if ser1.in_waiting > 0:
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
                        print(motorState)
                  
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
        print("Serial port closed.")