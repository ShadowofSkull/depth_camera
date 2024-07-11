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
            if ser1.in_waiting > 0:
                line = ser1.readline().decode("utf-8").rstrip()  # Decode the received data
                print(f"Received stop: {line}")  # Debug print
                motorState = line
            # Only send move commands if the motor stopped so skip the command if not
            # 1 means stop 0 is running
            if motorState == "0":
                print("in skip")
                time.sleep(1)
                continue
            
            # Send x direction movement command
            distance = direction_x + str(current_x) + "\n"
            print(f"Distance command: {distance}")
            ser1.write(distance.encode("utf-8"))
            time.sleep(5)  # Delay to allow for x movement

            if ser1.in_waiting > 0:
                line = ser1.readline().decode("utf-8").rstrip()  # Decode the received data
                print(f"Received: {line}")  # Debug print

            # If armState is set, send z direction movement command
            if current_armState:
                distance = current_armState + str(current_z) + "\n"
                print(f"Distance command: {distance}")
                ser1.write(distance.encode('utf-8'))
                
                    
            time.sleep(2)  # Short delay before next iteration
            if ser1.in_waiting > 0:
                line = ser1.readline().decode("utf-8").rstrip()  # Decode the received data
                print(f"Received: {line}")  # Debug print
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
