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
motorState = "1"  # stopped since motor not running at the start

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
    global x, z, armState, motorState
    
    while True:
        with lock:
            current_x = x
            current_z = z
            current_armState = armState
            x = 0  # Reset variable
            z = 0
            armState = ""
        
        if current_x == 0 and current_z == 0:
            continue
        
        try:
            print("Writing commands to serial")
            print(f"x: {current_x}, z: {current_z}, armState: {current_armState}")

            # Determine direction for x-axis movement
            direction_x = "R" if current_x >= 0 else "L"
            current_x = abs(current_x)
            
            # Send x direction movement command
            distance = f"{direction_x}{current_x}\n"
            print(f"distance: {distance}")
            ser1.write(distance.encode("utf-8"))

            # Wait for x axis movement completion
            motorState = ser1.read_until(b"\n").decode("utf-8").rstrip()
            print(f"Received motorState after x move: {motorState}")

            # If armState is set, send z direction movement command
            if current_armState:
                distance = f"{current_armState}{current_z}\n"
                print(f"distance: {distance}")
                ser1.write(distance.encode('utf-8'))

                # Wait for z axis movement completion
                motorState = ser1.read_until(b"\n").decode("utf-8").rstrip()
                print(f"Received motorState after z move: {motorState}")

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
