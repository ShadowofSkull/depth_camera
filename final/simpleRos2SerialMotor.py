import serial
import time
import rospy

# from astra_camera.msg import MotorControl, GripperControl
from std_msgs.msg import String
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


# Init
key = ""
# Lock for thread safety
lock = threading.Lock()


def key_cb(msg):
    global key
    with lock:
        key = msg.data


def main_loop():
    global key

    while True:
        print("start")
        with lock:
            current_key = key
            key = ""
        if key:
            continue

        try:
            print("Writing commands to serial")
            print(f"key: {key}")

            ser1.write(key.encode("utf-8"))

        except Exception as e:
            print(f"Failed to send command: {e}")
        time.sleep(0.1)


if __name__ == "__main__":
    try:
        # Initialize ROS node and subscribers
        rospy.init_node("pub_key_to_serial")
        rospy.Subscriber("keyboard", String, callback=key_cb)

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
