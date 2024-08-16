import serial
import time
import rospy

# from astra_camera.msg import MotorControl, GripperControl
from std_msgs.msg import String
import threading

# use match case once integrating keyboard, sync motor state is a must
# receive wasd then decide here what command to send F100, R10
# Attempt to connect to the serial port
ser2 = None

while ser2 is None:
    try:
        ser2 = serial.Serial("/dev/ttyUSB0", 115200, timeout=1.0)

    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)

ser2.reset_input_buffer()

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
        with lock:
            current_key = key
            print(f"current key: {current_key}")
            key = ""
        if not current_key:
            print("skip")
            time.sleep(0.5)
            continue

        try:
            print("Writing commands to serial")
            print(f"key: {current_key}")
            current_key = current_key + "\n"
            ser2.write(current_key.encode("utf-8"))

        except Exception as e:
            print(f"Failed to send command: {e}")
        time.sleep(0.5)


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
        ser2.close()

    finally:

        if ser2:
            ser2.close()
        print("Serial port closed.")
