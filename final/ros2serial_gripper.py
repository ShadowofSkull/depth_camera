import serial
import time
import rospy
from astra_camera.msg import GripperControl
from threading import Thread, Lock

ser1 = None
armState = ""
lock = Lock()

def setup_serial():
    global ser1
    while ser1 is None:
        try:
            # Change the port to the correct port
            ser1 = serial.Serial("/dev/ttyUSB0", 115200, timeout=1.0)
            ser1.reset_input_buffer()
            print("Serial OK")
        except:
            print("Error connecting to serial port. Retrying...")
            # Retry every 1 sec
            time.sleep(1)
    print("Gripper serial launched")

def grip_cb(msg):
    global armState
    with lock:
        armState = msg.flip
    print("Callback received, armState set to:", armState)

def serial_writer():
    global armState
    while not rospy.is_shutdown():
        try:
            with lock:
                if armState:
                    ser1.write((armState + '\n').encode())
                    print(f"armState sent: {armState}")
                    # Reset state
                    armState = ""
        except Exception as e:
            print("Serial write failed:", e)
        time.sleep(1)

if __name__ == "__main__":
    setup_serial()
    
    try:
        rospy.init_node("pub2gripper", anonymous=True)
        rospy.Subscriber("gripper_control", GripperControl, grip_cb)
        
        serial_thread = Thread(target=serial_writer)
        serial_thread.start()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("ROS Node terminated.")
        
    except KeyboardInterrupt:
        print("Close Serial Communication")
        
    finally:
        if ser1:
            ser1.close()
        print("Serial port closed.")
