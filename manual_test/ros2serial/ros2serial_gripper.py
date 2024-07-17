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
        print("in loop")
        try:
            with lock:
                if armState:
                    print("have armstate")
                    ser1.write((armState + '\n').encode("utf-8"))
                    print(f"armState sent: {armState}")
                    if ser1.in_waiting > 0:
                        line = ser1.readline().decode("utf-8").rstrip()  # Decode the received data
                        print(f"Received: {line}")  # Debug print
                    # Reset state
                    armState = ""
                print("in lock")
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
