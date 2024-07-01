#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import time

# Define GPIO pins using BCM numbering for the first sensor
GPIO_TRIGGER1 = 18  # Pin 12
GPIO_ECHO1 = 4      # Pin 7

# Define GPIO pins using BCM numbering for the second sensor
GPIO_TRIGGER2 = 23  # Pin 16
GPIO_ECHO2 = 24     # Pin 18

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_TRIGGER1, GPIO.OUT)
GPIO.setup(GPIO_ECHO1, GPIO.IN)
GPIO.setup(GPIO_TRIGGER2, GPIO.OUT)
GPIO.setup(GPIO_ECHO2, GPIO.IN)

# ROS initialization
rospy.init_node('ultrasonic_sensor_publisher')
pub = rospy.Publisher('InFrontOfSilo', String, queue_size=10)

def distance(trigger_pin, echo_pin):
    # Set Trigger to HIGH
    GPIO.output(trigger_pin, True)

    # Set Trigger after 10µs to LOW
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    # Start time
    StartTime = time.time()
    StopTime = time.time()

    # Save StartTime
    while GPIO.input(echo_pin) == 0:
        StartTime = time.time()

    # Save time of arrival
    while GPIO.input(echo_pin) == 1:
        StopTime = time.time()

    # Time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # Multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

if __name__ == "__main__":
    print("Ultrasonic Sensor launched")
    try:
        while not rospy.is_shutdown():
            dist1 = distance(GPIO_TRIGGER1, GPIO_ECHO1)
            dist2 = distance(GPIO_TRIGGER2, GPIO_ECHO2)
            rospy.loginfo("Measured Distance Sensor 1 = %.1f cm" % dist1)
            rospy.loginfo("Measured Distance Sensor 2 = %.1f cm" % dist2)

            # Determine if sensor 1 is in front of silo
            if dist1 < 5 and dist2 < 5: # 50 cm. Adjust this value after calibrate
                status = "y"  # Yes, sensor 1 is in front of silo
            else:
                status = "n"  # No, sensor 1 is not in front of silo

            # Publish status to ROS topic
            pub.publish(status)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Measurement stopped by User")
    finally:
        GPIO.cleanup()

