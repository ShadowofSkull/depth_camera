import subprocess

try:
    p1 = subprocess.Popen(["roslaunch", "astra_camera", "astra.launch"])
    p2 = subprocess.Popen(["python3", "detect_silo.py"])
    p3 = subprocess.Popen(["python3", "ultrasonic_sensor.py"])
    p4 = subprocess.Popen(["python3", "ros2serial_motor.py"])
    p5 = subprocess.Popen(["python3", "ros2serial_gripper.py"])
except Exception as e:
    print(f"err: {e}")

p1.wait()
p2.wait()
p3.wait()
p4.wait()
p5.wait()

stop = input("enter y")
if stop == 'y':
    p1.kill()
    p2.kill()
    p3.kill()
    p4.kill()
    p5.kill()

