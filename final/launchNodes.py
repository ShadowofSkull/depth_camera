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
# try:
#     result = subprocess.run(
#         ["roslaunch", "astra_camera", "astra.launch"],
#         capture_output=True,
#         check=True,
#         text=True,
#     )
#     print("execute roslaunch")
#     print(result.stdout)
# except subprocess.CalledProcessError as e:
#     print(f"Command failed with exit code {e.returncode}")
#     print(f"Error output: {e.stderr}")

# try:
#     result = subprocess.run(
#         ["python3", "detect_silo.py"],
#         capture_output=True,
#         check=True,
#         text=True,
#     )
#     print("execute python3")
#     print(result.stdout)
# except subprocess.CalledProcessError as e:
#     print(f"Command failed with exit code {e.returncode}")
#     print(f"Error output: {e.stderr}")

# try:
#     result = subprocess.run(
#         ["python3", "ultrasonic_sensor.py"],
#         capture_output=True,
#         check=True,
#         text=True,
#     )
#     print("execute python3")
#     print(result.stdout)
# except subprocess.CalledProcessError as e:
#     print(f"Command failed with exit code {e.returncode}")
#     print(f"Error output: {e.stderr}")

# try:
#     result = subprocess.run(
#         ["python3", "ros2serial_motor.py"],
#         capture_output=True,
#         check=True,
#         text=True,
#     )
#     print("execute python3")
#     print(result.stdout)
# except subprocess.CalledProcessError as e:
#     print(f"Command failed with exit code {e.returncode}")
#     print(f"Error output: {e.stderr}")

# try:
#     result = subprocess.run(
#         ["python3", "ros2serial_gripper.py"],
#         capture_output=True,
#         check=True,
#         text=True,
#     )
#     print("execute python3")
#     print(result.stdout)
# except subprocess.CalledProcessError as e:
#     print(f"Command failed with exit code {e.returncode}")
#     print(f"Error output: {e.stderr}")
