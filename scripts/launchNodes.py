import subprocess

try:
    result = subprocess.run(
        ["roslaunch", "astra_camera", "astra.launch"],
        capture_output=True,
        check=True,
        text=True,
    )
    print("execute roslaunch")
    print(result.stdout)
except subprocess.CalledProcessError as e:
    print(f"Command failed with exit code {e.returncode}")
    print(f"Error output: {e.stderr}")

try: 
    result = subprocess.run(
        ["python3", "detect_silo.py"],
        capture_output=True,
        check=True,
        text=True,
        cwd="/home/robot/ros_ws/src/ros_astra_camera/scripts"
    )
    print("execute python3")
    print(result.stdout)
except subprocess.CalledProcessError as e:
    print(f"Command failed with exit code {e.returncode}")
    print(f"Error output: {e.stderr}")