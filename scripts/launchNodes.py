import subprocess

try:
    result = subprocess.run(
        ["roslaunch", "astra_camera", "astra.launch"],
        capture_output=True,
        check=True,
        text=True,
    )
    print(result.stdout)
except subprocess.CalledProcessError as e:
    print(f"Command failed with exit code {e.returncode}")
    print(f"Error output: {e.stderr}")
