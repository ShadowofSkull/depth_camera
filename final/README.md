# Open day
## How to remotely control R2
1. Turn on Jetson
1. Download Anydesk
2. Connect to Adam's computer (ask Adam about it)
3. Open Adam's vscode and click the bottom left blue icon
4. Make sure Adam's computer is connected to Adam's Hotspot
6. Enter `Connect to Host`
7. Then type robot@172.20.10.5
8. Select Linux and enter
9. Enter the password `Sunway123`
10. Go to the directory
    ```shell
    cd ~/Desktop/depth_camera/final
    ```
11. Run in terminal
    ```shell
    python3 teleop.py
    ```
12. Make sure motor driver and high torque power is off before running this to be safe (anything other than switch label `1` is off)
    ```shell
    python3 simpleRos2Serial.py
    ```
13. Focus the terminal that ran `teleop.py` and press the correct keys to control the robot
14. Done that's it
