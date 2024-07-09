# depth_camera

## Table of Contents
  - [Setup ROS2 \& OpenNI SDK (Windows 11) *No longer using*](#setup-ros2--openni-sdk-windows-11-no-longer-using)
  - [Setup ROS \& OpenNI SDK (WIN 11) *Preferred*](#setup-ros--openni-sdk-win-11-preferred)
    - [IMPORTANT NOTES](#important-notes)
  - [QuickStart](#quickstart)
  - [How to take picture](#how-to-take-picture)
  - [Getting Detect.py to work](#getting-detectpy-to-work)
  - [Integration of ROS depth camera \& OpenCV](#integration-of-ros-depth-camera--opencv)
  - [Jetson Orin Nano](#jetson-orin-nano)
    - [Reinstall jetpack (The OS)](#reinstall-jetpack-the-os)
  - [Python virtual env](#python-virtual-env)
  - [Docker (able to use ros but udev which handles usb does not work)](#docker-able-to-use-ros-but-udev-which-handles-usb-does-not-work)
    - [Installation](#installation)
    - [QuickStart](#quickstart-1)
    - [Extra commands](#extra-commands)
    - [Building image from scratch](#building-image-from-scratch)
  - [Issues](#issues)
  - [Future Plans](#future-plans)
  - [Author](#author)


## Setup ROS2 & OpenNI SDK (Windows 11) *No longer using*
1. Download a VM of your choice (Oracle VirtualBox is used)
2. Download Ubuntu 22.04 LTS Jammy Jellyfish iso image (so its compatible with ROS2 Humble)
3. Setup the Ubuntu VM (at least 4GB RAM but 8GB is better, minimum 25GB storage, 4CPU, USB of depth cam need to be added)
4. Once in Ubuntu, download ROS2 Humble following [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (all commands are typed in terminal can press window key and search for it)
5. Open firefox and go to [here](https://www.orbbec.com/developers/openni-sdk/) & scroll to the bottom under `Wrappers for OpenNI SDK` to download OpenNI SDK for ROS2
6. After download the tar zip file, extract it & open the README.md. Follow the instructions in README.md. (`IMPORTANT`: Change every `galactic` word in the file to `humble`)
7. Halfway through you might run into an error with a command starting with `colcon` because its not installed. Follow the steps from [here](https://colcon.readthedocs.io/en/released/user/installation.html#:~:text=In%20the%20context%20of%20the%20ROS%20project) to download it.
8. At the end, you should see a program with black and white image

## Setup ROS & OpenNI SDK (WIN 11) *Preferred*
1. Download VM
2. Download Ubuntu 20
3. Setup VM (Remember to add usb to vm)
4. Follow steps from here to setup ROS [here](https://wiki.ros.org/noetic/Installation/Ubuntu)
5. Follow steps from here to setup OpenNI [here](https://github.com/orbbec/ros_astra_camera)

### IMPORTANT NOTES
1. Depth camera works on zheng laptop left usb port(1)
2. USB controller of VM is set to usb3.0 (has smoother video stream)
3. This might need to be rerun everytime for it to detect the camera
     ```shell
        roscd astra_camera
        ./scripts/create_udev_rules
        sudo udevadm control --reload && sudo udevadm trigger
     ```

## QuickStart
1. Run `roslaunch astra_camera astra.launch`
2. Run `python3 ultrasonic_sensor.py`
3. Run `python3 detect_silo.py`
4. Run `python3 ros2serial_gripper.py` and `python3 ros2serial_gripper.py`
5. Turn on motor and gripper power

## How to take picture
1. Use below command
```shell
   rosservice call /camera/save_images "{}"
```
2. Images are saved in `~/.ros/image` and are only available when the sensor is on.
3. xdg-open <filename>
   - Replace <filename> with filename

## Getting Detect.py to work
It uses custom made msg so need to follow this: 
1. Create Coords.msg and CoordsMatrix.msg in `/home/ros/ros_ws/src/ros_astra_camera/msg`
2. Copy contents of both files from `msg` folder here into your local machine 
3. Follow the steps here to build the custom msg, [link](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Common_step_for_msg_and_srv)

## Integration of ROS depth camera & OpenCV
1. Installation not needed since all necessary dependencies are downloaded in `ros_astra_camera` package
2. Get into `Scripts` folder in `ros_astra_camera`
```shell
   cd ~/ros_ws/src/ros_astra_camera/scripts
```
3. Create a python file
```shell
   touch detect.py
```
4. Paste code from [detect.py](https://github.com/ShadowofSkull/depth_camera/blob/main/detect.py) file into your file in VM
```shell
   gedit detect.py
```
5. Save the file and give it execution permission
```shell
   chmod +x detect.py
```
6. Be sure to source these files on all the terminal before running Step 7-8
```shell
   cd ~/ros_ws
   source /opt/ros/noetic/setup.bash
   source ~/ros_ws/devel/setup.bash
```
   - To not do this for all terminal
      ```shell
         gedit ~/.bashrc
      ```
   - Copy below into the very bottom of the file and save 
      ```shell
      source /opt/ros/noetic/setup.bash
      source ~/ros_ws/devel/setup.bash
      ```   
7. Run the depth camera
```shell
source ./devel/setup.bash 
roslaunch astra_camera astra.launch
``` 
8. Open a terminal and run
```shell
   roscore
```
9. Open another terminal and run
```shell
   rosrun ros_astra_camera detect.py
```

## Jetson Orin Nano
### Reinstall jetpack (The OS)
1. Using a linux machine (or maybe VM not tested), download Nvidia SDK Manager
2. Use Jumper Wire to connect Ground and Force Reset Pin on Jetson Orin Nano
3. Connect type C from Jetson to linux machine
4. Provide Jetson Power to boot in (safe mode/reset mode?)
5. Install with SDK Manager

## Python virtual env
Purpose is to create an isolated environment for python packages so there is no risk of dependencies conflict with other similar packages
1. Install virtual env(venv) package
```shell
pip install virtualenv
```
2. Create venv environment
```shell
python -m venv .venv
```
3. Activate env
   1. Windows
   ```shell
   .venv\Scripts\activate
   ```
   2. Linux/Mac
   ```shell
   source .venv/bin/activate
   ```
4. Install saved dependencies
```shell
pip install -r requirements.txt
```
5. To leave env
```shell
deactivate
```

- Once env is activate can use pip install as usual to install new dependencies
- To check dependencies use
```shell
pip list
```
- After adding new dependencies execute below to save dependencies
```shell
pip freeze > requirements.txt
```


## Docker (able to use ros but udev which handles usb does not work)
### Installation
1. Download docker desktop from the [site](https://www.docker.com/products/docker-desktop/)
2. Follow instructions here
   1. Mac: https://docs.docker.com/desktop/install/mac-install/
   2. Windows: https://docs.docker.com/desktop/install/windows-install/
3. vcxsrv X server is required to run gui through x11 forwarding in ssh, [install](https://sourceforge.net/projects/vcxsrv/)
4. Putty (optional)

### QuickStart
**Replace anything in <>**
1. Pull the docker image from dockerhub
```shell
docker pull shadowofskull/depth_cam:latest
```
2. Run a container from the image
```shell
docker run -it -p 22:22 --name ros-x11-ex shadowofskull/depth_cam:latest
```

### Extra commands
- To check container id
```shell
docker ps
```
- To add extra terminal
```shell
docker exec -it <container id> bash
```
- To rerun already created container use
```shell
docker start <container id> 
```

### Building image from scratch
1. Get into the folder Dockerfile exists
2. Run
```shell
docker build -t <image name> . 
```
   - Add --no-cache if having issue building

## Issues
1. Q:Color image not showing
   1. A: Issue lies in ros2 and openni sdk for ros2, use back ros instead
2. ROS2 openni seems to not support uvc camera color stream which our depth cam seems to be
   1. check if can activate uvc like in ros1 or not

## Future Plans
1. Find out how to integrate object detection with the depth camera/rviz

  
## Author
[Adam](https://github.com/Jung028) and [Zheng](https://github.com/ShadowofSkull/) 
