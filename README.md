# depth_camera

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
   - This needs to be rerun everytime for it to detect the camera
     ```shell
        roscd astra_camera
        ./scripts/create_udev_rules
        sudo udevadm control --reload && sudo  udevadm trigger
     ```

## Issues
1. Q:Color image not showing
   1. A: Issue lies in ros2 and openni sdk for ros2, use back ros instead

  

## Author
[Adam](https://github.com/Jung028) and [Zheng](https://github.com/ShadowofSkull/) 
