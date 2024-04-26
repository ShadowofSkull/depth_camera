# depth_camera

## Setup ROS2 & OpenNI SDK (Windows 11)
1. Download a VM of your choice (Oracle VirtualBox is used)
2. Download Ubuntu 22.04 LTS Jammy Jellyfish iso image (so its compatible with ROS2 Humble)
3. Setup the Ubuntu VM (at least 4GB RAM but 8GB is better, minimum 25GB storage, 4CPU, USB of depth cam need to be added)
4. Once in Ubuntu, download ROS2 Humble following [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (all commands are typed in terminal can press window key and search for it)
5. Open firefox and go to [here](https://www.orbbec.com/developers/openni-sdk/) & scroll to the bottom under `Wrappers for OpenNI SDK` to download OpenNI SDK for ROS2
6. After download the tar zip file, extract it & open the README.md. Follow the instructions in README.md. (`IMPORTANT`: Change every `galactic` word in the file to `humble`)
7. Halfway through you might run into an error with a command starting with `colcon` because its not installed. Follow the steps from [here](https://colcon.readthedocs.io/en/released/user/installation.html#:~:text=In%20the%20context%20of%20the%20ROS%20project) to download it.
8. At the end, you should see a program with black and white image

## Issues
- Color image not showing
  - Need to look into rviz or openNI doc

# Documented by Adam and Zheng 