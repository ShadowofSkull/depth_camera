# Using the ros img as base to build on 
FROM ros:noetic-perception-focal
# Ensure all commands are run as root
USER root

# Splitting installation since its build in layers less time is needed to rebuild the image if changes are made
# Install the necessary basic packages
RUN \
    apt update && \
    apt install -y openssh-server sudo git vim gedit cmake make

# Install OpenNI ros dependencies
RUN \
    apt update && \
    apt install -y libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev

# Install libuvc
RUN \
    git clone https://github.com/libuvc/libuvc.git && cd libuvc && \
    mkdir build && cd build && \
    cmake .. && make -j4 && \
    sudo make install && sudo ldconfig

# Create/Build workspace and clone the astra camera package
RUN \
    mkdir -p ~/ros_ws/src && \
    cd ~/ros_ws/src && \
    git clone https://github.com/orbbec/ros_astra_camera.git && \
    cd ~/ros_ws && catkin_make

# Install udev rules (driver stuff)
RUN \
    cd ~/ros_ws && source ./devel/setup.bash && \
    roscd astra_camera && ./scripts/create_udev_rules && \
    sudo udevadm control --reload && sudo  udevadm trigger

# Install additional packages (add any extra packages you need here)
RUN \
    apt update && \
    apt install -y ros-noetic-rviz

# Add the ros setup.bash to the bashrc
RUN echo -e "source /opt/ros/noetic/setup.bash\nsource ~/ros_ws/devel/setup.bash" >> ~/.bashrc

# Setup the ssh server config to allow x11 for gui
RUN sed -i 's/#*X11Forwarding.*/X11Forwarding yes/' /etc/ssh/sshd_config
RUN sed -i 's/#*X11UseLocalhost.*/X11UseLocalhost no/' /etc/ssh/sshd_config

# Add a user for ssh connection
RUN useradd -m user -p $(openssl passwd 1234)

# Create display environment variable for xlaunch
ENV DISPLAY=host.docker.internal:0.0

# Expose the ssh port
EXPOSE 22

# Start the ssh server and keep the container running
CMD service ssh start ; bash