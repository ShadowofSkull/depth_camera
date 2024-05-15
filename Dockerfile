# Using the ROS image as a base to build on
FROM ros:noetic-perception-focal

# Ensure all commands are run as root
USER root

# Update package lists, install necessary packages, and clean up
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    openssh-server sudo git vim gedit cmake make && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends libgflags-dev ros-noetic-image-geometry ros-noetic-camera-info-manager \
    ros-noetic-image-transport ros-noetic-image-publisher libgoogle-glog-dev \
    libusb-1.0-0-dev libeigen3-dev ros-noetic-rviz && \
    rm -rf /var/lib/apt/lists/*
        

# Install libuvc
RUN git clone https://github.com/libuvc/libuvc.git && \
    cd libuvc && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && \
    make install && ldconfig && \
    cd ../.. && rm -rf libuvc

# Create/Build workspace and clone the Astra camera package, setup udev rules (have issue when build)
# RUN mkdir -p ~/ros_ws/src && \
#     cd ~/ros_ws/src && \
#     git clone https://github.com/orbbec/ros_astra_camera.git && \
#     source /opt/ros/noetic/setup.bash && cd ~/ros_ws && \
    # catkin_make &&\
    # source ~/ros_ws/devel/setup.bash && roscd astra_camera && \
    # mkdir -p /etc/udev/ && ./scripts/create_udev_rules && \
    # udevadm control --reload && udevadm trigger

# Add the ROS setup.bash to the bashrc
RUN echo "source /opt/ros/noetic/setup.bash\nsource ~/ros_ws/devel/setup.bash" >> ~/.bashrc

# Setup the SSH server config to allow X11 for GUI
RUN sed -i 's/#*X11Forwarding.*/X11Forwarding yes/' /etc/ssh/sshd_config && \
    sed -i 's/#*X11UseLocalhost.*/X11UseLocalhost no/' /etc/ssh/sshd_config

# Add a user for SSH connection
RUN useradd -m user -p $(openssl passwd 1234)

# Create display environment variable with container ip for XLaunch
ENV DISPLAY=host.docker.internal:0.0

# Expose the SSH port
EXPOSE 22

# Start the SSH server and keep the container running
CMD service ssh start && bash
