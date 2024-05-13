# Using the ros img as base to build on 
FROM ros:noetic-perception-focal

# Ensure all commands are run as root
USER root

# Install necessary basic packages and OpenNI ros dependencies in a single step
RUN apt update && \
    apt install -y \
    openssh-server sudo git vim gedit cmake make \
    libgflags-dev ros-noetic-image-geometry ros-noetic-camera-info-manager \
    ros-noetic-image-transport ros-noetic-image-publisher libgoogle-glog-dev \
    libusb-1.0-0-dev libeigen3-dev && \
    apt clean && rm -rf /var/lib/apt/lists/*

# Install libuvc
RUN git clone https://github.com/libuvc/libuvc.git && \
    cd libuvc && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install && \
    ldconfig && \
    cd ../.. && \
    rm -rf libuvc

# Create/Build workspace and clone the astra camera package
RUN mkdir -p ~/ros_ws/src && \
    cd ~/ros_ws/src && \
    git clone https://github.com/orbbec/ros_astra_camera.git && \
    cd ~/ros_ws && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make && \
    rm -rf /var/lib/apt/lists/*

# Install additional packages
RUN apt update && \
    apt install -y ros-noetic-rviz && \
    apt clean && rm -rf /var/lib/apt/lists/*

# Add the ros setup.bash to the bashrc
RUN echo -e "source /opt/ros/noetic/setup.bash\nsource ~/ros_ws/devel/setup.bash" >> ~/.bashrc

# Setup the ssh server config to allow x11 for gui
RUN sed -i 's/#*X11Forwarding.*/X11Forwarding yes/' /etc/ssh/sshd_config && \
    sed -i 's/#*X11UseLocalhost.*/X11UseLocalhost no/' /etc/ssh/sshd_config

# Add a user for ssh connection
RUN useradd -m user -p $(openssl passwd 1234)

# Create display environment variable for xlaunch
ENV DISPLAY=host.docker.internal:0.0

# Expose the ssh port
EXPOSE 22

# Start the ssh server and keep the container running
CMD service ssh start && bash