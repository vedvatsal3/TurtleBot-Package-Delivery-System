# Base image
FROM osrf/ros:jazzy-desktop-full

# Set environment variables for ROS 2
ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    curl \
    lsb-release \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo apt repository and install gz-harmonic
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y gz-harmonic

# Create and set up the ROS 2 workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src

# Clone the necessary repositories
RUN git clone https://github.com/secorolab/create3_sim.git && \
    git clone https://github.com/secorolab/turtlebot4.git && \
    git clone https://github.com/secorolab/turtlebot4_simulator.git

# Install missing ROS dependencies
RUN apt-get update && apt-get install -y ros-jazzy-control-msgs ros-jazzy-irobot-create-nodes ros-jazzy-turtlebot4-desktop ros-jazzy-controller-manager ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-interfaces ros-jazzy-nav2-bringup

# Build the workspace
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Add ROS 2 workspace setup and domain ID to bashrc
RUN echo "export ROS_DOMAIN_ID=2" >> ~/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Source setup on container start
CMD ["/bin/bash"]
