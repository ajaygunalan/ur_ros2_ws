FROM althack/ros2:jazzy-gazebo

ARG USERNAME=ros
USER $USERNAME

# Install all ROS dependencies, tools, and debugging utilities in a single layer
RUN sudo add-apt-repository universe && \
    sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y --no-install-recommends \
    # Core tools
    liburdfdom-tools \
    python3-colcon-common-extensions \
    # ROS packages
    ros-jazzy-ur-description \
    ros-jazzy-ur-moveit-config \
    ros-jazzy-ur-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-velocity-controllers \
    ros-jazzy-position-controllers \
    ros-jazzy-ros2-control \
    # Debugging and network tools
    x11-apps \
    iputils-ping \
    net-tools \
    dnsutils \
    iproute2 \
    netcat-openbsd \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/$USERNAME/ros2_ws

# Setup bashrc with ROS environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc && \
    echo 'export DISPLAY=${DISPLAY:-$(ip route | grep default | awk "{print \$3}"):1}' >> ~/.bashrc