# Pre-built ROS2 Jazzy + Gazebo Harmonic stack
FROM althack/ros2:jazzy-gazebo

# Install UR robot packages and essential ROS2 tools (as root)
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    liburdfdom-tools \
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
    python3-colcon-common-extensions \
    && sudo rm -rf /var/lib/apt/lists/*

# Switch to ros user for workspace setup
USER ros

# Set working directory to workspace
WORKDIR /home/ros/ros2_ws

# Source ROS2 base and workspace overlay
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ros/.bashrc && \
    echo "source /home/ros/ros2_ws/install/setup.bash 2>/dev/null || true" >> /home/ros/.bashrc