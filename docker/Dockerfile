# ROS 2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox + TurtleBot3, headless-capable.
#
# Everything here is an off-the-shelf apt package (confirmed available for both
# arm64 and amd64 on ros:jazzy-ros-base-noble) - no source builds, no custom
# robot model. Built for both linux/arm64 (native on Apple Silicon, local dev)
# and linux/amd64 (native on GitHub Actions ubuntu-latest, CI) - never run this
# under QEMU emulation.
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Simulation
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces \
    # Robot
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-simulations \
    ros-jazzy-turtlebot3-navigation2 \
    ros-jazzy-turtlebot3-bringup \
    # SLAM + navigation
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    # Fault injection (standard tool, not custom code)
    ros-jazzy-topic-tools \
    # Build tooling for our own package
    python3-colcon-common-extensions \
    python3-pip \
    python3-yaml \
    python3-pytest \
    ros-dev-tools \
    # Analysis / demo-rendering tooling (used by milestones M1+)
    python3-matplotlib \
    python3-pandas \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir --break-system-packages ruff rosbags

ENV TURTLEBOT3_MODEL=burger
WORKDIR /workspace

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
