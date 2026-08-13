# Base Image: ROS 2 Humble Desktop on Ubuntu 22.04
FROM osrf/ros:humble-desktop

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies for OpenGL, GLFW, X11, Mesa, and USB devices
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    # OpenGL / GLFW / Render libraries for MuJoCo
    libgl1-mesa-dev \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libglew-dev \
    libglfw3 \
    libglfw3-dev \
    libosmesa6-dev \
    mesa-utils \
    # X11 & GUI utilities
    x11-apps \
    libxcursor1 \
    libxinerama1 \
    libxrandr2 \
    libxi6 \
    # USB Joystick tools
    joystick \
    evtest \
    # ROS 2 packages
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-realtime-tools \
    ros-humble-xacro \
    ros-humble-mujoco-ros2-control \
    ros-humble-moveit-servo \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*

# Install MuJoCo Python bindings & common robotics libraries
RUN pip3 install --no-cache-dir \
    mujoco \
    numpy \
    scipy \
    transforms3d

# Set Environment Variables for Rendering & ROS
ENV MUJOCO_GL=glfw
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display

# Setup workspace
WORKDIR /ros2_ws
COPY ./src /ros2_ws/src

# Some vendored repos (e.g. libfranka -> libfranka-common) use git submodules.
# `vcs import` does not initialize these, so do it explicitly for any repo
# under src/ that declares one; harmless no-op otherwise.
RUN git config --global --add safe.directory '*' && \
    find /ros2_ws/src -maxdepth 2 -name ".gitmodules" -execdir git submodule update --init --recursive \;

# Initialize rosdep (to catch any missed dependencies from src)
RUN rosdep init || true
RUN rosdep update
RUN apt-get update && rosdep install -y --from-paths src --ignore-src --rosdistro humble && rm -rf /var/lib/apt/lists/*

# Disable BUILD_TESTING workspace-wide, as a colcon default so it applies
# to *any* future `colcon build` too (e.g. a manual rebuild after editing
# a file), not just this one. Vendored deps (e.g. franka_hardware's gtest
# binaries) link directly against libfranka.so, which needs
# libpinocchio_*.so at link time. `ld` doesn't search LD_LIBRARY_PATH
# (only the runtime loader does), so even with ROS setup.bash sourced,
# those test binaries fail with "undefined reference" to pinocchio
# symbols. We don't run any vendored package's own test suite here, so
# skip compiling them entirely rather than patching link paths.
RUN mkdir -p /root/.colcon && \
    printf 'build:\n  cmake-args:\n    - -DBUILD_TESTING=OFF\n' > /root/.colcon/defaults.yaml

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Setup entrypoint script
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Ensure ROS is sourced automatically for interactive bash sessions
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
