FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake libeigen3-dev python3-colcon-common-extensions \
    python3-pytest python3-scipy clang-format \
    ros-humble-nav2-bringup ros-humble-ros-gz \
    ros-humble-ament-cmake-auto ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
    ros-humble-ament-lint-auto ros-humble-ament-lint-common \
    ros-humble-ament-cmake-clang-format ros-humble-ament-cmake-clang-tidy \
    ros-humble-ament-cmake-black libignition-gazebo6-dev \
    && rm -rf /var/lib/apt/lists/*
ENTRYPOINT ["/bin/bash"]
