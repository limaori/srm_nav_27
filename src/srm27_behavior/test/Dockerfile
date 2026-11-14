FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git libyaml-cpp-dev libzmq3-dev libsqlite3-dev \
    python3-colcon-common-extensions python3-pytest \
    ros-humble-nav2-bringup ros-humble-generate-parameter-library ros-humble-serial-driver ros-humble-asio-cmake-module \
    ros-humble-ament-cmake-auto ros-humble-ament-cmake-gtest \
    ros-humble-ament-lint-auto ros-humble-ament-lint-common \
    ros-humble-ament-cmake-clang-format ros-humble-ament-cmake-black \
    && rm -rf /var/lib/apt/lists/*
# Match the production BT ABI; Nav2 Humble independently uses BT.CPP v3.
RUN git clone --depth 1 --branch 4.6.2 https://github.com/BehaviorTree/BehaviorTree.CPP.git /tmp/btcpp \
    && cmake -S /tmp/btcpp -B /tmp/bt-build -DCMAKE_INSTALL_PREFIX=/opt/btcpp \
      -DBTCPP_UNIT_TESTS=OFF -DBTCPP_EXAMPLES=OFF -DBTCPP_TOOLS=OFF \
    && cmake --build /tmp/bt-build -j4 && cmake --install /tmp/bt-build \
    && rm -rf /tmp/btcpp /tmp/bt-build
ENV CMAKE_PREFIX_PATH=/opt/btcpp
ENV LD_LIBRARY_PATH=/opt/btcpp/lib
ENTRYPOINT ["/bin/bash"]
