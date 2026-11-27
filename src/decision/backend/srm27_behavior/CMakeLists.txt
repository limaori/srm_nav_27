cmake_minimum_required(VERSION 3.16)
project(srm27_behavior)

# 决策后端：BehaviorTree 执行服务端 + 客户端 + 裁判输入。
# BT 插件在 srm27_behavior_plugins（share/<pkg>/bt_plugins），共享状态在
# srm27_behavior_core；本包只保留运行期与装配资产（决策树/参数/launch/验证脚本）。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_executable(srm27_behavior_server src/srm27_behavior_server.cpp src/referee_inputs.cpp)

ament_auto_add_library(srm27_behavior_action_client SHARED
  src/srm27_behavior_client.cpp
)

rclcpp_components_register_node(srm27_behavior_action_client
  PLUGIN srm27_behavior::SentryBehaviorClient
  EXECUTABLE srm27_behavior_client
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  # 核心/插件的 gtest 分别在 srm27_behavior_core / srm27_behavior_plugins；
  # 本包只保留 lint 与 test/ 下的 ROS/Nav2 集成回归（由 validate_decision.sh 调用）。
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  decision_trees
  params
  launch
)
