cmake_minimum_required(VERSION 3.8)
project(srm27_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  # 只保留本分支（小陀螺 / fakeyaw 链路）自己的回归测试。
  # 其余测试已随 main 的 b2d6fc2 一并删除，因此这里不再跑 ament_lint_auto。
  # 用普通 Python 入口而不是 pytest：本机 pytest11 插件与 launch_testing/
  # 较新 pluggy 不兼容，会在收集阶段直接失败。
  find_program(SRM27_PYTHON3_EXECUTABLE NAMES python3 REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  foreach(test_name test_fakeyaw_launch_wiring test_fakeyaw_real_wiring
                    test_cli_parity test_lio_frontend_wiring
                    test_gimbal_tree_direction test_frame_contract)
    ament_add_test(${test_name}
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND ${SRM27_PYTHON3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.py
      OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/${test_name}.txt
      ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../srm27_nav_tools/scripts:${CMAKE_CURRENT_SOURCE_DIR}/../../localization/backend/srm27_localization_adapter/scripts:$ENV{PYTHONPATH}
    )
  endforeach()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  meshes
  pcd
  config
  nav2_bt
  rviz
)
