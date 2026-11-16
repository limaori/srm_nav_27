cmake_minimum_required(VERSION 3.8)
project(srm27_nav_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_test REQUIRED)

install(PROGRAMS
  scripts/gimbal_joint_state_relay.py
  scripts/gimbal_spin_gate.py
  scripts/simulation_ground_truth_odometry.py
  scripts/srm_minco_real_preflight.py
  scripts/srm_regression_metadata.py
  scripts/srm_speed_limits_check.py
  scripts/srm_velocity_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/srm_nav_pose_math.py
  scripts/srm_robot_description.py
  scripts/srm_speed_limits.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_program(SRM27_PYTHON3_EXECUTABLE NAMES python3 REQUIRED)
  ament_add_test(test_speed_limits
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${SRM27_PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_speed_limits.py
    OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/test_speed_limits.txt
    ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:$ENV{PYTHONPATH}
  )
endif()

ament_package()
