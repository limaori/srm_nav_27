cmake_minimum_required(VERSION 3.8)
project(srm27_localization_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_test REQUIRED)

install(PROGRAMS
  scripts/imu_saturation_guard.py
  scripts/zlio_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_program(SRM27_PYTHON3_EXECUTABLE NAMES python3 REQUIRED)
  ament_add_test(test_zlio_bridge_math
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${SRM27_PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_zlio_bridge_math.py
    OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/test_zlio_bridge_math.txt
    ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:$ENV{PYTHONPATH}
  )
endif()

ament_package()
