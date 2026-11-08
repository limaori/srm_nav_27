find_package(ament_cmake_gtest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

ament_add_gtest(frontend_config_test unit/config_test.cpp)
target_compile_definitions(frontend_config_test PRIVATE
    ZLIO_TEST_CONFIG="${PROJECT_SOURCE_DIR}/config/lio.yaml")
ament_add_gtest(frontend_state_test unit/state_test.cpp)
ament_add_gtest(frontend_input_test unit/input_test.cpp)
ament_add_gtest(frontend_estimation_test integration/estimation_test.cpp)
ament_add_gtest(frontend_initialization_test integration/initialization_test.cpp)
target_compile_definitions(frontend_initialization_test PRIVATE
    ZLIO_TEST_CONFIG="${PROJECT_SOURCE_DIR}/config/lio.yaml")
ament_add_gtest(frontend_map_test integration/map_test.cpp)
target_include_directories(frontend_map_test PRIVATE ../src)
ament_add_gtest(frontend_adapter_test ros/adapter_test.cpp
    ../apps/input.cpp ../apps/ros.cpp)
foreach(target frontend_config_test frontend_state_test frontend_input_test frontend_estimation_test
               frontend_map_test frontend_initialization_test frontend_adapter_test)
    target_link_libraries(${target} zlio Threads::Threads)
    set_tests_properties(${target} PROPERTIES TIMEOUT 30 LABELS "unit")
endforeach()
target_include_directories(frontend_adapter_test PRIVATE .. ../src)
ament_target_dependencies(frontend_adapter_test sensor_msgs nav_msgs)
set_tests_properties(frontend_estimation_test frontend_map_test frontend_initialization_test
    PROPERTIES LABELS "integration")
# 七个完整初始化场景在无优化插桩构建中需要更长的求解时间。
set_tests_properties(frontend_initialization_test PROPERTIES TIMEOUT 180)
add_launch_test(ros/test_lifecycle.py TARGET frontend_lifecycle_test TIMEOUT 60
    ARGS "config:=${PROJECT_SOURCE_DIR}/config/lio.yaml"
    LABELS "ros")
# 两种点云格式的协调恢复与独立前端自动恢复分别执行完整初始化。
add_launch_test(ros/test_sensor_pipeline.py TARGET frontend_sensor_pipeline_test TIMEOUT 240
    ARGS "node:=$<TARGET_FILE:zlio_node>" "config:=${PROJECT_SOURCE_DIR}/config/lio.yaml"
    LABELS "ros;integration")
add_launch_test(ros/test_visualization.py TARGET frontend_visualization_test TIMEOUT 60
    ARGS "node:=$<TARGET_FILE:zlio_visualizer>" LABELS "ros")
localization_enable_warnings()

set_tests_properties(frontend_lifecycle_test frontend_visualization_test
    frontend_sensor_pipeline_test PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")

# pytest 和动态导入的 launch 文件不在源码树生成字节码。
get_property(package_tests DIRECTORY PROPERTY TESTS)
set_property(TEST ${package_tests} APPEND PROPERTY ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
