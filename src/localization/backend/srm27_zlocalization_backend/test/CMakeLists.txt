find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

ament_add_gtest(backend_fusion_test unit/fusion_test.cpp)
ament_add_gtest(backend_measurement_test unit/measurement_test.cpp)
ament_add_gtest(backend_recovery_budget_test unit/recovery_budget_test.cpp)
ament_add_gtest(backend_correction_anchor_test unit/correction_anchor_test.cpp)
ament_add_gtest(backend_lifecycle_test unit/lifecycle_test.cpp unit/frontend_session_test.cpp
    ../apps/frontend_session.cpp)
ament_add_gtest(backend_graph_test integration/graph_test.cpp)
ament_add_gtest(backend_innovation_test integration/innovation_test.cpp)
ament_add_gtest(backend_uwb_factor_test integration/uwb_factor_test.cpp)
ament_add_gtest(backend_relocalization_test integration/relocalization_test.cpp)
ament_add_gtest(backend_localization_test
    integration/localization_test.cpp
    integration/localization_reset_test.cpp
    integration/localization_discontinuity_test.cpp
    integration/localization_continuity_test.cpp
    integration/pose_output_test.cpp)
ament_add_gtest(backend_descriptor_test integration/descriptor_test.cpp)
ament_add_gtest(backend_calibration_test tools/body_calibration_test.cpp
    ../tools/body_calibration.cpp)
ament_add_gtest(backend_storage_test integration/storage_test.cpp)
target_link_options(backend_storage_test PRIVATE "LINKER:--wrap=fsync")
ament_add_gtest(backend_cleanup_test tools/cleanup_test.cpp)
foreach(target backend_fusion_test backend_measurement_test backend_recovery_budget_test
               backend_correction_anchor_test
               backend_lifecycle_test backend_graph_test
               backend_storage_test backend_cleanup_test backend_innovation_test backend_uwb_factor_test
               backend_relocalization_test backend_localization_test backend_descriptor_test
               backend_calibration_test)
    target_link_libraries(${target} srm27_zlocalization_backend_core)
    target_include_directories(${target} PRIVATE support)
    set_tests_properties(${target} PROPERTIES TIMEOUT 30 LABELS "unit")
endforeach()
set_tests_properties(backend_localization_test backend_relocalization_test PROPERTIES TIMEOUT 120)
target_include_directories(backend_lifecycle_test PRIVATE .. ../src)
target_link_libraries(backend_cleanup_test srm27_zlocalization_backend_map_cleanup)
target_include_directories(backend_calibration_test PRIVATE ../tools)
set_tests_properties(backend_calibration_test PROPERTIES LABELS "tools;integration")
set_tests_properties(backend_graph_test backend_storage_test backend_innovation_test
    backend_uwb_factor_test backend_relocalization_test backend_localization_test
    backend_descriptor_test PROPERTIES LABELS "integration")
set_tests_properties(backend_cleanup_test PROPERTIES LABELS "geometry;tools")
ament_add_pytest_test(backend_tools_test tools/test_map_tools.py TIMEOUT 90
    ENV "BACKEND_BINARY_DIR=$<TARGET_FILE_DIR:srm27_zlocalization_backend_clean_map_bundle>")
ament_add_pytest_test(backend_launch_config_test ros/test_launch_config.py TIMEOUT 30)
set_tests_properties(backend_tools_test PROPERTIES LABELS "tools;integration")
set_tests_properties(backend_launch_config_test PROPERTIES LABELS "unit")
add_launch_test(ros/test_calibration_launch.py TARGET backend_calibration_launch_test
    TIMEOUT 60 LABELS "ros")
add_launch_test(ros/test_map_service.py TARGET backend_map_service_test TIMEOUT 90
    ARGS "node:=$<TARGET_FILE:srm27_zlocalization_backend_node>" LABELS "ros")

# 大数据验收需显式启用，指定数据文件缺失时配置失败。
ament_add_pytest_test(backend_evaluation_metrics_test evaluation/test_metrics.py TIMEOUT 30)
set_tests_properties(backend_evaluation_metrics_test PROPERTIES LABELS "unit")
set(LOCALIZATION_TEST_DATA_DIR "" CACHE PATH "公开 stairs-native 标签快照目录")
if(LOCALIZATION_TEST_DATA_DIR)
    foreach(file geometry.bin labels.bin manifest.json)
        if(NOT EXISTS "${LOCALIZATION_TEST_DATA_DIR}/${file}")
            message(FATAL_ERROR "Missing evaluation input: ${LOCALIZATION_TEST_DATA_DIR}/${file}")
        endif()
    endforeach()
    add_executable(cleanup_dataset_driver evaluation/cleanup_dataset.cpp)
    target_link_libraries(cleanup_dataset_driver srm27_zlocalization_backend_map_cleanup)
    ament_add_pytest_test(backend_labeled_cleanup_test evaluation/test_labeled_cleanup.py
        TIMEOUT 180 ENV "LOCALIZATION_TEST_DATA_DIR=${LOCALIZATION_TEST_DATA_DIR}"
        "CLEANUP_DATASET_DRIVER=$<TARGET_FILE:cleanup_dataset_driver>")
    set_tests_properties(backend_labeled_cleanup_test PROPERTIES LABELS "evaluation" RUN_SERIAL TRUE)
endif()

localization_enable_warnings()

set_tests_properties(backend_map_service_test backend_calibration_launch_test PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")

# pytest 和动态导入的 launch 文件不在源码树生成字节码。
get_property(package_tests DIRECTORY PROPERTY TESTS)
set_property(TEST ${package_tests} APPEND PROPERTY ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
