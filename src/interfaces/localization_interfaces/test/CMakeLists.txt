find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(Threads REQUIRED)
include(${PROJECT_SOURCE_DIR}/cmake/Warnings.cmake)
ament_add_gtest(reset_transaction_test reset_transaction_test.cpp TIMEOUT 30)
target_compile_features(reset_transaction_test PRIVATE cxx_std_17)
target_include_directories(reset_transaction_test PRIVATE ../include)
rosidl_get_typesupport_target(test_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(reset_transaction_test "${test_typesupport}" Threads::Threads)
ament_add_pytest_test(wire_roundtrip_test test_wire_roundtrip.py TIMEOUT 30
    APPEND_ENV "PYTHONPATH=${PROJECT_BINARY_DIR}/rosidl_generator_py")
set_tests_properties(reset_transaction_test wire_roundtrip_test PROPERTIES LABELS "unit")
localization_enable_warnings()

# pytest 和动态导入的 launch 文件不在源码树生成字节码。
get_property(package_tests DIRECTORY PROPERTY TESTS)
set_property(TEST ${package_tests} APPEND PROPERTY ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
