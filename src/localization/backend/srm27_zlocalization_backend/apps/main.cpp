/**
 * @file main.cpp
 * @brief 装配后端 ROS 执行器并管理进程退出。
 */
#include "apps/logger.hpp"
#include "apps/node.hpp"
#include "backend/logger.hpp"

/// 初始化 ROS、日志器和并发执行器，并在退出时释放进程资源。
int main(int argc, char** argv) {
    int exit_code = 0;
    std::unique_ptr<backend::ros::RosLogger> ros_logger;
    try {
        rclcpp::init(argc, argv);
        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);
        auto node = std::make_shared<backend::ros::LocalizationNode>(options);
        // 领域层日志必须挂在节点 logger 上，否则 rclcpp 不会把它发到 /rosout：
        // 现场只能从 launch 的 stdout 看到 `global correction committed` /
        // `loop scan-to-map accepted`，用 /rosout 做诊断的工具（debug_tools/
        // plot_backend_correction.py）会一条都收不到。节点构造期间的启动日志仍在
        // 安装 logger 之前，只走 stdout；运行期修正/回环日志从下面这行起进 /rosout。
        ros_logger = std::make_unique<backend::ros::RosLogger>(node->get_logger());
        backend::setDefaultLogger(ros_logger.get());
        // 三个执行线程并行处理同步输入、高频输出和扫描匹配维护；
        // 控制器互斥锁与前端 epoch 约束状态提交。
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
        executor.add_node(node);
        executor.spin();
        executor.remove_node(node);
        node->finish();
        exit_code = node->failed() ? 1 : 0;
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("backend"), "backend failed: %s", error.what());
        exit_code = 1;
    }

    backend::setDefaultLogger(nullptr);
    if (rclcpp::ok())
        rclcpp::shutdown();
    return exit_code;
}
