/**
 * @file main.cpp
 * @brief 启动单线程 ROS 执行器并处理前端进程退出。
 */
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "node.hpp"

/// 启动 ROS 执行器并管理节点生命周期。
int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<zlio::ros::OdometryNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        executor.remove_node(node);
        node.reset();
    } catch (const std::exception& error) {
        std::cerr << "ROS node failed: " << error.what() << '\n';
        if (rclcpp::ok())
            rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
