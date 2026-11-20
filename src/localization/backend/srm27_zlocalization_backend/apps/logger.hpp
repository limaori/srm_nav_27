/**
 * @file logger.hpp
 * @brief 将后端日志级别及文本发送到 rclcpp 日志端口。
 */
#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <utility>

#include "backend/logger.hpp"

namespace backend::ros {

/// 把领域层日志转发到 rclcpp 的固定 logger。
class RosLogger : public Logger {
public:
    /// 保存用于后端日志输出的 rclcpp 日志器。
    explicit RosLogger(rclcpp::Logger logger) : logger_(std::move(logger)) {}

    /// 按后端日志级别调用对应的 rclcpp 日志输出。
    void log(LogLevel level, const std::string& message) override {
        switch (level) {
            case LogLevel::Debug:
                RCLCPP_DEBUG_STREAM(logger_, message);
                break;
            case LogLevel::Info:
                RCLCPP_INFO_STREAM(logger_, message);
                break;
            case LogLevel::Warn:
                RCLCPP_WARN_STREAM(logger_, message);
                break;
            case LogLevel::Error:
                RCLCPP_ERROR_STREAM(logger_, message);
                break;
        }
    }

private:
    rclcpp::Logger logger_;
};

}  // namespace backend::ros
