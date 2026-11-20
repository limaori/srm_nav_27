/**
 * @file logger.hpp
 * @brief 定义不依赖 ROS 的日志输出接口及默认输出安装边界。
 */
#pragma once

#include <sstream>
#include <string>

namespace backend {

enum class LogLevel { Debug, Info, Warn, Error };

/// 日志 sink 抽象。实现只需把一条已格式化的消息投递到具体后端。
class Logger {
public:
    /// 通过日志接口释放具体输出实现。
    virtual ~Logger() = default;
    /// 将一条已格式化的消息提交到对应级别的日志输出。
    virtual void log(LogLevel level, const std::string& message) = 0;
};

/// 进程级默认 sink。未安装后端时返回控制台后备实现。
Logger& defaultLogger();

/// 安装默认 sink（adapter 在启动时调用）。传 nullptr 恢复控制台后备。
void setDefaultLogger(Logger* logger);

}  // namespace backend

/// 流式日志宏。用法：BACKEND_WARN_STREAM("radius=" << r << " rejected")。
/// 变参展开以容忍消息表达式中的逗号（如嵌套函数调用）。
#define BACKEND_LOG_STREAM(level, ...)                                   \
    do {                                                                 \
        std::ostringstream _backend_log_oss;                             \
        _backend_log_oss << __VA_ARGS__;                                 \
        ::backend::defaultLogger().log((level), _backend_log_oss.str()); \
    } while (0)

#define BACKEND_DEBUG_STREAM(...) BACKEND_LOG_STREAM(::backend::LogLevel::Debug, __VA_ARGS__)
#define BACKEND_INFO_STREAM(...) BACKEND_LOG_STREAM(::backend::LogLevel::Info, __VA_ARGS__)
#define BACKEND_WARN_STREAM(...) BACKEND_LOG_STREAM(::backend::LogLevel::Warn, __VA_ARGS__)
#define BACKEND_ERROR_STREAM(...) BACKEND_LOG_STREAM(::backend::LogLevel::Error, __VA_ARGS__)
