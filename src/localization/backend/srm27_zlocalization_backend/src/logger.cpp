/**
 * @file logger.cpp
 * @brief 管理默认日志输出并提供控制台后备实现。
 */

#include "backend/logger.hpp"

#include <atomic>
#include <iostream>

namespace backend {
namespace {

/// 控制台后备 sink：未安装后端时使用，按级别输出到 cerr/cout。
class ConsoleLogger : public Logger {
public:
    /// 将一条已格式化的消息提交到对应级别的日志输出。
    void log(LogLevel level, const std::string& message) override {
        switch (level) {
            case LogLevel::Error:
            case LogLevel::Warn:
                std::cerr << "[backend] " << message << "\n";
                break;
            case LogLevel::Info:
            case LogLevel::Debug:
                std::cout << "[backend] " << message << "\n";
                break;
        }
    }
};

/// 返回进程内控制台日志输出实例。
ConsoleLogger& consoleLogger() {
    static ConsoleLogger instance;
    return instance;
}

// 当前 sink。启动阶段单次安装，运行期只读，atomic 足以保证可见性。
std::atomic<Logger*> g_logger{nullptr};

}  // namespace

/// 返回已安装的日志输出，未安装时使用控制台。
Logger& defaultLogger() {
    Logger* logger = g_logger.load(std::memory_order_acquire);
    return logger != nullptr ? *logger : consoleLogger();
}

/// 原子安装默认日志输出，空指针恢复控制台输出。
void setDefaultLogger(Logger* logger) {
    g_logger.store(logger, std::memory_order_release);
}

}  // namespace backend
