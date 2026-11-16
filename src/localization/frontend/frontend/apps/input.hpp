/**
 * @file input.hpp
 * @brief 管理传感器回调准入、事件队列及停止排空的生命周期。
 */
#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <variant>

#include "zlio/types.hpp"

namespace zlio::ros {

/// 按入队顺序保存传感器事件，并在关闭后等待已获准的回调结束。
class InputQueue {
public:
    using Event = std::variant<Imu, Scan>;
    enum class CloseMode { Drain, Cancel };

    /// 在作用域内持有一次输入准入，退出时自动归还在途计数。
    class Admission {
    public:
        /// 为开放队列登记一次输入回调。
        explicit Admission(InputQueue& queue);
        /// 归还已获准输入的在途计数。
        ~Admission();
        /// 禁止复制输入准入的唯一所有权。
        Admission(const Admission&) = delete;
        /// 禁止复制输入准入的唯一所有权。
        Admission& operator=(const Admission&) = delete;
        /// 返回是否已取得输入准入。
        explicit operator bool() const { return accepted_; }

    private:
        InputQueue& queue_;
        bool accepted_;
    };

    /// 设置待处理事件数及点数上限。
    InputQueue(std::size_t events, std::size_t points);
    /// 关闭新准入；排空模式接收在途输入，取消模式丢弃其剩余结果。
    void close(CloseMode mode = CloseMode::Drain);
    /// 返回队列是否已经进入失败取消状态。
    bool failed() const;
    /// 接收已获准回调产生的事件；超出容量时抛出异常。
    void push(Event event);
    /// 取得下一事件；正常排空完成或失败取消后返回空值。
    std::optional<Event> take();
    /// 等待在途回调结束并释放剩余事件，调用前应先关闭准入。
    void finish();

private:
    enum class State { Open, Draining, Failed };
    /// 为开放队列增加一个在途输入。
    bool begin();
    /// 减少在途输入并唤醒排空等待者。
    void finishInput();

    const std::size_t maximum_events_, maximum_points_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<Event> events_;
    State state_ = State::Open;
    std::size_t active_inputs_ = 0, queued_points_ = 0;
};

}  // namespace zlio::ros
