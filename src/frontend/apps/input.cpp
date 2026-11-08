/**
 * @file input.cpp
 * @brief 实现输入准入、容量限制及排空或取消时的线程汇合。
 */
#include "input.hpp"

#include <stdexcept>
#include <utility>

namespace zlio::ros {

/// 申请输入准入，并借用队列直到当前回调退出。
InputQueue::Admission::Admission(InputQueue& queue) : queue_(queue), accepted_(queue.begin()) {
}

/// 为已获准的输入回调归还在途计数。
InputQueue::Admission::~Admission() {
    if (accepted_)
        queue_.finishInput();
}

/// 保存待处理事件数及点数上限。
InputQueue::InputQueue(std::size_t events, std::size_t points)
    : maximum_events_(events), maximum_points_(points) {
}

/// 关闭新输入，选择排空或取消待处理事件。
void InputQueue::close(CloseMode mode) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode == CloseMode::Cancel || state_ == State::Failed)
            state_ = State::Failed;
        else
            state_ = State::Draining;
    }
    changed_.notify_all();
}

/// 返回输入通道是否因错误关闭。
bool InputQueue::failed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == State::Failed;
}

/// 接收已经取得准入的回调所产生的事件。
void InputQueue::push(Event event) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == State::Failed)
            return;
        const auto* scan = std::get_if<Scan>(&event);
        const auto points = scan ? scan->size() : 0;
        if (events_.size() >= maximum_events_ || points > maximum_points_ - queued_points_)
            throw std::length_error("input event queue capacity exceeded");
        events_.push_back(std::move(event));
        queued_points_ += points;
    }
    changed_.notify_one();
}

/// 取得下一事件，在关闭并完成排空后返回空值。
std::optional<InputQueue::Event> InputQueue::take() {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [&] {
        return state_ == State::Failed || !events_.empty() ||
               (state_ == State::Draining && active_inputs_ == 0);
    });
    if (state_ == State::Failed || events_.empty())
        return std::nullopt;
    Event event = std::move(events_.front());
    events_.pop_front();
    if (const auto* scan = std::get_if<Scan>(&event))
        queued_points_ -= scan->size();
    return event;
}

/// 等待在途回调结束并释放剩余事件。
void InputQueue::finish() {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [&] { return active_inputs_ == 0; });
    events_.clear();
    queued_points_ = 0;
}

/// 为开放队列增加一个在途输入。
bool InputQueue::begin() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::Open)
        return false;
    ++active_inputs_;
    return true;
}

/// 减少在途输入并唤醒排空等待者。
void InputQueue::finishInput() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        --active_inputs_;
    }
    changed_.notify_all();
}

}  // namespace zlio::ros
