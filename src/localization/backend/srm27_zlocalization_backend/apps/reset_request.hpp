/**
 * @file reset_request.hpp
 * @brief 管理单次 ROS 重置请求的传输登记及超时回收。
 */
#pragma once

#include <chrono>
#include <localization_interfaces/srv/reset_frontend.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/client.hpp>

namespace backend::ros {
/// 只拥有一条 ROS 传输请求；清理回调登记不结束上层的幂等 reset 事务。
class ResetRequest {
public:
    using Client = rclcpp::Client<localization_interfaces::srv::ResetFrontend>;
    using Clock = std::chrono::steady_clock;

    /// 替换旧传输登记并发送单次异步请求，保存本次请求编号和发送时刻。
    std::int64_t send(Client& client, Client::SharedRequest request, Client::CallbackType callback,
                      Clock::time_point now = Clock::now()) {
        std::lock_guard<std::mutex> lock(mutex_);
        clear(client);
        const auto pending = client.async_send_request(std::move(request), std::move(callback));
        request_id_ = pending.request_id;
        sent_at_ = now;
        return *request_id_;
    }

    /// 按稳态时间回收已经超时的 ROS 请求回调登记。
    void expire(Client& client, double timeout_seconds, Clock::time_point now = Clock::now()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (request_id_ && std::chrono::duration<double>(now - sent_at_).count() >= timeout_seconds)
            clear(client);
    }

private:
    /// 移除当前传输请求的回调登记，并清空本地持有的请求编号。
    void clear(Client& client) {
        if (request_id_)
            client.remove_pending_request(*request_id_);
        request_id_.reset();
    }

    std::mutex mutex_;
    std::optional<std::int64_t> request_id_;
    Clock::time_point sent_at_;
};
}  // namespace backend::ros
