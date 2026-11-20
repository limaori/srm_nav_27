/**
 * @file frontend_connection.hpp
 * @brief 拥有前端实例准入、重置客户端及订阅代次。
 */
#pragma once
#include <atomic>
#include <cstdint>
#include <exception>
#include <functional>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "apps/frontend_session.hpp"
#include "apps/reset_request.hpp"

namespace backend {
class Localization;
}

namespace backend::ros {
/// ROS 前端连接是重置端口的具体实现，业务图状态仍由控制器唯一提交。
class FrontendConnection final : public FrontendResetPort {
public:
    using FailureHandler = std::function<void(const char*, std::exception_ptr)>;
    using Boundary = std::function<void()>;
    /// 绑定节点、故障处理和输入边界重建回调。
    FrontendConnection(rclcpp::Node& node, const std::atomic<bool>& failed, FailureHandler failure,
                       Boundary boundary);
    /// 保存输入协议类型与重置服务名称。
    void configure(bool atomic_input, std::string service);
    /// 绑定业务控制器及其当前代次。
    void bind(Localization& controller);
    /// 创建前端重置服务客户端。
    void connect();
    /// 回收超时传输，并推进身份确认和输入中断恢复。
    void maintain(double timeout_seconds);
    /// 返回当前订阅使用的后端代次。
    std::uint64_t epoch() const { return epoch_.load(); }
    /// 验收会话身份、帧序号及故障通知，返回是否允许接入当前输入。
    bool acceptIdentity(std::uint64_t instance, std::uint64_t epoch,
                        std::optional<std::uint64_t> sequence = std::nullopt,
                        bool reset_required = false, bool failed = false);
    /// 查询前端重置服务是否可用。
    bool frontendResetReady() const override;
    /// 发送带固定业务身份的幂等重置请求。
    bool sendFrontendReset(const FrontendResetTransaction& transaction) override;
    /// 确认新会话并重建订阅和输出水位。
    void onFrontendResetBoundary() override;

private:
    /// 按观测时的后端代次提交身份确认或输入故障。
    void applySessionAction(FrontendSession::Admission action, std::uint64_t expected_epoch);
    rclcpp::Node& node_;
    const std::atomic<bool>& failed_;
    FailureHandler failure_;
    Boundary boundary_;
    Localization* controller_ = nullptr;
    bool atomic_input_ = false;
    std::string service_;
    std::atomic<std::uint64_t> epoch_{0};
    FrontendSession session_;
    ResetRequest reset_request_;
    ResetRequest::Client::SharedPtr reset_client_;
};
}  // namespace backend::ros
