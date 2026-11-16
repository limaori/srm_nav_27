/**
 * @file reset_transaction_cache.hpp
 * @brief 保存有界的进程内重置事务，复用重复请求的执行状态和终态回复。
 */
#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <random>
#include <string>
#include <utility>

#include "localization_interfaces/srv/reset_frontend.hpp"

namespace localization_interfaces {

/// 独占重置事务及本进程身份；缓存锁不跨越调用方的实际重置操作。
class ResetTransactionCache {
public:
    using Service = localization_interfaces::srv::ResetFrontend;
    using Request = Service::Request;
    using Response = Service::Response;

    /// 区分调用方应执行重置，还是回复已经由缓存填写。
    enum class BeginResult {
        Execute,
        Responded,
    };

    /// 创建进程身份并设置终态缓存容量，至少保留一条事务。
    explicit ResetTransactionCache(const std::size_t capacity = 32)
        : capacity_(capacity > 0 ? capacity : 1), frontend_instance_id_(makeInstanceId()) {}

    /// 登记唯一在途事务，拒绝冲突身份，或复用相同幂等键的已有回复。
    /// 空回复指针不登记事务，并返回 Responded。
    BeginResult begin(const Request& request, Response* response) {
        if (response == nullptr) {
            return BeginResult::Responded;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const Key key{request.requester_instance_id, request.transaction_id};
        if (key.first == 0 || key.second == 0 || request.target_epoch == 0) {
            fillResponse(request, Response::REJECTED,
                         "reset transaction identifiers must be non-zero", reset_generation_,
                         response);
            return BeginResult::Responded;
        }

        const auto existing = entries_.find(key);
        if (existing != entries_.end()) {
            if (existing->second.target_epoch != request.target_epoch) {
                fillResponse(request, Response::REJECTED,
                             "idempotency key was reused with a different target epoch",
                             reset_generation_, response);
                return BeginResult::Responded;
            }
            *response = existing->second;
            return BeginResult::Responded;
        }

        if (active_.first != 0 || active_.second != 0) {
            fillResponse(request, Response::REJECTED, "another reset transaction is active",
                         reset_generation_, response);
            return BeginResult::Responded;
        }

        Response pending;
        fillResponse(request, Response::IN_PROGRESS, "reset transaction is still in progress",
                     reset_generation_, &pending);
        *response = pending;
        entries_.emplace(key, std::move(pending));
        insertion_order_.push_back(key);
        active_ = key;
        pruneLocked();
        return BeginResult::Execute;
    }

    /// 提交不携带坐标系边界的终态，空指针仅省略回填。
    void finish(const Request& request, const std::uint8_t outcome, std::string message,
                Response* response) {
        Response result;
        result.outcome = outcome;
        result.message = std::move(message);
        finish(request, std::move(result), response);
    }

    /// 保存完整终态及成功暖重置的坐标系边界，统一填写身份并推进重置代次。
    void finish(const Request& request, Response result, Response* response) {
        std::lock_guard<std::mutex> lock(mutex_);
        const Key key{request.requester_instance_id, request.transaction_id};
        auto entry = entries_.find(key);
        if (entry == entries_.end() || entry->second.outcome != Response::IN_PROGRESS ||
            entry->second.target_epoch != request.target_epoch) {
            if (response != nullptr) {
                fillResponse(request, Response::INDETERMINATE,
                             "reset transaction cache lost active state", reset_generation_,
                             response);
            }
            return;
        }

        std::uint8_t terminal = result.outcome;
        if (terminal != Response::REJECTED && terminal != Response::COMPLETED &&
            terminal != Response::INDETERMINATE) {
            terminal = Response::INDETERMINATE;
            result.message = "reset server produced an invalid terminal outcome";
        }
        if (terminal == Response::COMPLETED || terminal == Response::INDETERMINATE) {
            ++reset_generation_;
            if (reset_generation_ == 0) {
                ++reset_generation_;
            }
        }

        Response terminal_response;
        fillResponse(request, terminal, result.message, reset_generation_, &terminal_response);
        if (terminal == Response::COMPLETED && result.has_boundary) {
            terminal_response.has_boundary = true;
            terminal_response.source_epoch = result.source_epoch;
            terminal_response.boundary_stamp = result.boundary_stamp;
            terminal_response.old_from_new = result.old_from_new;
        }
        entry->second = std::move(terminal_response);
        active_ = Key{};
        if (response != nullptr)
            *response = entry->second;
    }

    /// 返回当前缓存所属的前端进程身份，缓存存续期间保持不变。
    std::uint64_t frontendInstanceId() const noexcept { return frontend_instance_id_; }

private:
    using Key = std::pair<std::uint64_t, std::uint64_t>;

    /// 混合进程内序号、稳态时钟和可用随机源，生成非零实例标识。
    static std::uint64_t makeInstanceId() noexcept {
        static std::atomic<std::uint64_t> sequence{1};
        const std::uint64_t counter = sequence.fetch_add(1, std::memory_order_relaxed);
        const std::uint64_t ticks =
            static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        std::uint64_t entropy = 0;
        try {
            std::random_device random;
            entropy = (static_cast<std::uint64_t>(random()) << 32U) ^
                      static_cast<std::uint64_t>(random());
        } catch (...) {
            entropy = 0;
        }
        std::uint64_t value = entropy ^ ticks ^ (counter * 0x9e3779b97f4a7c15ULL);
        value ^= value >> 30U;
        value *= 0xbf58476d1ce4e5b9ULL;
        value ^= value >> 27U;
        value *= 0x94d049bb133111ebULL;
        value ^= value >> 31U;
        return value != 0 ? value : counter;
    }

    /// 回填完整请求身份及执行结果，调用方保证回复指针有效。
    void fillResponse(const Request& request, const std::uint8_t outcome,
                      const std::string& message, const std::uint64_t generation,
                      Response* response) const {
        *response = Response{};
        response->outcome = outcome;
        response->message = message;
        response->requester_instance_id = request.requester_instance_id;
        response->transaction_id = request.transaction_id;
        response->target_epoch = request.target_epoch;
        response->frontend_instance_id = frontend_instance_id_;
        response->reset_generation = generation;
    }

    /// 在持锁状态下按登记顺序淘汰终态，保留仍在执行的事务。
    void pruneLocked() {
        while (entries_.size() > capacity_ && !insertion_order_.empty()) {
            const Key oldest = insertion_order_.front();
            insertion_order_.pop_front();
            const auto entry = entries_.find(oldest);
            if (entry == entries_.end()) {
                continue;
            }
            if (entry->second.outcome == Response::IN_PROGRESS) {
                insertion_order_.push_back(oldest);
                break;
            }
            entries_.erase(entry);
        }
    }

    const std::size_t capacity_;
    const std::uint64_t frontend_instance_id_;
    std::mutex mutex_;
    std::map<Key, Response> entries_;
    std::deque<Key> insertion_order_;
    Key active_;
    std::uint64_t reset_generation_ = 0;
};

}  // namespace localization_interfaces
