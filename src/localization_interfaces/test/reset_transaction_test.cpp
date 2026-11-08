/**
 * @file reset_transaction_test.cpp
 * @brief 验证重置事务的幂等身份、完整边界回复和非成功结果隔离。
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <thread>

#include "localization_interfaces/reset_transaction_cache.hpp"

namespace localization_interfaces {
namespace {
using Cache = ResetTransactionCache;
using Response = Cache::Response;

/// 构造具有独立事务编号及目标代次的有效请求。
Cache::Request request(std::uint64_t transaction = 1, std::uint64_t epoch = 2) {
    Cache::Request value;
    value.requester_instance_id = 11;
    value.transaction_id = transaction;
    value.target_epoch = epoch;
    return value;
}

/// 构造包含非平凡坐标变换和纳秒时间的暖重置终态。
Response warmResult() {
    Response result;
    result.outcome = Response::COMPLETED;
    result.message = "completed with odometry boundary";
    result.has_boundary = true;
    result.source_epoch = 7;
    result.boundary_stamp.sec = 19;
    result.boundary_stamp.nanosec = 987654321;
    result.old_from_new.translation.x = 2.5;
    result.old_from_new.translation.y = -3.25;
    result.old_from_new.translation.z = .75;
    result.old_from_new.rotation.x = .5;
    result.old_from_new.rotation.y = -.5;
    result.old_from_new.rotation.z = .5;
    result.old_from_new.rotation.w = .5;
    return result;
}

/// 验证不携带边界的回复同时清除全部附属字段。
void expectNoBoundary(const Response& response) {
    const Response empty;
    EXPECT_FALSE(response.has_boundary);
    EXPECT_EQ(response.source_epoch, 0U);
    EXPECT_EQ(response.boundary_stamp, empty.boundary_stamp);
    EXPECT_EQ(response.old_from_new, empty.old_from_new);
}

TEST(ResetTransaction, DuplicateReusesActiveAndTerminalReply) {
    Cache cache;
    Response response;
    const auto value = request();
    ASSERT_EQ(cache.begin(value, &response), Cache::BeginResult::Execute);
    EXPECT_EQ(cache.begin(value, &response), Cache::BeginResult::Responded);
    EXPECT_EQ(response.outcome, Response::IN_PROGRESS);
    cache.finish(value, Response::COMPLETED, "completed", &response);
    EXPECT_EQ(response.reset_generation, 1U);
    EXPECT_EQ(response.frontend_instance_id, cache.frontendInstanceId());
    const auto terminal = response;
    cache.begin(value, &response);
    EXPECT_EQ(response, terminal);
}

/// 暖重置完整终态独立于后续事务推进，重复请求逐字段复用原结果。
TEST(ResetTransaction, DuplicatePreservesCompleteWarmBoundaryAfterLaterTransactions) {
    Cache cache;
    Response response;
    const auto value = request(1, 8);
    ASSERT_EQ(cache.begin(value, &response), Cache::BeginResult::Execute);
    const Response result = warmResult();
    cache.finish(value, result, &response);
    ASSERT_TRUE(response.has_boundary);
    EXPECT_EQ(response.source_epoch, result.source_epoch);
    EXPECT_EQ(response.boundary_stamp, result.boundary_stamp);
    EXPECT_EQ(response.old_from_new, result.old_from_new);
    EXPECT_EQ(response.requester_instance_id, value.requester_instance_id);
    EXPECT_EQ(response.transaction_id, value.transaction_id);
    EXPECT_EQ(response.target_epoch, value.target_epoch);
    EXPECT_EQ(response.frontend_instance_id, cache.frontendInstanceId());
    EXPECT_EQ(response.reset_generation, 1U);
    const Response terminal = response;
    ASSERT_EQ(cache.begin(request(2, 9), &response), Cache::BeginResult::Execute);
    cache.finish(request(2, 9), Response::COMPLETED, "cold session", &response);
    expectNoBoundary(response);
    EXPECT_EQ(response.reset_generation, 2U);
    EXPECT_EQ(cache.begin(value, &response), Cache::BeginResult::Responded);
    EXPECT_EQ(response, terminal);
}

/// 拒绝和在途结果不得残留调用方旧回复中的暖重置边界。
TEST(ResetTransaction, AdmissionRepliesClearStaleBoundaryFields) {
    Cache cache;
    Response response = warmResult();
    ASSERT_EQ(cache.begin(request(), &response), Cache::BeginResult::Execute);
    expectNoBoundary(response);
    response = warmResult();
    EXPECT_EQ(cache.begin(request(), &response), Cache::BeginResult::Responded);
    EXPECT_EQ(response.outcome, Response::IN_PROGRESS);
    expectNoBoundary(response);
    response = warmResult();
    EXPECT_EQ(cache.begin(request(1, 3), &response), Cache::BeginResult::Responded);
    EXPECT_EQ(response.outcome, Response::REJECTED);
    expectNoBoundary(response);
    response = warmResult();
    EXPECT_EQ(cache.begin(request(2), &response), Cache::BeginResult::Responded);
    EXPECT_EQ(response.outcome, Response::REJECTED);
    expectNoBoundary(response);
    response = warmResult();
    cache.finish(request(2), warmResult(), &response);
    EXPECT_EQ(response.outcome, Response::INDETERMINATE);
    expectNoBoundary(response);
}

/// 失败、无效终态及冷启动成功都清除提交载荷中的边界。
TEST(ResetTransaction, OnlyCompletedWarmResultMayContainBoundary) {
    Cache cache;
    const std::array<std::uint8_t, 5> outcomes{Response::REJECTED, Response::INDETERMINATE,
                                               Response::IN_PROGRESS, 255, Response::COMPLETED};
    for (std::size_t index = 0; index < outcomes.size(); ++index) {
        const auto value = request(index + 1);
        Response response = warmResult();
        ASSERT_EQ(cache.begin(value, &response), Cache::BeginResult::Execute);
        Response result = warmResult();
        result.outcome = outcomes[index];
        if (result.outcome == Response::COMPLETED)
            result.has_boundary = false;
        cache.finish(value, result, nullptr);
        ASSERT_EQ(cache.begin(value, &response), Cache::BeginResult::Responded);
        expectNoBoundary(response);
    }
}

TEST(ResetTransaction, ConflictingIdentityCannotReplaceActiveWork) {
    Cache cache;
    Response response;
    ASSERT_EQ(cache.begin(request(), &response), Cache::BeginResult::Execute);
    EXPECT_EQ(cache.begin(request(2), &response), Cache::BeginResult::Responded);
    EXPECT_EQ(response.outcome, Response::REJECTED);
    cache.begin(request(1, 3), &response);
    EXPECT_EQ(response.outcome, Response::REJECTED);
    cache.finish(request(), Response::COMPLETED, "completed", &response);
    EXPECT_EQ(response.outcome, Response::COMPLETED);
}

TEST(ResetTransaction, TerminalOutcomesAdvanceGenerationAccordingToSideEffects) {
    Cache cache;
    Response response;
    const std::array<std::uint8_t, 4> outcomes{Response::REJECTED, Response::COMPLETED,
                                               Response::INDETERMINATE, 255};
    const std::array<std::uint64_t, 4> generations{0, 1, 2, 3};
    for (std::size_t index = 0; index < outcomes.size(); ++index) {
        const auto value = request(index + 1);
        ASSERT_EQ(cache.begin(value, &response), Cache::BeginResult::Execute);
        cache.finish(value, outcomes[index], "terminal", &response);
        EXPECT_EQ(response.reset_generation, generations[index]);
        EXPECT_EQ(response.outcome, index == 3 ? Response::INDETERMINATE : outcomes[index]);
    }
}

TEST(ResetTransaction, InvalidIdentifiersDoNotOccupyCache) {
    Cache cache;
    Response response;
    for (int field = 0; field < 3; ++field) {
        auto value = request();
        if (field == 0)
            value.requester_instance_id = 0;
        if (field == 1)
            value.transaction_id = 0;
        if (field == 2)
            value.target_epoch = 0;
        EXPECT_EQ(cache.begin(value, &response), Cache::BeginResult::Responded);
        EXPECT_EQ(response.outcome, Response::REJECTED);
    }
    EXPECT_EQ(cache.begin(request(), nullptr), Cache::BeginResult::Responded);
    EXPECT_EQ(cache.begin(request(), &response), Cache::BeginResult::Execute);
}

TEST(ResetTransaction, ConcurrentDuplicatesAuthorizeExactlyOneExecution) {
    Cache cache;
    std::array<Cache::BeginResult, 8> results{};
    std::array<std::thread, 8> workers;
    for (std::size_t index = 0; index < workers.size(); ++index)
        workers[index] = std::thread([&, index] {
            Response response;
            results[index] = cache.begin(request(), &response);
        });
    for (auto& worker : workers)
        worker.join();
    EXPECT_EQ(std::count(results.begin(), results.end(), Cache::BeginResult::Execute), 1);
}

TEST(ResetTransaction, BoundedCacheRetainsNewestTerminalAndActiveEntry) {
    Cache cache(1);
    Response response;
    cache.begin(request(1), &response);
    cache.finish(request(1), Response::COMPLETED, "first", &response);
    cache.begin(request(2), &response);
    cache.begin(request(2), &response);
    EXPECT_EQ(response.outcome, Response::IN_PROGRESS);
    cache.finish(request(2), Response::COMPLETED, "second", &response);
    cache.begin(request(2), &response);
    EXPECT_EQ(response.message, "second");
    EXPECT_EQ(response.reset_generation, 2U);
}
}  // namespace
}  // namespace localization_interfaces
