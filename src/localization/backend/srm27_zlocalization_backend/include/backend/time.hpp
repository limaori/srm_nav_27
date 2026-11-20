/**
 * @file time.hpp
 * @brief 使用整数纳秒表达传感器与调度时间边界。
 */
#pragma once

#include <cmath>
#include <cstdint>

namespace backend {

/// 以整数纳秒存储的时间戳。
class Timestamp {
public:
    /// 构造以整数纳秒存储的时间戳。
    constexpr Timestamp() = default;

    /// 由纳秒构造（与 rclcpp::Time::nanoseconds() 对接）。
    static constexpr Timestamp fromNanoseconds(std::int64_t ns) { return Timestamp(ns); }

    /// 由秒构造，四舍五入到最近纳秒。
    static Timestamp fromSeconds(double seconds) {
        return Timestamp(static_cast<std::int64_t>(std::llround(seconds * 1e9)));
    }

    /// 返回整数纳秒表示。
    std::int64_t nanoseconds() const { return ns_; }
    /// 返回以秒为单位的时间值。
    double seconds() const { return static_cast<double>(ns_) * 1e-9; }

    /// 判断时间值是否为零。
    bool isZero() const { return ns_ == 0; }

    /// 两个时间戳之差，单位秒。语义等价 (a - b).seconds()。
    double operator-(const Timestamp& rhs) const {
        return static_cast<double>(ns_ - rhs.ns_) * 1e-9;
    }

    /// 比较两个时间戳是否相等。
    bool operator==(const Timestamp& rhs) const { return ns_ == rhs.ns_; }
    /// 比较两个时间戳是否不同。
    bool operator!=(const Timestamp& rhs) const { return ns_ != rhs.ns_; }
    /// 比较两个时间戳的先后顺序。
    bool operator<(const Timestamp& rhs) const { return ns_ < rhs.ns_; }
    /// 比较当前时间戳是否不早于给定时刻。
    bool operator>=(const Timestamp& rhs) const { return ns_ >= rhs.ns_; }

private:
    /// 构造以整数纳秒存储的时间戳。
    explicit constexpr Timestamp(std::int64_t ns) : ns_(ns) {}

    std::int64_t ns_ = 0;
};

}  // namespace backend
