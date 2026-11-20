/**
 * @file cache.hpp
 * @brief 按字节和条目预算缓存只读几何，并合并相同键的并发加载。
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <future>
#include <limits>
#include <list>
#include <map>
#include <mutex>

#include "backend/cloud.hpp"

namespace backend {
/// 维护有界只读对象缓存；同键并发缺失共享一次加载及其异常结果。
template <class Key, class Value = Cloud>
class CloudCache {
public:
    using ConstPtr = std::shared_ptr<const Value>;
    /// 设置缓存字节预算及最多保留的条目数量。
    explicit CloudCache(std::size_t byte_limit,
                        std::size_t entry_limit = std::numeric_limits<std::size_t>::max())
        : byte_limit_(byte_limit), entry_limit_(entry_limit) {}

    /// 按点云容器容量计费查询缓存；空点云只返回给调用方，不占用缓存条目。
    template <class Loader>
    ConstPtr get(const Key& key, Loader&& loader) const {
        return get(key, std::forward<Loader>(loader), [](const Value& value) {
            return value.empty() ? 0 : value.points.capacity() * sizeof(PointType);
        });
    }

    /// 按调用方提供的字节计量查询或加载对象，并合并同键并发缺失。
    template <class Loader, class Measure>
    ConstPtr get(const Key& key, Loader&& loader, Measure&& measure) const {
        std::shared_ptr<std::promise<ConstPtr>> promise;
        std::shared_future<ConstPtr> future;
        std::uint64_t generation;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto found = entries_.find(key);
            if (found != entries_.end()) {
                ++stats_.hits;
                lru_.splice(lru_.begin(), lru_, found->second.position);
                return found->second.cloud;
            }
            generation = generation_;
            auto loading = pending_.find(key);
            if (loading != pending_.end()) {
                ++stats_.waits;
                future = loading->second;
            } else {
                ++stats_.loads;
                promise = std::make_shared<std::promise<ConstPtr>>();
                future = promise->get_future().share();
                pending_.emplace(key, future);
            }
        }
        // 合并的调用在锁外等待同一加载者，不延长缓存临界区。
        if (!promise)
            return future.get();
        try {
            ConstPtr cloud = loader();  // 不持缓存或地图锁进行 I/O/几何计算。
            {
                std::lock_guard<std::mutex> lock(mutex_);
                // clear 后的旧加载仍可满足原调用，但不得重新填充当前缓存。
                if (generation == generation_) {
                    pending_.erase(key);
                    const std::size_t bytes = cloud ? measure(*cloud) : 0;
                    retainLocked(key, cloud, bytes);
                }
            }
            promise->set_value(cloud);
            return cloud;
        } catch (...) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (generation == generation_)
                    pending_.erase(key);
            }
            promise->set_exception(std::current_exception());
            throw;
        }
    }

    /// 推进缓存代次并清空条目及待加载登记，已有租约和累计统计继续有效。
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        ++generation_;
        entries_.clear();
        lru_.clear();
        pending_.clear();
        stats_.bytes = 0;
    }
    /// 返回当前缓存占用及累计命中、加载和逐出统计。
    CloudCacheStats stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = stats_;
        result.entries = entries_.size();
        return result;
    }

private:
    struct Entry {
        ConstPtr cloud;
        std::size_t bytes;
        typename std::list<Key>::iterator position;
    };
    /// 在缓存锁内按字节与条目预算逐出旧项，再保留可计费的只读对象。
    void retainLocked(const Key& key, const ConstPtr& cloud, std::size_t bytes) const {
        if (cloud && bytes > 0 && bytes <= byte_limit_ && entry_limit_ > 0) {
            while (!lru_.empty() &&
                   (stats_.bytes + bytes > byte_limit_ || entries_.size() >= entry_limit_)) {
                const auto last = entries_.find(lru_.back());
                stats_.bytes -= last->second.bytes;
                entries_.erase(last);
                lru_.pop_back();
                ++stats_.evictions;
            }
            lru_.push_front(key);
            entries_.emplace(key, Entry{cloud, bytes, lru_.begin()});
            stats_.bytes += bytes;
            stats_.peak_bytes = std::max(stats_.peak_bytes, stats_.bytes);
        }
    }

    std::size_t byte_limit_, entry_limit_;
    mutable std::mutex mutex_;
    mutable std::map<Key, Entry> entries_;
    mutable std::list<Key> lru_;
    mutable std::map<Key, std::shared_future<ConstPtr>> pending_;
    mutable CloudCacheStats stats_;
    std::uint64_t generation_ = 0;
};
}  // namespace backend
