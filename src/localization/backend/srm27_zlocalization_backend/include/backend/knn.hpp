/**
 * @file knn.hpp
 * @brief 提供精确点云近邻索引、分块查询和内存计量。
 */
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <flann/flann.hpp>
#include <memory>
#include <vector>

#include "backend/cloud.hpp"

namespace backend {
/// 与 PCL 默认 XYZ 表示、L2_Simple<float>、15 点叶节点及精确查询相同。
/// 块内复用连续查询/输出数组；索引只读，可由独占工作区并发查询。
class PointKnnIndex {
public:
    /// 复制有限 XYZ 坐标并构建精确近邻索引和原始点映射。
    void setInputCloud(const Cloud::ConstPtr& cloud) {
        index_.reset();
        coordinates_.clear();
        mapping_.clear();
        coordinates_.reserve(cloud->size() * 3);
        mapping_.reserve(cloud->size());
        for (std::size_t i = 0; i < cloud->size(); ++i) {
            const auto& p = (*cloud)[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;
            coordinates_.insert(coordinates_.end(), {p.x, p.y, p.z});
            mapping_.push_back(i);
        }
        if (mapping_.empty())
            return;
        index_ =
            std::make_unique<Index>(flann::Matrix<float>(coordinates_.data(), mapping_.size(), 3),
                                    flann::KDTreeSingleIndexParams(15));
        index_->buildIndex();
    }
    /// 估算近邻索引及其持有数组的内存占用。
    std::size_t bytes() const {
        // usedMemory 不包括输入和重排后的坐标、NNIndex 的点指针数组。
        return sizeof(*this) + coordinates_.capacity() * sizeof(float) +
               mapping_.capacity() * sizeof(std::size_t) + coordinates_.size() * sizeof(float) +
               mapping_.size() * sizeof(float*) + (index_ ? index_->usedMemory() : 0);
    }
    /// 分块查询有限点的精确近邻，并按原始点序号访问结果。
    template <class Query, class Visitor>
    void query(std::size_t count, int requested, Query&& point_at, Visitor&& visit) const {
        const std::size_t k =
            std::min(mapping_.size(), static_cast<std::size_t>(std::max(0, requested)));
        if (!index_ || k == 0)
            return;
        const std::size_t block_size =
            std::min<std::size_t>(1024, std::max<std::size_t>(1, 65536 / k));
        std::vector<float> xyz, distances(block_size * k);
        std::vector<std::size_t> rows, indices(block_size * k);
        std::vector<Eigen::Vector3d> exact;
        xyz.reserve(block_size * 3);
        rows.reserve(block_size);
        exact.reserve(block_size);
        flann::SearchParams params(-1, 0.0F, true);
        params.cores = 1;  // 候选层已并行；这里不另开线程。
        for (std::size_t begin = 0; begin < count; begin += block_size) {
            xyz.clear();
            rows.clear();
            exact.clear();
            for (std::size_t i = begin; i < std::min(count, begin + block_size); ++i) {
                const Eigen::Vector3d p = point_at(i);
                const Eigen::Vector3f q = p.cast<float>();
                if (!p.allFinite() || !q.allFinite())
                    continue;
                xyz.insert(xyz.end(), {q.x(), q.y(), q.z()});
                rows.push_back(i);
                exact.push_back(p);
            }
            if (rows.empty())
                continue;
            flann::Matrix<std::size_t> ids(indices.data(), rows.size(), k);
            flann::Matrix<float> squared_distances(distances.data(), rows.size(), k);
            index_->knnSearch(flann::Matrix<float>(xyz.data(), rows.size(), 3), ids,
                              squared_distances, k, params);
            for (std::size_t row = 0; row < rows.size(); ++row) {
                for (std::size_t j = 0; j < k; ++j)
                    indices[row * k + j] = mapping_[indices[row * k + j]];
                visit(rows[row], exact[row], indices.data() + row * k, distances.data() + row * k,
                      k);
            }
        }
    }

private:
    using Index = flann::Index<flann::L2_Simple<float>>;
    std::vector<float> coordinates_;
    std::vector<std::size_t> mapping_;
    std::unique_ptr<Index> index_;
};
}  // namespace backend
