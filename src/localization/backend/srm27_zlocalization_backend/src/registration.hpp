/**
 * @file registration.hpp
 * @brief 保存点面配准的私有准备层、单次求解工作区和缓存实现。
 */
#pragma once

#include <tuple>

#include "backend/cache.hpp"
#include "backend/knn.hpp"
#include "backend/registration.hpp"

namespace backend {

struct Registration::PreparedPlane {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    bool valid = false;
};

struct Registration::PreparedLevel {
    Cloud::ConstPtr source;  // 保留缓存键身份；粗层另持有降采样云。
    Cloud::ConstPtr target;
    std::vector<PreparedPlane> planes;
    PointKnnIndex tree;
    double max_correspondence_distance = 0.0;
    double huber_delta = 0.0;
    int min_correspondences = 1;
};

struct Registration::PlaneCorrespondence {
    Eigen::Vector3d source;
    Eigen::Vector3d target;
    Eigen::Vector3d normal;
    double residual = 0.0;
};

struct Registration::LevelRun {
    Eigen::Matrix4d estimate = Eigen::Matrix4d::Identity();
    PoseHessianAnalysis hessian_analysis;
    PoseLinearSolveReport linear_solve;
    std::vector<PlaneCorrespondence> correspondences;
    int iterations = 0;
    bool converged = false;
};

struct Registration::TargetCache::Impl {
    // 点云身份已涵盖地图版本与子图选择；其余字段覆盖 prepareLevel 使用的配置。
    using Key = std::tuple<std::uintptr_t, double, double, int, double, double, double, double,
                           double, int>;
    /// 设置准备结果的字节与条目预算。
    Impl(std::size_t bytes, std::size_t entries) : cache(bytes, entries) {}
    CloudCache<Key, PreparedLevel> cache;
};

}  // namespace backend
