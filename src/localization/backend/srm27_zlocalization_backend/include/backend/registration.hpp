/**
 * @file registration.hpp
 * @brief 定义点到面配准结果、目标缓存及配准接口。
 */
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "backend/cloud.hpp"
#include "backend/config.hpp"
#include "backend/measurement.hpp"
#include "backend/quality.hpp"

namespace backend {

/// 点到面配准的位姿、观测信息与质量结果。
struct RegistrationResult {
    /// 配准是否收敛。
    bool converged = false;
    /// 本次配准是否执行了粗层求解。
    bool used_coarse_level = false;
    /// 细层与粗层的总迭代次数。
    int iterations = 0;
    /// 源点云坐标系到目标点云坐标系的变换。
    Eigen::Matrix4d source_to_target = Eigen::Matrix4d::Identity();
    /// 局部配准观测模型的有效 RMSE 上限，m。
    double model_validity_rmse_limit = std::numeric_limits<double>::infinity();
    /// 目标坐标系左扰动下的位姿观测信息，分量顺序为 [平移, 旋转]。
    PoseMeasurementInformation measurement_information;
    /// 满秩位姿观测的目标系左扰动协方差，顺序为 [平移, 旋转]。
    PoseCovariance6d covariance = PoseCovariance6d::Zero();
    /// 位姿观测协方差是否有效。
    bool covariance_valid = false;
    /// 残差、对应支撑与可观性评估结果。
    ObservabilityQuality quality;
    /// 最终细层配准的 Hessian 分析结果。
    PoseHessianAnalysis hessian_analysis;
    /// 最终细层配准的线性求解统计。
    PoseLinearSolveReport linear_solve;

    /// 判断配准是否收敛、位姿有限且具有非零秩的有效观测信息。
    bool likelihoodValid() const {
        return converged && source_to_target.allFinite() && measurement_information.valid &&
               measurement_information.rank > 0;
    }

    /// 判断观测信息有效，且最终 RMSE 位于局部模型的有效范围内。
    bool modelValid() const {
        return likelihoodValid() && std::isfinite(quality.rmse) &&
               std::isfinite(model_validity_rmse_limit) &&
               quality.rmse <= model_validity_rmse_limit;
    }

    /// 判断观测信息、残差和对应支撑是否通过几何验收。
    bool geometryVerified() const { return likelihoodValid() && quality.geometrySupported(); }

    /// 判断配准是否满足完整位姿锚的几何、满秩可观性和协方差有效条件。
    bool anchorValid() const {
        return geometryVerified() && quality.observable && quality.observable_rank == 6 &&
               measurement_information.rank == 6 && covariance_valid && covariance.allFinite();
    }
};

/// 准备目标平面、执行源点云配准并评估观测质量。
class Registration {
public:
    class TargetCache;
    /// 构造点面校正器并关联可选的只读目标准备缓存。
    explicit Registration(const RegistrationConfig& config = RegistrationConfig(),
                          TargetCache* target_cache = nullptr);

    /// 规范化当前配置并清理依赖旧配置的准备状态。
    void reconfigure(const RegistrationConfig& config);
    /// 准备目标点云与局部平面；target_id 为目标标识，负值关闭同目标快速复用检查。
    bool setTarget(const Cloud& target, int target_id);
    /// 准备给定目标并更新目标身份；无效输入清空准备状态。
    bool setTarget(const Cloud::ConstPtr& target, int target_id);
    /// 清除当前目标身份及细、粗层准备结果。
    void invalidateCache();
    /// 返回当前已经完成平面准备的细层目标点云。
    Cloud::ConstPtr cachedTarget() const;
    /// 计算对齐源点落入当前细层目标邻域的比例。
    double overlap(const Cloud& aligned, double max_distance) const;

    /// 从源到目标的初始变换开始配准，返回位姿、观测信息和质量诊断。
    /// information_scale 缩放输出观测信息，coarse_source 提供可选的粗层源点云。
    RegistrationResult align(const Cloud& source, const Eigen::Matrix4d& initial_guess,
                             double information_scale = 1.0,
                             const Cloud::ConstPtr& coarse_source = {});

private:
    struct PreparedPlane;
    struct PreparedLevel;
    struct PlaneCorrespondence;
    struct LevelRun;

    /// 按指定尺度准备目标点云、近邻索引和可信局部平面。
    bool prepareLevel(const Cloud::ConstPtr& target, double scale, PreparedLevel* level) const;
    /// 在需要时从细层持有的原始目标惰性准备粗层。
    bool ensureCoarseLevel();
    /// 按点云身份及完整准备配置取得只读目标层。
    std::shared_ptr<const PreparedLevel> prepareCachedLevel(const Cloud::ConstPtr& target,
                                                            double scale) const;
    /// 按目标平面索引和距离门限建立当前位姿的点面对应。
    std::vector<PlaneCorrespondence> buildCorrespondences(
        const PreparedLevel& level, const Cloud& source,
        const Eigen::Matrix4d& source_to_target) const;
    /// 迭代单层点面配准，保留最终对应和正规方程诊断。
    LevelRun runLevel(const PreparedLevel& level, const Cloud& source,
                      const Eigen::Matrix4d& initial_guess, int max_iterations,
                      std::vector<PlaneCorrespondence> initial_correspondences) const;
    /// 结合残差、覆盖和谱分析计算几何及可观性质量。
    ObservabilityQuality evaluateQuality(const std::vector<PlaneCorrespondence>& correspondences,
                                         int source_points,
                                         const PoseHessianAnalysis& analysis) const;
    /// 按残差尺度和信息缩放系数构造位姿平方根信息。
    PoseMeasurementInformation buildMeasurementInformation(const PoseHessianAnalysis& analysis,
                                                           double rmse, int correspondences,
                                                           double information_scale) const;
    /// 按对应原序计算残差均方根，空集合返回无穷大。
    static double residualRmse(const std::vector<PlaneCorrespondence>& correspondences);
    /// 施加共模位姿协方差并恢复可用的满秩协方差。
    void boundMeasurementInformation(RegistrationResult& result) const;
    /// 累加鲁棒正规方程并求解可观位姿增量。
    Eigen::Matrix4d solveOnce(const std::vector<PlaneCorrespondence>& correspondences,
                              double huber_delta, double max_correspondence_distance,
                              PoseHessianAnalysis* analysis, bool* solved,
                              PoseLinearSolveReport* report = nullptr) const;

    /// 当前生效的配准配置。
    RegistrationConfig config_;
    /// 当前目标标识，负值关闭同目标快速复用检查。
    int cached_prior_index_ = -1;
    /// 已准备的细层目标。
    std::shared_ptr<const PreparedLevel> fine_level_;
    /// 按需准备的粗层目标。
    std::shared_ptr<const PreparedLevel> coarse_level_;
    /// 可选的共享目标缓存，由调用方管理生命周期。
    TargetCache* target_cache_;
};

/// 按字节与条目预算保留细、粗层只读目标准备结果。
class Registration::TargetCache {
public:
    /// 构造具有字节与条目预算的只读目标准备缓存。
    TargetCache(std::size_t bytes, std::size_t entries);
    /// 释放缓存自身持有的准备结果，已有只读租约继续有效。
    ~TargetCache();
    /// 禁用缓存复制构造。
    TargetCache(const TargetCache&) = delete;
    /// 禁用缓存复制赋值。
    TargetCache& operator=(const TargetCache&) = delete;
    /// 清空细、粗层只读目标准备缓存。
    void clear();
    /// 返回当前缓存占用、命中及逐出统计。
    CloudCacheStats stats() const;

private:
    friend class Registration;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace backend
