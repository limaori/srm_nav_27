/**
 * @file search.hpp
 * @brief 声明一次重定位请求内共享的候选、验证结果和运行上下文。
 */
#pragma once

#include <limits>
#include <string>
#include <vector>

#include "backend/quality.hpp"
#include "backend/registration.hpp"
#include "backend/search.hpp"

namespace backend::relocalization_detail {

struct VerificationEntry {
    Eigen::Matrix4d T_map_prior = Eigen::Matrix4d::Identity();
    std::string label;
    double yaw_seed = 0.0;
    Eigen::Matrix4d source_to_target_seed = Eigen::Matrix4d::Identity();
};

struct RelocalizationBest {
    bool found = false;
    int index = -1;
    std::string source;
    double descriptor_distance = std::numeric_limits<double>::max();
    double descriptor_score = 0.0;
    double descriptor_yaw = 0.0;
    double residual_score = std::numeric_limits<double>::max();
    PoseCovariance6d covariance = PoseCovariance6d::Zero();
    bool covariance_valid = false;
    ObservabilityQuality quality;
    Eigen::Matrix4d T_map_odom = Eigen::Matrix4d::Identity();
    std::string hypothesis;
};

struct VerificationRunResult {
    bool success = false;
    Eigen::Matrix4d measured = Eigen::Matrix4d::Identity();
    double residual_score = std::numeric_limits<double>::max();
    PoseCovariance6d covariance = PoseCovariance6d::Zero();
    bool covariance_valid = false;
    ObservabilityQuality quality;
    int tried_alignments = 0;
    double best_observed_residual_score = std::numeric_limits<double>::max();
    ObservabilityQuality best_observed_quality;
};

struct CandidateVerificationInput {
    std::string source;
    LoopCandidate hit;
    Eigen::Matrix4d T_map_prior = Eigen::Matrix4d::Identity();
    std::vector<VerificationEntry> entries;
    Cloud::ConstPtr target;
};

struct CandidateVerificationResult {
    int tried = 0;
    double best_failed_residual_score = std::numeric_limits<double>::max();
    ObservabilityQuality best_failed_quality;
    RelocalizationBest best;
};

class Search {
public:
    /// 绑定一次恢复计算必需的地图、描述子、配置和请求引用。
    Search(const KeyframeMap& keyframes, const PlaceDescriptor& descriptor,
           RelocalizationTargetCache& target_cache, const RelocalizationRequest& request);

    /// 执行当前请求对应的位置来源或描述子回退链。
    RelocalizationResult run();

private:
    /// 按三维空间半径筛选描述子召回允许的先验关键帧。
    std::vector<int> findPriorKeyframesWithinRadius(const Eigen::Vector3d& center,
                                                    double radius) const;
    /// 返回候选中的最小描述子距离，空集合返回最大值。
    static double bestRetrievalDistance(const std::vector<LoopCandidate>& items);
    /// 准备局部描述子召回并确定全局补充预算和距离门限。
    bool queryCandidates();
    /// 按描述子约定整平当前源云，保留原始匹配源坐标。
    void levelQueryCloud();
    /// 准备细层配准源并记录目标子图的关键帧预算。
    bool prepareScanToMapInputs();
    /// 按先验关键帧和子图配置取得可复用的几何验证目标。
    RelocalizationPreparedTarget directTargetFor(int center_index);
    /// 将配准支撑、残差和可观性诊断格式化为日志文本。
    static std::string qualitySummary(const ObservabilityQuality& quality);
    /// 检查候选编号、通用距离门限和启动专用距离门限。
    bool descriptorGatePassed(const LoopCandidate& hit) const;

    /// 按确定性质量排序更新最佳恢复结果并转换协方差坐标。
    void recordIfBest(RelocalizationBest& best, const CandidateVerificationInput& input,
                      const VerificationEntry& entry, const VerificationRunResult& result) const;
    /// 格式化航向初值及相对角度偏移的诊断标签。
    static std::string yawSampleLabel(const std::string& prefix, int offset, double step_deg);
    /// 将描述子整平坐标的初值转换到原始源与目标坐标。
    Eigen::Matrix4d descriptorSeedToRawSourceToTarget(const Eigen::Matrix4d& descriptor_seed,
                                                      const Eigen::Matrix4d& T_map_prior,
                                                      bool level_with_odom_roll_pitch) const;
    /// 根据描述子航向初值及偏移生成单个几何验证假设。
    VerificationEntry makeVerificationEntry(const CandidateVerificationInput& input, int yaw_offset,
                                            double yaw_step_rad, double yaw_step_deg) const;
    /// 由物理位置先验及航向采样构造原始传感器位姿假设。
    VerificationEntry makePositionVerificationEntry(const CandidateVerificationInput& input,
                                                    int yaw_offset, double yaw_step_rad,
                                                    double yaw_step_deg) const;
    /// 执行单次配准并验收完整位姿可观性和位置先验一致性。
    VerificationRunResult evaluatePreparedEntry(const VerificationEntry& entry,
                                                Registration& corrector) const;
    /// 验收描述子候选并准备其先验位姿和目标点云租约。
    bool buildCandidateInput(const LoopCandidate& hit, CandidateVerificationInput* out);
    /// 按位置召回关键帧准备几何验证输入及航向来源。
    bool buildPositionCandidateInput(int prior_index, CandidateVerificationInput* out);
    /// 按指定偏航偏移序列展开描述子候选的验证假设。
    std::vector<CandidateVerificationInput> buildDescriptorBatchInputs(
        const std::vector<CandidateVerificationInput>& base_inputs,
        const std::vector<int>& offsets) const;
    /// 按指定偏航偏移序列展开位置先验候选的验证假设。
    std::vector<CandidateVerificationInput> buildPositionBatchInputs(
        const std::vector<CandidateVerificationInput>& base_inputs,
        const std::vector<int>& offsets) const;
    /// 在单个候选目标上顺序验证航向假设并保留最佳结果。
    CandidateVerificationResult runCandidateVerification(
        const CandidateVerificationInput& input) const;
    /// 将配置工作数限制在任务数和可用硬件并发数之内。
    std::size_t parallelWorkerCount(std::size_t task_count) const;
    /// 并行验证候选并按原始序号归约诊断及最佳结果。
    bool runCandidateSet(const std::vector<CandidateVerificationInput>& inputs);

    /// 首批航向假设全部失败后才执行回退扫描。
    bool runSeedThenFallbackSweep(const std::vector<CandidateVerificationInput>& seed_inputs,
                                  const std::vector<CandidateVerificationInput>& sweep_inputs,
                                  const char* seed_suffix, const char* sweep_suffix);
    /// 根据候选是否具有可信航向选择首批和回退验证批次。
    bool runSeedThenSweepBatches(const std::vector<CandidateVerificationInput>& base_inputs,
                                 const char* seed_suffix, const char* sweep_suffix);
    /// 召回位置邻近候选并执行位置先验约束下的航向验证。
    bool tryPositionPrior();
    /// 验证局部描述子召回候选并保持确定性顺序。
    bool tryLocalCandidates();
    /// 在预算内验证尚未尝试的全局描述子候选。
    bool tryGlobalFallback();
    /// 记录已接受恢复结果的来源、航向和质量诊断。
    void acceptBestRelocalization(const char* suffix) const;
    /// 将最佳候选转换为可提交的完整恢复结果。
    RelocalizationResult makeResult() const;

    const KeyframeMap& keyframes_;
    const PlaceDescriptor& descriptor_;
    RelocalizationTargetCache& target_cache_;
    const RelocalizationRequest& request_;
    const RelocalizationConfig& config_;
    const std::vector<Eigen::Vector3d>& prior_kf_positions_;

    std::string backend_name_ = "descriptor";
    bool initializing_ = false;
    bool can_global_supplement_ = false;
    int local_candidate_count_ = 1;
    int global_fallback_budget_ = 0;
    int target_keyframe_limit_ = 0;
    int global_fallback_considered_ = 0;
    int tried_verifications_ = 0;
    double best_descriptor_distance_ = std::numeric_limits<double>::max();
    double descriptor_distance_limit_ = std::numeric_limits<double>::max();
    double best_failed_residual_score_ = std::numeric_limits<double>::max();
    ObservabilityQuality best_failed_quality_;
    Cloud::Ptr level_query_cloud_;
    Cloud::ConstPtr source_ds_;
    std::vector<int> subset_;
    std::vector<LoopCandidate> candidates_;
    RelocalizationBest best_;
};

}  // namespace backend::relocalization_detail
