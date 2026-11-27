// 验证先验恢复、常规跟踪、回环及 UWB 融合的完整提交。
#include "localization_fixture.hpp"

namespace backend {
namespace {
using testing::LocalizationFlow;

class PriorLoopFlow : public LocalizationFlow, public ::testing::WithParamInterface<bool> {};

/// 真正提交回环（含超过 fixed-lag 的历史节点），并验证 UWB 接纳和重置后双库隔离。
TEST_P(PriorLoopFlow, PriorLoopAndOptionalUwbSurviveEpochResetWithIndependentStores) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.loop.enabled = true;
    config.loop.cooldown_keyframes = 2;
    config.loop.candidate_max_distance = 2;
    config.graph.fixed_lag_seconds = 1;
    config.uwb.enabled = GetParam();
    reset.available = true;
    start(true);
    const auto prior_revision = prior.revision();
    const auto descriptor_revision = prior_descriptor.revision();
    bool uwb_accepted = false;
    for (int index = 0; index < 6; ++index) {
        const double time = 10 + index;
        const auto pose = testing::scenePose(.01 * index, 0, 0, 0);
        if (GetParam()) {
            ASSERT_TRUE(controller->ingestUwbPosition(time, pose.block<3, 1>(0, 3)));
        }
        ASSERT_TRUE(ingest(frame(time, pose, pose)));
        maintain();
        ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
        uwb_accepted |= controller->snapshot().localization.reason.find("UWB NIS=") != std::string::npos;
    }
    ASSERT_FALSE(controller->visualization().loop_edges.empty());
    EXPECT_GT(controller->visualization().loop_edges.back().first, 3);
    EXPECT_EQ(trajectory.size(), 6U);
    EXPECT_EQ(descriptor.size(), trajectory.size());
    if (GetParam()) {
        EXPECT_TRUE(uwb_accepted);
    }
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 1);
    const auto pose = testing::scenePose(.06, 0, 0, 0);
    if (GetParam()) {
        ASSERT_TRUE(controller->ingestUwbPosition(16, pose.block<3, 1>(0, 3)));
    }
    ASSERT_TRUE(ingest(frame(16, pose, Eigen::Matrix4d::Identity(), 1)));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_EQ(trajectory.size(), 7U);
    EXPECT_EQ(descriptor.size(), trajectory.size());
    EXPECT_EQ(trajectory.latest().epoch, 1U);
    EXPECT_EQ(prior.size(), 1U);
    EXPECT_EQ(prior_descriptor.size(), 1U);
    EXPECT_EQ(prior.revision(), prior_revision);
    EXPECT_EQ(prior_descriptor.revision(), descriptor_revision);
}

INSTANTIATE_TEST_SUITE_P(WithAndWithoutUwb, PriorLoopFlow, ::testing::Bool());

TEST_F(LocalizationFlow, PriorAndLoopRejectSharedDescriptorStorage) {
    config.prior_map_enabled = true;
    config.loop.enabled = true;
    EXPECT_THROW((Localization(trajectory, prior, descriptor, descriptor, config, clock,
                               reset, states, visualization)), std::invalid_argument);
}

/// 先验 initialization_only 的对照场景：里程计相对真值持续漂移。
class PriorInitializationOnlyFlow : public LocalizationFlow,
                                    public ::testing::WithParamInterface<bool> {};

/// 一次性先验：首次匹配验收后 map->odom 冻结（定位全走前端里程计）；
/// 常规先验仍按跟踪周期持续把漂移拉回。
TEST_P(PriorInitializationOnlyFlow, PriorCorrectionsFollowTheConfiguredLatch) {
    const bool initialization_only = GetParam();
    const auto anchor = testing::scenePose(.2, -.1, .05, .05);
    config.startup_hint.T_map_sensor = anchor;
    config.startup_hint.yaw_rad = .05;
    config.prior_map_initialization_only = initialization_only;
    start(true);
    ASSERT_TRUE(ingest(frame(10, anchor, Eigen::Matrix4d::Identity())));
    EXPECT_EQ(controller->status(), LocalizationStatus::Initializing);
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto initial = output(frame(10, anchor, Eigen::Matrix4d::Identity()));
    ASSERT_TRUE(initial.valid);
    // 之后里程计比真值多走 0.03 m：常规跟踪会把 map->odom 拉回来；
    // initialization_only 则不再产生先验观测，map->odom 保持常量。
    bool changed = false;
    for (int index = 1; index <= 3; ++index) {
        const auto truth = anchor * testing::scenePose(.1 * index, 0, 0, 0);
        const auto odom = testing::scenePose(.1 * index + .03, 0, 0, 0);
        const auto next = frame(10 + index, truth, odom);
        ASSERT_TRUE(ingest(next));
        maintain();
        ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
        const auto current = output(next);
        ASSERT_TRUE(current.valid);
        changed = changed || (current.T_map_odom - initial.T_map_odom).norm() > 1e-4;
    }
    EXPECT_EQ(changed, !initialization_only);
}

INSTANTIATE_TEST_SUITE_P(PriorInitializationOnly, PriorInitializationOnlyFlow, ::testing::Bool());

/// 启动提示只表达车体航向；传感器相对车体的安装角（云台转角/倾斜）取当前端 odom 姿态。
/// 倾斜安装时若把提示的 level 旋转当传感器位姿，种子会差一整个安装角而匹配失败。
TEST_F(LocalizationFlow, StartupHintTakesSensorAttitudeFromLiveOdom) {
    Eigen::Matrix4d mount = Eigen::Matrix4d::Identity();
    mount.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(.5, Eigen::Vector3d::UnitX()).toRotationMatrix() *
        Eigen::AngleAxisd(.35, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix4d hint = testing::scenePose(0, 0, 0, .1);
    const Eigen::Matrix4d truth = hint * mount;
    config.startup_hint.T_map_sensor = hint;
    config.startup_hint.yaw_rad = .1;
    start(true);
    ASSERT_TRUE(ingest(frame(10, truth, mount)));
    EXPECT_EQ(controller->status(), LocalizationStatus::Initializing);
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_LT((output(frame(10, truth, mount)).T_map_sensor - truth).norm(), 5e-3);
}

/// 验证先验重定位、后续跟踪及输出版本更新，同一几何观测只提交一次。
TEST_F(LocalizationFlow, PriorRecoveryAndTrackingCommitGeometryOnce) {
    const auto anchor = testing::scenePose(.2, -.1, .05, .05);
    config.startup_hint.T_map_sensor = testing::scenePose(.15, -.1, .05, .05);
    config.startup_hint.yaw_rad = .05;
    start(true);
    const auto prior_revision = prior.revision();
    auto first = frame(10, anchor, Eigen::Matrix4d::Identity());
    ASSERT_TRUE(ingest(first));
    EXPECT_EQ(controller->status(), LocalizationStatus::Initializing);
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_LT((output(first).T_map_sensor - anchor).norm(), 2e-4);
    const Eigen::Matrix4d truth = anchor * testing::scenePose(.15, 0, 0, 0);
    auto next = frame(11, truth, testing::scenePose(.18, 0, 0, 0));
    ASSERT_TRUE(ingest(next));
    const auto prediction = output(next);
    maintain();
    const auto corrected = output(next);
    ASSERT_TRUE(corrected.valid);
    EXPECT_TRUE(corrected.pose_covariance_valid);
    EXPECT_LT((corrected.T_map_sensor - truth).norm(), .005);
    EXPECT_FALSE(controller->projectionIsCurrent(prediction));
    EXPECT_TRUE(controller->projectionIsCurrent(corrected));
    const auto covariance = corrected.pose_covariance;
    const auto revision = controller->snapshot().pose_revision;
    maintain();
    EXPECT_EQ(controller->snapshot().pose_revision, revision);
    EXPECT_LT((output(next).pose_covariance - covariance).norm(), 1e-12);
    EXPECT_EQ(prior.revision(), prior_revision);
}

/// 在恢复计算期间插入更新帧，检查恢复锚提交后按新帧运动传播输出。
TEST_F(LocalizationFlow, RecoveryPropagatesNewerFrameArrivingBeforeCommit) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.registration_debug_enabled = true;
    start(true);
    auto first = frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    auto next = frame(11, testing::scenePose(.2, 0, 0, 0), testing::scenePose(.2, 0, 0, 0));
    ASSERT_TRUE(ingest(first));
    bool received = false;
    visualization.before_commit = [&] { received = ingest(next); };
    maintain();
    ASSERT_TRUE(received);
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_DOUBLE_EQ(trajectory.latest().timestamp, first.timestamp);
    EXPECT_LT((output(next).T_map_sensor - next.odom_pose).norm(), 2e-4);
}

/// 手动重定位请求等待新恢复源，成功后保持先验地图内容与版本。
TEST_F(LocalizationFlow, LostManualRecoveryWaitsForNewDataAndKeepsPriorImmutable) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.recovery.suspect_failures_before_lost = 1;
    config.recovery.automatic_when_lost = false;
    start(true);
    auto first = frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    ASSERT_TRUE(ingest(first));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto prior_revision = prior.revision();
    auto sparse = frame(11, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    sparse.cloud_local->resize(1);
    ASSERT_TRUE(ingest(sparse, Eigen::Vector3d(10, 0, 0)));
    EXPECT_EQ(controller->status(), LocalizationStatus::Suspect);
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Lost);
    auto next = frame(12, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    ASSERT_TRUE(ingest(next));
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    ASSERT_TRUE(controller->requestManualRelocalization());
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    next.timestamp = 13;
    ASSERT_TRUE(ingest(next));
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto restored = output(next);
    ASSERT_TRUE(restored.valid);
    EXPECT_TRUE(restored.pose_trusted);
    EXPECT_TRUE(restored.pose_covariance_valid);
    EXPECT_LT((restored.T_map_sensor - Eigen::Matrix4d::Identity()).norm(), 2e-4);
    EXPECT_FALSE(controller->requestManualRelocalization());
    EXPECT_EQ(prior.revision(), prior_revision);
}

/// 执行真实回环检索、几何验证和图提交，检查后续检索遵守冷却间隔。
TEST_F(LocalizationFlow, ActualLoopRetrievalVerificationAndGraphCommitRespectCooldown) {
    config.loop.enabled = true;
    config.loop.cooldown_keyframes = 2;
    config.loop.history_size = 1;
    config.loop.candidate_max_distance = 2;
    start(false);
    for (int index = 0; index < 4; ++index) {
        ASSERT_TRUE(
            ingest(frame(10 + index, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
        maintain();
    }
    const auto edges = controller->visualization().loop_edges;
    ASSERT_EQ(edges.size(), 1U);
    EXPECT_EQ(edges.front().first, 2);
    EXPECT_LT(edges.front().second, edges.front().first);
    const auto revision = controller->snapshot().pose_revision;
    maintain();
    EXPECT_EQ(controller->snapshot().pose_revision, revision);
    const int accepted_index = edges.front().first;
    for (int index = 4; index < 7; ++index) {
        ASSERT_TRUE(
            ingest(frame(10 + index, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
        maintain();
    }
    ASSERT_EQ(controller->visualization().loop_edges.size(), 1U);
    EXPECT_GT(controller->visualization().loop_edges.front().first, accepted_index);
    EXPECT_EQ(controller->visualization().loop_edges.front().first, 6);
    EXPECT_EQ(descriptor.size(), trajectory.size());
}

/// 拒绝不一致回环后，原里程计轨迹和正常跟踪状态保持不变。
TEST_F(LocalizationFlow, InconsistentLoopCannotRewriteOdometryOrDegradeTrackingHealth) {
    config.loop.enabled = true;
    config.loop.source.max_frames = 1;
    config.graph.odom_noise.translation_variance_per_second = 1e-6;
    config.graph.odom_noise.rotation_variance_per_second = 1e-6;
    start(false);
    FrontendFrame current;
    for (int index = 0; index < 4; ++index) {
        current =
            frame(10 + index, Eigen::Matrix4d::Identity(), testing::scenePose(.1 * index, 0, 0, 0));
        ASSERT_TRUE(ingest(current));
        maintain();
    }
    LoopWork candidate_work;
    candidate_work.latest = trajectory.latest();
    candidate_work.trajectory = trajectory.snapshot();
    candidate_work.candidates = descriptor.detectLatestCandidates();
    candidate_work.descriptor_level_with_odom_roll_pitch = descriptor.levelWithOdomRollPitch();
    LoopMatcher matcher({config.loop, config.relocalization.point_to_plane_config});
    ASSERT_TRUE(matcher.compute(candidate_work).selected);
    EXPECT_TRUE(controller->visualization().loop_edges.empty());
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_LT((output(current).T_map_sensor - current.odom_pose).norm(), 1e-10);
}

/// LiDAR 局部模型失效时立即进入恢复，同周期通过的 UWB 观测不能抵消该事件。
TEST_F(LocalizationFlow, InvalidLidarModelCannotBeHiddenByConsistentUwbEvidence) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.uwb.enabled = true;
    start(true);
    ASSERT_TRUE(controller->ingestUwbPosition(10, Eigen::Vector3d::Zero()));
    ASSERT_TRUE(ingest(frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    auto invalid_geometry =
        frame(11, testing::scenePose(300, 0, 0, 0), testing::scenePose(.1, 0, 0, 0));
    ASSERT_TRUE(controller->ingestUwbPosition(11, Eigen::Vector3d(.1, 0, 0)));
    ASSERT_TRUE(ingest(invalid_geometry));
    const auto before = output(invalid_geometry);
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Suspect);
    const auto frozen = output(invalid_geometry);
    EXPECT_FALSE(frozen.pose_trusted);
    EXPECT_LT((frozen.T_map_sensor - before.T_map_sensor).norm(), 1e-10);
    EXPECT_FALSE(controller->projectionIsCurrent(before));
}

class PriorUwbCalibrationFlow : public LocalizationFlow, public ::testing::WithParamInterface<bool> {};

/// 有/无回环时，通过先验位姿联合标定原始 UWB 坐标并实际接纳因子。
TEST_P(PriorUwbCalibrationFlow, PriorTrackingCalibratesRawUwbCoordinatesFromIndependentAnchors) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.uwb.enabled = true;
    config.uwb.use_z = false;
    config.uwb.estimate_zero_pose_online = true;
    config.uwb.estimate_time_offset_online = true;
    config.loop.enabled = GetParam();
    start(true);
    const auto T_map_uwb = testing::scenePose(1, -2, 0, .25);
    FrontendFrame current;
    bool uwb_accepted = false;
    for (int index = 0; index < 14; ++index) {
        const auto truth = testing::scenePose(.35 * index, .15 * (index % 3), 0, 0);
        const Eigen::Vector3d raw = T_map_uwb.block<3, 3>(0, 0).transpose() *
                                    (truth.block<3, 1>(0, 3) - T_map_uwb.block<3, 1>(0, 3));
        ASSERT_TRUE(controller->ingestUwbPosition(10 + index, raw));
        current = frame(10 + index, truth, truth);
        ASSERT_TRUE(ingest(current));
        maintain();
        ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
        uwb_accepted |= controller->snapshot().localization.reason.find("UWB NIS=") != std::string::npos;
    }
    const auto position = controller->latestUwbBodyPositionMap();
    ASSERT_TRUE(position);
    EXPECT_LT((position->head<2>() - current.odom_pose.block<2, 1>(0, 3)).norm(), .03);
    EXPECT_LT((output(current).T_map_sensor - current.odom_pose).norm(), .01);
    EXPECT_TRUE(uwb_accepted);
    if (GetParam()) {
        EXPECT_FALSE(controller->visualization().loop_edges.empty());
    }
}

INSTANTIATE_TEST_SUITE_P(WithAndWithoutLoop, PriorUwbCalibrationFlow, ::testing::Bool());

/// UWB 启动对齐拒绝离群样本后，继续使用新观测建立可信定位。
TEST_F(LocalizationFlow, UwbBootstrapRejectsAnOutlierOnceThenAcceptsFreshEvidence) {
    config.innovation.conflicts_before_suspect = 2;
    config.uwb.enabled = true;
    config.uwb.use_z = false;
    config.uwb.antenna_position_sensor = Eigen::Vector3d(.3, .1, 0);
    start(false);
    const auto alignment = testing::scenePose(3, -2, 0, .3);
    FrontendFrame current;
    for (int index = 0; index < 12; ++index) {
        const auto odom = testing::scenePose(.5 * index, .15 * (index % 3), 0, 0);
        const Eigen::Matrix4d truth = alignment * odom;
        const Eigen::Vector3d antenna =
            truth.block<3, 3>(0, 0) * config.uwb.antenna_position_sensor + truth.block<3, 1>(0, 3);
        ASSERT_TRUE(controller->ingestUwbPosition(10 + index, antenna));
        current = frame(10 + index, truth, odom);
        ASSERT_TRUE(ingest(current));
        maintain();
    }
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_LT((output(current).T_map_sensor - alignment * current.odom_pose).norm(), 1e-4);
    current.timestamp = 22;
    ASSERT_TRUE(controller->ingestUwbPosition(22, Eigen::Vector3d(100, 100, 0)));
    ASSERT_TRUE(ingest(current));
    const auto before = output(current);
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::ObsDegraded);
    EXPECT_LT((output(current).T_map_sensor - before.T_map_sensor).norm(), 1e-10);
    const auto generation = controller->snapshot().localization.generation;
    maintain();
    EXPECT_EQ(controller->snapshot().localization.generation, generation);
    current.timestamp = 23;
    const Eigen::Matrix4d truth = alignment * current.odom_pose;
    const Eigen::Vector3d antenna =
        truth.block<3, 3>(0, 0) * config.uwb.antenna_position_sensor + truth.block<3, 1>(0, 3);
    ASSERT_TRUE(controller->ingestUwbPosition(23, antenna));
    ASSERT_TRUE(ingest(current));
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_TRUE(controller->latestUwbBodyPositionMap());
}

/// 累计修正预算：逐步 1.5° 的错解（每一步都过得了 NIS 创新门与单次 15° 上限）
/// 不能一路把 map->odom 转过去。判据见 backend/correction_anchor.hpp。
///
/// 步长取 1.5° 是有讲究的：
/// - 5° 的注入会先被 NIS 创新门拒掉（实测 NIS=80.9），根本走不到累计闸；
/// - 1.5° 能过 NIS，又高于"安静"阈值 1°（所以锚点不会被追认），正好隔离出累计闸的作用。
TEST_F(LocalizationFlow, CumulativeAnchorStopsRepeatedSmallRotations) {
    constexpr double kDeg = M_PI / 180.0;
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.correction_anchor.enabled = true;
    config.correction_anchor.max_rotation_rad = 8.0 * kDeg;
    config.correction_anchor.max_translation = 1.0;
    config.correction_anchor.reseat_rotation_rad = 1.0 * kDeg;
    config.correction_anchor.reseat_translation = .15;
    config.correction_anchor.reseat_quiet_frames = 5;
    // 把 SUSPECT 推远：本用例只看"位姿有没有被一路转过去"，不看状态机。
    config.innovation.conflicts_before_suspect = 1000;
    start(true);

    double time = 10;
    ASSERT_TRUE(ingest(frame(time, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);

    // odom 直行且不转；"扫描落在的位姿"每一步多转 1.5°。不加闸时 10 步就是 15°。
    double max_yaw_deg = 0.0;
    for (int index = 1; index <= 10; ++index) {
        SCOPED_TRACE(index);
        time += .5;
        auto odom = testing::scenePose(.1 * index, 0, 0, 0);
        auto truth = testing::scenePose(.1 * index, 0, 0, 1.5 * kDeg * index);
        auto input = frame(time, truth, odom);
        ASSERT_TRUE(ingest(input));
        maintain();
        const auto projection = output(input);
        const double yaw =
            std::atan2(projection.T_map_odom(1, 0), projection.T_map_odom(0, 0)) / kDeg;
        max_yaw_deg = std::max(max_yaw_deg, std::fabs(yaw));
    }
    // 下界：证明前几步确实被接受并转过（否则这个用例什么也没验证）；
    // 上界：8° 预算 + 图优化余量，远小于不设闸时的 15°。
    EXPECT_GT(max_yaw_deg, 3.0);
    EXPECT_LT(max_yaw_deg, 11.0);
}

/// 稳定偏差（真实漂移）不能因为超过累计预算就永久冻结：连续够多帧一致后放行一次。
///
/// 现场依据（real_zloc_1127_193946）：47.3 s 起连续 7 次先验观测都报
/// delta_t=1.17~1.28 m / delta_r=3.4~3.7°，全被 0.8 m 的锚点拒；7 连冲突 →
/// SUSPECT → LOST，之后 70 s 内跟踪与恢复都被同一道闸拒，0 次提交，
/// 包尾点云相对先验图歪 1.8 m / ≥9.5°。
///
/// 这里用**旋转轴**复现同一判据（合成场景的配准对纯平移不敏感：源点云带着
/// 预测位姿初始化，ICP 一次迭代就"收敛"回预测，实测 delta_t 恒为 0；
/// 旋转轴的修正则确实会进观测，见 CumulativeAnchorStopsRepeatedSmallRotations）。
/// 平移轴的同一条判据由单测 ConsistentDriftReleasesAfterEnoughAgreeingFrames 覆盖。
TEST_F(LocalizationFlow, ConsistentOffsetReleasesTheAnchorInsteadOfFreezing) {
    constexpr double kDeg = M_PI / 180.0;
    constexpr double kOffsetDeg = 9.5;  // > 8° 预算，< 10° 放行半径
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.correction_anchor.enabled = true;
    config.correction_anchor.max_rotation_rad = 8.0 * kDeg;
    config.correction_anchor.max_translation = 0.8;
    config.correction_anchor.reseat_rotation_rad = 1.0 * kDeg;
    config.correction_anchor.reseat_translation = .15;
    config.correction_anchor.reseat_quiet_frames = 5;
    config.correction_anchor.release_enabled = true;
    config.correction_anchor.release_frames = 5;
    config.correction_anchor.release_agree_translation = 0.12;
    config.correction_anchor.release_agree_rotation_rad = 1.0 * kDeg;
    config.correction_anchor.release_max_translation = 1.6;
    config.correction_anchor.release_max_rotation_rad = 10.0 * kDeg;
    // 只看"位姿有没有被拉过去"，不看状态机。
    config.innovation.conflicts_before_suspect = 1000;
    start(true);

    double time = 10;
    ASSERT_TRUE(ingest(frame(time, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);

    // odom 正常前进，但"扫描落在的位姿"始终比 odom 多 9.5°：这就是稳定漂移。
    double released_yaw_deg = 0.0;
    bool saw_conflict = false;
    bool saw_release = false;
    for (int index = 1; index <= 6; ++index) {
        SCOPED_TRACE(index);
        time += .5;
        auto odom = testing::scenePose(.1 * index, 0, 0, 0);
        auto truth = testing::scenePose(.1 * index, 0, 0, kOffsetDeg * kDeg);
        auto input = frame(time, truth, odom);
        ASSERT_TRUE(ingest(input));
        maintain();
        const auto projection = output(input);
        ASSERT_TRUE(projection.valid);
        const double yaw_deg =
            std::atan2(projection.T_map_odom(1, 0), projection.T_map_odom(0, 0)) / kDeg;
        const std::string& reason = controller->snapshot().localization.reason;
        if (index < 5) {
            // 前 4 帧：越界被拒，但位姿不能被搬走。
            saw_conflict |= reason.find("over cumulative correction anchor") != std::string::npos;
            EXPECT_LT(std::fabs(yaw_deg), 3.0) << "第 " << index << " 帧不该放行";
        } else {
            saw_release |= reason.find("accepted by consistent-evidence release") !=
                           std::string::npos;
            released_yaw_deg = yaw_deg;
        }
        // 越界冲突会让状态进 OBS_DEGRADED（正常），但不能升级到 SUSPECT/LOST ——
        // 那才是"永久冻结"的入口。
        EXPECT_NE(controller->status(), LocalizationStatus::Suspect);
        EXPECT_NE(controller->status(), LocalizationStatus::Lost);
    }
    EXPECT_TRUE(saw_conflict);
    EXPECT_TRUE(saw_release) << "第 5 帧必须放行，否则就是永久冻结（现场故障）";
    // 放行后 map→odom 必须被拉到测量值附近（9.5°），而不是原地不动。
    EXPECT_NEAR(released_yaw_deg, kOffsetDeg, 2.0);
}

/// 幅度超过放行半径的偏差仍必须被拒：同构走廊的稳定错解不能靠"一致"混进来。
TEST_F(LocalizationFlow, OffsetBeyondReleaseCapStaysRejected) {
    constexpr double kDeg = M_PI / 180.0;
    constexpr double kOffsetDeg = 14.0;  // > 10° 放行半径
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.correction_anchor.enabled = true;
    config.correction_anchor.max_rotation_rad = 8.0 * kDeg;
    config.correction_anchor.max_translation = 0.8;
    config.correction_anchor.release_enabled = true;
    config.correction_anchor.release_frames = 5;
    config.correction_anchor.release_agree_translation = 0.12;
    config.correction_anchor.release_agree_rotation_rad = 1.0 * kDeg;
    config.correction_anchor.release_max_translation = 1.6;
    config.correction_anchor.release_max_rotation_rad = 10.0 * kDeg;
    config.innovation.conflicts_before_suspect = 1000;
    start(true);

    ASSERT_TRUE(ingest(frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);

    double max_yaw_deg = 0.0;
    for (int index = 1; index <= 10; ++index) {
        SCOPED_TRACE(index);
        const double time = 10 + .5 * index;
        const auto odom = testing::scenePose(.1 * index, 0, 0, 0);
        const auto truth = testing::scenePose(.1 * index, 0, 0, kOffsetDeg * kDeg);
        auto input = frame(time, truth, odom);
        ASSERT_TRUE(ingest(input));
        maintain();
        const auto projection = output(input);
        ASSERT_TRUE(projection.valid);
        max_yaw_deg = std::max(
            max_yaw_deg,
            std::fabs(std::atan2(projection.T_map_odom(1, 0), projection.T_map_odom(0, 0)) / kDeg));
    }
    // 不管一致多少帧，都不许把车搬过放行半径。
    EXPECT_LT(max_yaw_deg, 12.0);
}

/// 快速自转期间必须**跳过**先验观测，而不是让它冲突到冻结。
/// 判据：整段自转状态保持 NORMAL，且 map→odom 不被改动。
TEST_F(LocalizationFlow, FastBodySpinSkipsPriorObservationsInsteadOfConflicting) {
    constexpr double kDeg = M_PI / 180.0;
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.tracking.max_body_angular_rate_rad = 60.0 * kDeg;  // 实车 profile 的阈值
    start(true);

    double time = 10.0;
    ASSERT_TRUE(ingest(frame(time, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto baseline = output(frame(time, Eigen::Matrix4d::Identity(),
                                       Eigen::Matrix4d::Identity())).T_map_odom;

    // 每 0.1 s 转 30° = 300°/s，远高于 60°/s 的闸门；跑 20 帧（2 s，约 600°）。
    for (int index = 1; index <= 20; ++index) {
        SCOPED_TRACE(index);
        time += 0.1;
        const auto pose = testing::scenePose(0, 0, 0, 30.0 * kDeg * index);
        auto input = frame(time, pose, pose);
        ASSERT_TRUE(ingest(input));
        maintain();
        // 跳过观测不会推进状态机：全程不许进 SUSPECT/LOST。
        EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    }
    // 没有观测被提交，map->odom 必须和自转前一模一样。
    const auto after = output(frame(time, testing::scenePose(0, 0, 0, 30.0 * kDeg * 20),
                                    testing::scenePose(0, 0, 0, 30.0 * kDeg * 20))).T_map_odom;
    EXPECT_LT((after - baseline).norm(), 1e-9);
}

/// 云台反向扫动：0.1 s 一帧、在 +20°/-20° 之间来回。首尾净转角经常接近 0，
/// 但 0.5 s 内的路径长度始终约 400°/s。旧实现（0.1 s 净转角）会在反向窗口放行
/// 观测并冲突；2026-11-27 实车 bag 就是这样把状态推到 SUSPECT/LOST 的。
TEST_F(LocalizationFlow, BodySpinGateUsesPathLengthNotNetRotation) {
    constexpr double kDeg = M_PI / 180.0;
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.tracking.max_body_angular_rate_rad = 60.0 * kDeg;  // 实车 profile 的阈值
    start(true);

    double time = 10.0;
    ASSERT_TRUE(ingest(frame(time, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto baseline = output(frame(time, Eigen::Matrix4d::Identity(),
                                       Eigen::Matrix4d::Identity())).T_map_odom;

    for (int index = 1; index <= 40; ++index) {
        SCOPED_TRACE(index);
        time += 0.1;
        const double yaw = (index % 2 == 0 ? 20.0 : -20.0) * kDeg;
        const auto pose = testing::scenePose(0, 0, 0, yaw);
        ASSERT_TRUE(ingest(frame(time, pose, pose)));
        maintain();
        // 闸门必须全程跳过观测：不许进 SUSPECT/LOST，也不许改 map->odom。
        EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    }
    const auto after = output(frame(time, testing::scenePose(0, 0, 0, 20.0 * kDeg),
                                    testing::scenePose(0, 0, 0, 20.0 * kDeg))).T_map_odom;
    EXPECT_LT((after - baseline).norm(), 1e-9);
}
}  // namespace
}  // namespace backend
