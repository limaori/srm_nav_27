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

/// 验证先验重定位、后续跟踪及输出版本更新，同一几何观测只提交一次。
TEST_F(LocalizationFlow, PriorRecoveryAndTrackingCommitGeometryOnce) {
    const auto anchor = testing::scenePose(.2, -.1, .05, .05);
    config.startup_hint.T_map_sensor = testing::scenePose(.15, -.1, .05, .05);
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
}  // namespace
}  // namespace backend
