// 验证输出投影与协方差锚在里程计换代后的物理一致性。
#include <gtest/gtest.h>

#include "backend/covariance.hpp"
#include "backend/math.hpp"
#include "backend/pose_graph.hpp"
#include "backend/pose_output.hpp"
#include "scene.hpp"

namespace backend {
namespace {
/// 图被清空后保留的物理协方差锚，在更换里程计坐标系前后给出相同传播结果。
TEST(PoseOutput, ReframingPreservesPhysicalProjectionAndRetainedGraphUncertainty) {
    PoseGraphConfig config;
    PoseGraph graph(config);
    const auto map_from_old = testing::scenePose(3, -2, .4, .6);
    const auto old_pose = testing::scenePose(1, .5, .2, -.2);
    Keyframe anchor;
    anchor.index = 0;
    anchor.timestamp = 10;
    anchor.odom_pose = old_pose;
    anchor.optimized_pose = map_from_old * old_pose;
    graph.addPrior(0, anchor.optimized_pose, 10);
    graph.update(false);
    PoseOutput poses(config.odom_noise);
    poses.initialize();
    poses.setTransform(map_from_old);
    poses.refreshAnchor(anchor, graph);
    LocalizationStateSnapshot state;
    state.status = LocalizationStatus::Normal;
    const auto current_old = testing::scenePose(1.7, .8, .3, .15);
    const auto before =
        poses.projectForOutput(current_old, PoseCovariance6d::Identity(), true, 10.8, 0, state, 0);
    ASSERT_TRUE(before.pose_covariance_valid);

    graph.reset(config);
    const auto old_from_new = testing::scenePose(1.2, .6, .2, .3);
    poses.reframeOdometry(old_from_new, 0, 1);
    poses.resetEpoch(1);
    const auto after =
        poses.projectForOutput(old_from_new.inverse() * current_old, PoseCovariance6d::Identity(),
                               true, 10.8, 1, state, 1);
    ASSERT_TRUE(after.pose_covariance_valid);
    EXPECT_LT((after.T_map_sensor - before.T_map_sensor).norm(), 1e-12);
    EXPECT_LT((after.pose_covariance - before.pose_covariance).norm(), 1e-12);
}

/// 无可连接图节点时，停车参考仍独立保存全局协方差并按真实后续运动传播。
TEST(PoseOutput, ParkedCovarianceAnchorSurvivesWarmBoundariesWithoutGraphNodes) {
    OdometryProcessNoiseConfig noise;
    PoseOutput poses(noise);
    poses.initialize();
    const auto map_from_parked = testing::scenePose(3, -2, .4, .6);
    poses.setTransform(map_from_parked);
    FrontendFrame parked;
    parked.epoch = 1;
    parked.timestamp = 10;
    parked.odom_pose = testing::scenePose(.2, -.3, .1, .25);
    PoseCovariance6d root = PoseCovariance6d::Identity();
    root(0, 4) = .2;
    root(1, 5) = -.3;
    const PoseCovariance6d initial_covariance = .04 * root * root.transpose();
    poses.anchorCovariance(parked, initial_covariance);
    const auto first_boundary = testing::scenePose(.3, .1, 0, .4);
    const auto second_boundary = testing::scenePose(.1, -.2, 0, -.3);
    poses.reframeOdometry(first_boundary, 1, 2);
    poses.reframeOdometry(second_boundary, 2, 3);
    FrontendFrame current;
    current.epoch = 3;
    current.timestamp = 11.4;
    current.odom_pose = testing::scenePose(.7, -.2, .2, .2);
    current.pose_covariance = PoseCovariance6d::Identity() * 1e-4;
    current.pose_covariance_valid = true;
    const Eigen::Matrix4d motion =
        parked.odom_pose.inverse() * first_boundary * second_boundary * current.odom_pose;
    const Eigen::Matrix4d reference = map_from_parked * parked.odom_pose;
    const auto expected = propagatePoseWithRelativeCovariance(
        reference, initial_covariance, reference * motion,
        noise.covariance(current.timestamp - parked.timestamp, motion.block<3, 1>(0, 3).norm(),
                         rotationAngleFromMatrix(motion)));
    const auto actual = poses.covarianceAt(current);
    ASSERT_TRUE(actual);
    EXPECT_LT((*actual - expected).norm(), 1e-12);
    EXPECT_GT(actual->trace(), initial_covariance.trace());
}
}  // namespace
}  // namespace backend
