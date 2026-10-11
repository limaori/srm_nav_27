#!/usr/bin/env python3
"""只读真值评估器（方案 §8）：按独立真值判定每轮初始化的成功与精度。

输入（只读，不发布任何 TF，不参与配准）：

* ``chassis_odometry_gt``：世界系真值 ``T_W_B``；
* ``/tf``：被测链路的 ``map → odom`` 与 ``odom → base_link``；
* ``gicp_diagnostics``：GICP 逐次诊断 JSON（方案 §9）。

输出：

* ``gicp_eval/status``：当前轮次的实时状态（JSON）；
* ``<output_dir>/<run_id>/iterations.csv``：逐次配准诊断 + 匹配后的真值误差；
* ``<output_dir>/<run_id>/summary.json`` / ``summary.csv``：本轮判定结果。

时间口径（方案 §8.1）：以扫描时间戳为基准取样真值与 TF，时间差超过
``max_time_offset`` 的样本标记为无效；GICP 的 TF 时间戳是「扫描时间 +0.1 s」，
评估不使用该未来时间戳，而是直接用扫描时间戳查 ``odom → base_link``。
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gicp_validation_common import (  # noqa: E402
    angle_diff,
    invert,
    make_transform,
    matrix_to_quaternion,
    quaternion_to_matrix,
    transform_from_pose,
    yaw_from_matrix,
)

CSV_FIELDS = [
    "seq",
    "event",
    "sim_time",
    "wall_time",
    "scan_stamp",
    "scan_age",
    "raw_points",
    "source_points",
    "converged",
    "iterations",
    "accepted",
    "branch",
    "reason",
    "inlier_ratio",
    "rmse",
    "dpos",
    "dangle",
    "duration_ms",
    "est_x",
    "est_y",
    "est_yaw",
    "gt_x",
    "gt_y",
    "gt_yaw",
    "e_xy",
    "e_yaw",
    "e_z",
    "time_offset",
    "valid",
    "suspicious",
    "init_x",
    "init_y",
    "init_yaw",
]

SUMMARY_FIELDS = [
    "run_id",
    "case_id",
    "category",
    "attempt_index",
    "passed",
    "failure_reason",
    "t_first_accepted_wall",
    "t_first_within_tolerance_wall",
    "t_stable_end_wall",
    "init_duration_wall",
    "init_duration_sim",
    "accepted_count",
    "valid_samples",
    "e_xy_median",
    "e_xy_p95",
    "e_xy_max",
    "e_yaw_median",
    "e_yaw_p95",
    "e_yaw_max",
    "suspect_count",
    "max_jump_xy",
    "max_jump_yaw",
    "init_error_xy",
    "init_error_yaw",
]


class TruthEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("gicp_truth_evaluator")

        self.declare_parameter("gt_topic", "chassis_odometry_gt")
        self.declare_parameter("diagnostic_topic", "gicp_diagnostics")
        self.declare_parameter("initialpose_topic", "initialpose")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("output_dir", "/tmp/gicp_validation/results")
        self.declare_parameter("run_id", "")
        self.declare_parameter("case_id", "")
        self.declare_parameter("category", "")
        self.declare_parameter("attempt_index", 0)
        self.declare_parameter("spawn_x", 3.4)
        self.declare_parameter("spawn_y", 9.5)
        self.declare_parameter("spawn_yaw", 0.0)
        self.declare_parameter("tolerance_xy", 0.15)
        self.declare_parameter("tolerance_yaw_deg", 3.0)
        self.declare_parameter("stable_window", 3.0)
        self.declare_parameter("min_accepted_in_window", 6)
        self.declare_parameter("max_jump_xy", 0.20)
        self.declare_parameter("max_jump_yaw_deg", 5.0)
        # 仿真真值 chassis_odometry_gt 约 10 Hz，扫描约 15-20 Hz；0.05 s 会把一半
        # 样本判成无效，因此默认放宽到 0.08 s（方案 §8.1 要求「超过 50 ms 标记无效」
        # 是按真值可按扫描时间戳插值的前提；本工程真值是零阶保持，故记录实际 offset）。
        self.declare_parameter("max_time_offset", 0.08)
        self.declare_parameter("suspect_xy", 0.5)
        self.declare_parameter("suspect_yaw_deg", 10.0)
        self.declare_parameter("observation_timeout", 10.0)
        self.declare_parameter("post_success_observe", 0.0)
        self.declare_parameter("buffer_seconds", 60.0)
        self.declare_parameter("task_timeout", 120.0)
        self.declare_parameter("write_csv", True)
        self.declare_parameter("spawn_tolerance", 0.05)

        self.t_map_world = invert(
            transform_from_pose(
                float(self.get_parameter("spawn_x").value),
                float(self.get_parameter("spawn_y").value),
                0.0,
                float(self.get_parameter("spawn_yaw").value),
            )
        )
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.output_dir = self.get_parameter("output_dir").value
        self.run_id = self.get_parameter("run_id").value or datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )
        self.case_id = self.get_parameter("case_id").value
        self.category = self.get_parameter("category").value
        self.attempt_index = int(self.get_parameter("attempt_index").value)
        self.tolerance_xy = float(self.get_parameter("tolerance_xy").value)
        self.tolerance_yaw = math.radians(float(self.get_parameter("tolerance_yaw_deg").value))
        self.stable_window = float(self.get_parameter("stable_window").value)
        self.min_accepted_in_window = int(
            self.get_parameter("min_accepted_in_window").value
        )
        self.max_jump_xy = float(self.get_parameter("max_jump_xy").value)
        self.max_jump_yaw = math.radians(float(self.get_parameter("max_jump_yaw_deg").value))
        self.max_time_offset = float(self.get_parameter("max_time_offset").value)
        self.suspect_xy = float(self.get_parameter("suspect_xy").value)
        self.suspect_yaw = math.radians(float(self.get_parameter("suspect_yaw_deg").value))
        self.observation_timeout = float(self.get_parameter("observation_timeout").value)
        self.post_success_observe = float(self.get_parameter("post_success_observe").value)
        self.buffer_seconds = float(self.get_parameter("buffer_seconds").value)
        self.task_timeout = float(self.get_parameter("task_timeout").value)
        self.write_csv = bool(self.get_parameter("write_csv").value)
        self.spawn_tolerance = float(self.get_parameter("spawn_tolerance").value)

        self.gt_samples = deque()
        self.gt_checked = False
        self.odom_base_samples = deque()
        self.current_map_odom = None
        self.current_map_odom_wall = None
        self.current_map_odom_sim = None

        self.active = False
        self.round_start_wall = None
        self.round_start_sim = None
        self.t0_wall = None
        self.t0_sim = None
        self.t0_ready = False  # 初值已应用且其后首帧有效点云到达
        self.first_accepted_wall = None
        self.first_accepted_sim = None
        self.first_within_tolerance_wall = None
        self.stable_end_wall = None
        self.stable_end_sim = None
        self.success = False
        self.failure_reason = ""
        self.accepted_in_window = []
        self.accepted_records = []
        self.iteration_rows = []
        self.suspicious_records = []
        self.applied_init = None
        self.init_msg_wall = None
        self.init_sent_wall = None
        self.init_sent_sim = None
        self.observed_jump_xy = 0.0
        self.observed_jump_yaw = 0.0
        self.last_accepted_pose = None
        self.diag_seen = 0
        self.accepted_diag_seen = 0
        self.tf_conflict_frames = {}

        self.csv_path = None
        self.csv_file = None
        self.csv_writer = None
        self.summary_path = None

        self.gt_sub = self.create_subscription(
            Odometry, self.get_parameter("gt_topic").value, self.on_gt, 50
        )
        self.diag_sub = self.create_subscription(
            String, self.get_parameter("diagnostic_topic").value, self.on_diagnostic, 200
        )
        self.init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("initialpose_topic").value,
            self.on_initialpose,
            10,
        )
        self.tf_sub = self.create_subscription(
            TFMessage, self.get_parameter("tf_topic").value, self.on_tf, 200
        )

        self.status_pub = self.create_publisher(String, "gicp_eval/status", 20)
        self.start_srv = self.create_service(Trigger, "gicp_eval/start_run", self.on_start_run)
        self.case_sub = self.create_subscription(
            StringMsg, "gicp_eval/case", self.on_case_message, 10
        )
        self.stop_srv = self.create_service(Trigger, "gicp_eval/stop_run", self.on_stop_run)
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            "[evaluator] 就绪：run_id=%s case=%s 容差 %.2f m / %.1f° 稳定窗 %.1f s "
            "T_M_W 平移=%s"
            % (
                self.run_id,
                self.case_id or "<unset>",
                self.tolerance_xy,
                math.degrees(self.tolerance_yaw),
                self.stable_window,
                np.round(self.t_map_world[:3, 3], 4).tolist(),
            )
        )

    # ------------------------------------------------------------------ 订阅

    def on_gt(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        stamp = msg.header.stamp
        transform = make_transform(
            [pose.position.x, pose.position.y, pose.position.z],
            quaternion_to_matrix(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        sim_time = stamp.sec + stamp.nanosec * 1e-9
        self.gt_samples.append((sim_time, transform))
        self.prune(self.gt_samples, sim_time)

        if not self.gt_checked:
            self.gt_checked = True
            expected = invert(self.t_map_world)
            error_xy = float(np.linalg.norm(transform[:2, 3] - expected[:2, 3]))
            error_yaw = angle_diff(yaw_from_matrix(transform[:3, :3]), yaw_from_matrix(expected[:3, :3]))
            self.get_logger().info(
                "[evaluator] 真值起点检查：gt=(%.3f, %.3f, yaw=%.4f) 期望=(%.3f, %.3f, yaw=%.4f) "
                "误差=%.4f m / %.3f°"
                % (
                    transform[0, 3],
                    transform[1, 3],
                    yaw_from_matrix(transform[:3, :3]),
                    expected[0, 3],
                    expected[1, 3],
                    yaw_from_matrix(expected[:3, :3]),
                    error_xy,
                    math.degrees(error_yaw),
                )
            )
            if error_xy > self.spawn_tolerance or error_yaw > math.radians(2.0):
                self.get_logger().error(
                    "[evaluator] 真值起点与配置的出生点不一致：T_W_B 语义或地图原点需要重新确认"
                    "（方案 §4.1），本轮评估结果不可直接采信。"
                )

    def on_tf(self, msg: TFMessage) -> None:
        now_wall = time.monotonic()
        sim_time = self.get_clock().now().nanoseconds * 1e-9
        for transform_stamped in msg.transforms:
            parent = transform_stamped.header.frame_id.lstrip("/")
            child = transform_stamped.child_frame_id.lstrip("/")
            stamp = (
                transform_stamped.header.stamp.sec
                + transform_stamped.header.stamp.nanosec * 1e-9
            )
            matrix = make_transform(
                [
                    transform_stamped.transform.translation.x,
                    transform_stamped.transform.translation.y,
                    transform_stamped.transform.translation.z,
                ],
                quaternion_to_matrix(
                    transform_stamped.transform.rotation.x,
                    transform_stamped.transform.rotation.y,
                    transform_stamped.transform.rotation.z,
                    transform_stamped.transform.rotation.w,
                ),
            )
            if parent == self.map_frame and child == self.odom_frame:
                self.current_map_odom = matrix
                self.current_map_odom_wall = now_wall
                self.current_map_odom_sim = stamp
            elif parent == self.odom_frame and child == self.base_frame:
                self.odom_base_samples.append((stamp, matrix, now_wall))
                self.prune_stamped(self.odom_base_samples, sim_time)

    def on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        transform = make_transform(
            [pose.position.x, pose.position.y, pose.position.z],
            quaternion_to_matrix(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        self.applied_init = transform
        self.init_msg_wall = time.monotonic()
        self.get_logger().info(
            "[evaluator] 记录初值消息：map→%s 猜测 (%.3f, %.3f, yaw=%.4f)"
            % (
                self.base_frame,
                transform[0, 3],
                transform[1, 3],
                yaw_from_matrix(transform[:3, :3]),
            )
        )

    # ------------------------------------------------------------------ 工具

    def prune(self, buffer, sim_time: float) -> None:
        while buffer and sim_time - buffer[0][0] > self.buffer_seconds:
            buffer.popleft()

    def prune_stamped(self, buffer, sim_time: float) -> None:
        while buffer and sim_time - buffer[0][0] > self.buffer_seconds:
            buffer.popleft()

    @staticmethod
    def lookup(buffer, sim_time: float, max_offset: float):
        if not buffer:
            return None, None
        best = None
        best_offset = None
        for sample in buffer:
            offset = abs(sample[0] - sim_time)
            if best_offset is None or offset < best_offset:
                best = sample
                best_offset = offset
        if best_offset is None or best_offset > max_offset:
            return None, best_offset
        return best, best_offset

    def truth_at(self, sim_time: float):
        """返回 (T_M_B_gt, time_offset)；T_M_B_gt = T_M_W · T_W_B。"""
        sample, offset = self.lookup(self.gt_samples, sim_time, self.max_time_offset)
        if sample is None:
            return None, offset
        return self.t_map_world @ sample[1], offset

    def estimate_at(self, sim_time: float):
        """返回 (T_M_B_est, time_offset)；T_M_B_est = T_M_O_est · T_O_B。"""
        if self.current_map_odom is None:
            return None, None
        sample, offset = self.lookup(self.odom_base_samples, sim_time, self.max_time_offset)
        if sample is None:
            return None, offset
        return self.current_map_odom @ sample[1], offset

    # ------------------------------------------------------------------ 诊断

    def on_diagnostic(self, msg: String) -> None:
        try:
            record = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("[evaluator] 无法解析诊断 JSON，已跳过")
            return

        self.diag_seen += 1
        event = record.get("event", "registration")
        scan_stamp_ns = int(record.get("scan_stamp_ns", 0))
        sim_time = scan_stamp_ns * 1e-9 if scan_stamp_ns else self.get_clock().now().nanoseconds * 1e-9

        if event == "initial_pose_applied":
            # 初值应用事件：t0 需要等到其后首帧有效点云进入配准时刻
            self.t0_ready = False
            self.init_sent_wall = record.get("_sent_wall", None)
            self.init_sent_sim = record.get("_sent_sim", None)
            self.t0_wall = record.get("_sent_wall", time.monotonic())
            self.t0_sim = record.get("_sent_sim", sim_time)
            self.first_accepted_wall = None
            self.first_accepted_sim = None
            self.first_within_tolerance_wall = None
            self.stable_end_wall = None
            self.stable_end_sim = None
            self.success = False
            self.failure_reason = ""
            self.accepted_in_window = []
            self.accepted_records = []
            self.suspicious_records = []
            self.observed_jump_xy = 0.0
            self.observed_jump_yaw = 0.0
            self.last_accepted_pose = None
            self.accepted_diag_seen = 0
            self.get_logger().info(
                "[evaluator] 初值事件：t0 起点 sim=%.3f（等待其后首帧有效点云）"
                % (self.t0_sim if self.t0_sim is not None else -1.0)
            )
            self.write_row(record, sim_time, None, None)
            return

        truth, gt_offset = self.truth_at(sim_time)
        estimate, tf_offset = self.estimate_at(sim_time)
        valid = truth is not None and estimate is not None
        offset = None
        if gt_offset is not None and tf_offset is not None:
            offset = max(gt_offset, tf_offset)
        elif gt_offset is not None:
            offset = gt_offset
        else:
            offset = tf_offset

        e_xy = e_yaw = e_z = None
        if valid:
            e_xy = float(np.linalg.norm(estimate[:2, 3] - truth[:2, 3]))
            e_yaw = angle_diff(
                yaw_from_matrix(estimate[:3, :3]), yaw_from_matrix(truth[:3, :3])
            )
            e_z = float(abs(estimate[2, 3] - truth[2, 3]))

        accepted = bool(record.get("accepted", False))
        source_points = int(record.get("source_points", 0))
        has_new_scan = bool(record.get("has_scan", False)) and source_points > 0

        if accepted and has_new_scan:
            self.accepted_diag_seen += 1
            if not self.t0_ready:
                # 方案 §6.4：t0 = 初值被应用后首帧有效点云到达（开始配准）的时刻
                self.t0_ready = True
                self.t0_sim = sim_time
                if self.init_msg_wall is not None:
                    self.t0_wall = self.init_msg_wall
            if self.first_accepted_wall is None:
                self.first_accepted_wall = time.monotonic()
                self.first_accepted_sim = sim_time
            if valid:
                within = e_xy <= self.tolerance_xy and e_yaw <= self.tolerance_yaw
                if within and self.first_within_tolerance_wall is None:
                    self.first_within_tolerance_wall = time.monotonic()
                if self.last_accepted_pose is not None:
                    jump_xy = float(
                        np.linalg.norm(estimate[:2, 3] - self.last_accepted_pose[:2, 3])
                    )
                    jump_yaw = angle_diff(
                        yaw_from_matrix(estimate[:3, :3]),
                        yaw_from_matrix(self.last_accepted_pose[:3, :3]),
                    )
                    self.observed_jump_xy = max(self.observed_jump_xy, jump_xy)
                    self.observed_jump_yaw = max(self.observed_jump_yaw, jump_yaw)
                self.last_accepted_pose = estimate
                self.accepted_records.append(
                    {
                        "sim_time": sim_time,
                        "wall_time": time.monotonic(),
                        "e_xy": e_xy,
                        "e_yaw": e_yaw,
                        "within": within,
                    }
                )
                self.accepted_in_window.append(
                    {
                        "sim_time": sim_time,
                        "wall_time": time.monotonic(),
                        "within": within,
                        "e_xy": e_xy,
                        "e_yaw": e_yaw,
                    }
                )
                self.check_success()
            if suspicious:
                self.suspicious_records.append(
                    {
                        "seq": record.get("seq"),
                        "sim_time": sim_time,
                        "e_xy": e_xy,
                        "e_yaw": e_yaw,
                        "branch": record.get("branch"),
                        "rmse": record.get("rmse"),
                        "inlier_ratio": record.get("inlier_ratio"),
                    }
                )
                self.get_logger().error(
                    "[evaluator] 可疑 accepted：seq=%s e_xy=%.3f m e_yaw=%.2f° rmse=%.3f "
                    "inliers=%.3f branch=%s"
                    % (
                        record.get("seq"),
                        e_xy,
                        math.degrees(e_yaw),
                        record.get("rmse", float("nan")),
                        record.get("inlier_ratio", float("nan")),
                        record.get("branch"),
                    )
                )

        self.write_row(record, sim_time, (e_xy, e_yaw, e_z), offset, valid, suspicious)

    def check_success(self) -> None:
        """方案 §8.2：连续稳定窗内满足精度、次数与跳变约束。

        稳定窗取「末尾连续的 within=True 区间」：恢复过程中的中间结果允许误差大，
        但只要最终进入并保持精度窗口，就按窗口末端判定成功。
        """
        if self.success or not self.accepted_records:
            return
        trailing = []
        for record in reversed(self.accepted_records):
            if not record["within"]:
                break
            trailing.append(record)
        trailing.reverse()
        if len(trailing) < self.min_accepted_in_window:
            return
        # 稳定窗末端以最后一个有效样本为准（时间对齐失败的 accepted 不能刷新有效性）
        last_valid = trailing[-1]
        window_start = last_valid["wall_time"] - self.stable_window
        in_window = [r for r in trailing if r["wall_time"] >= window_start]
        if len(in_window) < self.min_accepted_in_window:
            return
        if (
            self.observed_jump_xy > self.max_jump_xy
            or self.observed_jump_yaw > self.max_jump_yaw
        ):
            return
        self.success = True
        self.stable_end_wall = last_valid["wall_time"]
        self.stable_end_sim = last_valid["sim_time"]
        self.get_logger().info(
            "[evaluator] 判定成功：稳定窗 %.1f s 内 %d 次 accepted；首帧 accepted 墙钟 t0→%.3f s，"
            "稳定完成 t0→%.3f s"
            % (
                self.stable_window,
                len(in_window),
                (self.first_accepted_wall - self.t0_wall)
                if (self.first_accepted_wall and self.t0_wall)
                else float("nan"),
                (self.stable_end_wall - self.t0_wall) if self.t0_wall else float("nan"),
            )
        )

    def on_timer(self) -> None:
        if not self.active:
            return
        now_wall = time.monotonic()
        if self.t0_wall is not None and not self.success:
            elapsed = now_wall - self.t0_wall
            if elapsed > self.observation_timeout:
                self.active = False
                if not self.failure_reason:
                    self.failure_reason = "observation_timeout"
                self.get_logger().warn(
                    "[evaluator] %.1f s 内未达成稳定成功条件，判定失败（%s）"
                    % (elapsed, self.failure_reason)
                )
                self.flush()
        if self.round_start_wall is not None and now_wall - self.round_start_wall > self.task_timeout:
            if self.active:
                self.active = False
                self.failure_reason = self.failure_reason or "task_timeout"
                self.flush()

        status = {
            "run_id": self.run_id,
            "case_id": self.case_id,
            "active": self.active,
            "t0_ready": self.t0_ready,
            "success": self.success,
            "failure_reason": self.failure_reason,
            "diag_seen": self.diag_seen,
            "accepted_diag_seen": self.accepted_diag_seen,
            "valid_samples": len(self.accepted_records),
            "suspicious": len(self.suspicious_records),
            "elapsed": (now_wall - self.t0_wall) if self.t0_wall else None,
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(message)

    # ------------------------------------------------------------------ 服务

    def on_case_message(self, msg: StringMsg) -> None:
        """由 runner 发布本轮 case 元数据（JSON），评估器据此命名输出文件。"""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("[evaluator] case 元数据不是合法 JSON，已忽略")
            return
        self.set_case(
            payload.get("run_id", self.run_id),
            payload.get("case_id", self.case_id),
            payload.get("category", self.category),
            int(payload.get("attempt_index", self.attempt_index)),
        )
        self.get_logger().info(
            "[evaluator] 本轮 case 元数据：%s（%s）" % (self.case_id, self.category)
        )

    def on_start_run(self, request, response):
        del request
        self.active = True
        self.round_start_wall = time.monotonic()
        self.round_start_sim = self.get_clock().now().nanoseconds * 1e-9
        self.t0_wall = None
        self.t0_sim = None
        self.t0_ready = False
        self.first_accepted_wall = None
        self.first_within_tolerance_wall = None
        self.stable_end_wall = None
        self.stable_end_sim = None
        self.success = False
        self.failure_reason = ""
        self.accepted_records = []
        self.accepted_in_window = []
        self.iteration_rows = []
        self.suspicious_records = []
        self.observed_jump_xy = 0.0
        self.observed_jump_yaw = 0.0
        self.last_accepted_pose = None
        self.diag_seen = 0
        self.accepted_diag_seen = 0
        self.open_csv()
        response.success = True
        response.message = f"run {self.run_id}/{self.case_id} started"
        self.get_logger().info("[evaluator] 开始评估轮次 %s" % self.case_id)
        return response

    def on_stop_run(self, request, response):
        del request
        self.active = False
        if not self.success and not self.failure_reason:
            self.failure_reason = "stopped_without_success"
        summary = self.flush()
        response.success = True
        response.message = json.dumps(summary, ensure_ascii=False)
        return response

    # ------------------------------------------------------------------ 记录

    def set_case(self, run_id: str, case_id: str, category: str, attempt_index: int) -> None:
        self.run_id = run_id
        self.case_id = case_id
        self.category = category
        self.attempt_index = attempt_index

    def open_csv(self) -> None:
        if not self.write_csv:
            return
        directory = os.path.join(self.output_dir, self.run_id)
        os.makedirs(directory, exist_ok=True)
        self.csv_path = os.path.join(directory, f"iterations_{self.case_id}.csv")
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        self.csv_file.flush()

    def write_row(self, record, sim_time, errors, offset, valid=True, suspicious=False) -> None:
        e_xy = e_yaw = e_z = None
        if errors is not None:
            e_xy, e_yaw, e_z = errors
        row = {name: "" for name in CSV_FIELDS}
        row.update(
            {
                "seq": record.get("seq", ""),
                "event": record.get("event", "registration"),
                "sim_time": f"{sim_time:.6f}" if sim_time is not None else "",
                "wall_time": f"{record.get('node_wall_ns', 0) * 1e-9:.6f}"
                if record.get("node_wall_ns")
                else f"{time.monotonic():.6f}",
                "scan_stamp": f"{int(record.get('scan_stamp_ns', 0)) * 1e-9:.6f}",
                "scan_age": record.get("scan_age", ""),
                "raw_points": record.get("raw_points", ""),
                "source_points": record.get("source_points", ""),
                "converged": record.get("converged", ""),
                "iterations": record.get("iterations", ""),
                "accepted": record.get("accepted", ""),
                "branch": record.get("branch", ""),
                "reason": record.get("reason", ""),
                "inlier_ratio": record.get("inlier_ratio", ""),
                "rmse": record.get("rmse", ""),
                "dpos": record.get("dpos", ""),
                "dangle": record.get("dangle", ""),
                "duration_ms": record.get("duration_ms", ""),
                "est_x": record.get("result_x", ""),
                "est_y": record.get("result_y", ""),
                "est_yaw": record.get("result_yaw", ""),
                "time_offset": f"{offset:.6f}" if offset is not None else "",
                "valid": valid,
                "suspicious": suspicious,
                "init_x": record.get("init_x", ""),
                "init_y": record.get("init_y", ""),
                "init_yaw": record.get("init_yaw", ""),
            }
        )
        if e_xy is not None:
            row["e_xy"] = f"{e_xy:.6f}"
            row["e_yaw"] = f"{e_yaw:.6f}"
            row["e_z"] = f"{e_z:.6f}"
        if self.csv_writer is not None:
            self.csv_writer.writerow(row)
            self.csv_file.flush()
        self.iteration_rows.append(row)

    def flush(self) -> dict:
        summary = self.build_summary()
        if self.write_csv:
            directory = os.path.join(self.output_dir, self.run_id)
            os.makedirs(directory, exist_ok=True)
            summary_path = os.path.join(directory, "summary.json")
            with open(summary_path, "w", encoding="utf-8") as stream:
                json.dump(summary, stream, ensure_ascii=False, indent=2)
            self.summary_path = summary_path
            summary_csv = os.path.join(directory, "summary.csv")
            write_header = not os.path.isfile(summary_csv)
            with open(summary_csv, "a", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
                if write_header:
                    writer.writeheader()
                writer.writerow({name: summary.get(name, "") for name in SUMMARY_FIELDS})
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        return summary

    def build_summary(self) -> dict:
        accepted = self.accepted_records
        e_xy = [r["e_xy"] for r in accepted]
        e_yaw = [r["e_yaw"] for r in accepted]

        def pct(values, q):
            if not values:
                return float("nan")
            return float(np.percentile(np.asarray(values, dtype=np.float64), q))

        init_error_xy = ""
        init_error_yaw = ""
        if self.applied_init is not None and self.gt_samples:
            truth = self.t_map_world @ self.gt_samples[-1][1]
            init_error_xy = float(
                np.linalg.norm(self.applied_init[:2, 3] - truth[:2, 3])
            )
            init_error_yaw = angle_diff(
                yaw_from_matrix(self.applied_init[:3, :3]),
                yaw_from_matrix(truth[:3, :3]),
            )

        return {
            "run_id": self.run_id,
            "case_id": self.case_id,
            "category": self.category,
            "attempt_index": self.attempt_index,
            "passed": self.success,
            "failure_reason": self.failure_reason,
            "t_first_accepted_wall": (
                self.first_accepted_wall - self.t0_wall
                if (self.first_accepted_wall and self.t0_wall)
                else ""
            ),
            "t_first_within_tolerance_wall": (
                self.first_within_tolerance_wall - self.t0_wall
                if (self.first_within_tolerance_wall and self.t0_wall)
                else ""
            ),
            "t_stable_end_wall": (
                self.stable_end_wall - self.t0_wall if (self.stable_end_wall and self.t0_wall) else ""
            ),
            "init_duration_wall": (
                self.stable_end_wall - self.t0_wall if (self.stable_end_wall and self.t0_wall) else ""
            ),
            "init_duration_sim": (
                self.stable_end_sim - self.t0_sim
                if (self.stable_end_sim is not None and self.t0_sim is not None)
                else ""
            ),
            "accepted_count": len(accepted),
            "valid_samples": len([r for r in accepted]),
            "e_xy_median": pct(e_xy, 50),
            "e_xy_p95": pct(e_xy, 95),
            "e_xy_max": max(e_xy) if e_xy else "",
            "e_yaw_median": pct(e_yaw, 50),
            "e_yaw_p95": pct(e_yaw, 95),
            "e_yaw_max": max(e_yaw) if e_yaw else "",
            "suspect_count": len(self.suspicious_records),
            "max_jump_xy": self.observed_jump_xy,
            "max_jump_yaw": self.observed_jump_yaw,
            "init_error_xy": init_error_xy,
            "init_error_yaw": init_error_yaw,
            "details": {
                "diag_seen": self.diag_seen,
                "accepted_diag_seen": self.accepted_diag_seen,
                "t0_sim": self.t0_sim,
                "t0_wall": self.t0_wall,
                "suspicious": self.suspicious_records,
                "csv": self.csv_path,
            },
        }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="GICP 初始化验证的只读真值评估器")
    known, remaining = parser.parse_known_args(argv)
    del known
    rclpy.init(args=remaining)
    node = TruthEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.success and node.t0_wall is not None:
            node.failure_reason = node.failure_reason or "shutdown"
            node.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
