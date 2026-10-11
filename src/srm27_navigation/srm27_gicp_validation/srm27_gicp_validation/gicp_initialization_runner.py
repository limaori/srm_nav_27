#!/usr/bin/env python3
"""GICP 初始重定位批量轮次驱动（方案 §6/§7/§9）。

职责：

* 从 YAML 读取试验矩阵（类别、初值误差、每格重复次数、随机种子）；
* 按轮次启动仿真、点云/LIO 链路与独立真值评估器；
* 每轮重启 GICP 节点（可配 ``restart_mode`` 覆盖冷启动 / 运行中重置初值两种口径）；
* 按真值计算「带误差的机器人位姿」并换算成 ``T_M_O_init``，发布 ``/initialpose``；
* 超时归档，写出 ``manifest.yaml`` 与逐轮 ``summary``。

与方案 §3.2 的差异记录在生成的报告里：为把 945 次矩阵变成可执行的批测，
默认用「同一 Gazebo 会话内重启 GICP 节点」的方式，方案要求的完整冷启动另有
``restart_mode: cold_full`` 模式。

关键坐标换算（方案 §5）::

    T_M_O_init = T_M_B_guess · T_O_B(t_init)^-1
"""

import argparse
import copy
import json
import threading
import math
import os
import random
import signal
import subprocess
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
import rclpy.executors
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gicp_validation_common import (  # noqa: E402
    angle_diff,
    invert,
    make_transform,
    matrix_to_quaternion,
    quaternion_to_matrix,
    transform_from_pose,
    wrap_to_pi,
    yaw_from_matrix,
    yaw_from_quaternion,
)

WORKSPACE = "/home/srm/srm_nav_27"


class ValidationRunner(Node):
    def __init__(self, config: dict, output_dir: str, run_id: str) -> None:
        super().__init__("gicp_initialization_runner")
        self.config = config
        self.output_dir = output_dir
        self.run_id = run_id
        self.namespace = config["namespace"]
        self.spawn = transform_from_pose(
            float(config["spawn"]["x"]),
            float(config["spawn"]["y"]),
            0.0,
            float(config["spawn"]["yaw"]),
        )
        self.t_map_world = invert(self.spawn)

        self.latest_gt = None
        self.gt_stamp = None
        self.gt_count = 0
        self.latest_odom_base = None
        self.latest_scan_stamp = None
        self.scan_count = 0
        self.gicp_log = None
        self.stack_processes = []
        self.case_results = []
        self.current_case = None
        self.spawn_verified = False
        self.gicp_process = None
        self.spawn_check_error = None

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, f"{self.namespace}/initialpose", 10
        )
        self.gt_sub = self.create_subscription(
            Odometry, f"{self.namespace}/chassis_odometry_gt", self.on_gt, 50
        )
        self.scan_sub = self.create_subscription(
            PointCloud2,
            f"{self.namespace}/{config['topics']['registered_scan']}",
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.status_sub = self.create_subscription(
            String, "gicp_eval/status", self.on_status, 20
        )
        self.case_pub = self.create_publisher(String, "gicp_eval/case", 10)
        self._last_status = {}
        # 关键：runner 是同步流程（等待→发初值→等待），必须在后台线程里跑一个
        # executor，否则订阅回调永远不会被执行，gt/scan 计数永远是 0。
        self.spin_stop = threading.Event()
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()
        self.get_logger().info(
            "[runner] 订阅：gt=%s scan=%s initialpose=%s（namespace=%r）"
            % (
                self.gt_sub.topic_name,
                self.scan_sub.topic_name,
                self.initialpose_pub.topic_name,
                self.namespace,
            )
        )

    def _spin_loop(self) -> None:
        """后台线程里持续处理回调：runner 主流程是同步等待式的，必须有人 spin。"""
        while not self.spin_stop.is_set() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    # ------------------------------------------------------------------ 订阅

    def on_gt(self, msg: Odometry) -> None:
        self.gt_count += 1
        pose = msg.pose.pose
        self.latest_gt = make_transform(
            [pose.position.x, pose.position.y, pose.position.z],
            quaternion_to_matrix(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        self.gt_stamp = time.monotonic()
        if not self.spawn_verified:
            self.spawn_verified = True
            expected = invert(self.t_map_world)
            error_xy = float(np.linalg.norm(self.latest_gt[:2, 3] - expected[:2, 3]))
            self.spawn_check_error = error_xy
            self.get_logger().info(
                "[runner] 出生点真值检查：gt=(%.3f, %.3f, yaw=%.4f) 期望=(%.3f, %.3f) 误差=%.4f m"
                % (
                    self.latest_gt[0, 3],
                    self.latest_gt[1, 3],
                    yaw_from_matrix(self.latest_gt[:3, :3]),
                    expected[0, 3],
                    expected[1, 3],
                    error_xy,
                )
            )
            if error_xy > 0.05:
                self.get_logger().error(
                    "[runner] 出生点与配置不一致，后续误差计算不可信（方案 §4.1）"
                )

    def on_scan(self, msg: PointCloud2) -> None:
        del msg
        self.scan_count += 1
        self.latest_scan_stamp = time.monotonic()

    def on_status(self, msg: String) -> None:
        try:
            self._last_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ------------------------------------------------------------------ 进程

    def shell(self, command: str, log_name: str, env_extra: dict | None = None):
        env = os.environ.copy()
        if env_extra:
            env.update(env_extra)
        log_path = os.path.join(self.output_dir, "logs", log_name)
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        stream = open(log_path, "a", encoding="utf-8")
        stream.write(f"\n===== {datetime.now().isoformat()} :: {command}\n")
        stream.flush()
        process = subprocess.Popen(
            ["bash", "-lc", command],
            cwd=WORKSPACE,
            env=env,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        return process, log_path

    def start_stack(self) -> None:
        cfg = self.config
        if cfg.get("skip_stack_management"):
            # 链路与评估器由外层会话脚本（script/gicp_validation/run_session.sh）启动，
            # runner 只等待输入就绪。
            self.get_logger().info("[runner] skip_stack_management=true：只等待输入就绪")
            if not self.wait_for_inputs(float(cfg.get("stack_startup_timeout", 240.0))):
                raise TimeoutError("等待真值里程计与 registered_scan 超时")
            return
        launch = cfg.get("stack_command")
        if not launch:
            raise RuntimeError("配置缺少 stack_command：无法启动仿真/定位链路")
        for name, command in launch.items():
            process, log_path = self.shell(command, f"stack_{name}.log")
            self.stack_processes.append((name, process, log_path))
            self.get_logger().info("[runner] 启动 %s -> %s" % (name, log_path))
            time.sleep(float(cfg.get("stack_launch_gap", 2.0)))

        startup_timeout = float(cfg.get("stack_startup_timeout", 180.0))
        deadline = time.monotonic() + startup_timeout
        while time.monotonic() < deadline:
            ready = self.latest_gt is not None and self.scan_count > 5
            if ready:
                self.get_logger().info("[runner] 链路就绪：真值里程计与 registered_scan 均有数据")
                return
            time.sleep(0.5)
        raise TimeoutError("链路启动超时：真值里程计或 registered_scan 未就绪")

    def stop_stack(self) -> None:
        for name, process, _ in self.stack_processes:
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGINT)
                except ProcessLookupError:
                    pass
        for name, process, _ in self.stack_processes:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.stack_processes = []

    def start_gicp(self, params_file: str) -> None:
        cfg = self.config
        command = cfg["gicp_command"].format(params_file=params_file, run_id=self.run_id)
        process, log_path = self.shell(command, "gicp.log")
        self.gicp_process = process
        self.gicp_log = log_path

    def stop_gicp(self) -> None:
        process = getattr(self, "gicp_process", None)
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    # ------------------------------------------------------------------ 评估服务

    def call_service(self, name: str, timeout: float = 5.0):
        client = self.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=timeout):
            self.destroy_client(client)
            return None
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.destroy_client(client)
        return future.result() if future.done() else None

    # ------------------------------------------------------------------ 轮次

    def wait_for_inputs(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        last_report = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() - last_report > 10.0:
                last_report = time.monotonic()
                self.get_logger().info(
                    "[runner] 等待输入：gt=%d 条(scans=%d) 最新 gt %.1f s 前 / 最新扫描 %.1f s 前"
                    % (
                        self.gt_count,
                        self.scan_count,
                        (time.monotonic() - self.gt_stamp) if self.gt_stamp else -1.0,
                        (time.monotonic() - self.latest_scan_stamp)
                        if self.latest_scan_stamp
                        else -1.0,
                    )
                )
            fresh_gt = self.gt_stamp is not None and time.monotonic() - self.gt_stamp < 1.0
            fresh_scan = (
                self.latest_scan_stamp is not None
                and time.monotonic() - self.latest_scan_stamp < 1.0
            )
            if fresh_gt and fresh_scan:
                return True
            time.sleep(0.1)
        return False

    def odom_base_now(self):
        """由真值求得当前 odom→base_link：T_O_B = T_O_W · T_W_B。

        odom 与地图同原点（方案 §4.1），因此在此工程中 T_M_O(t) 的估计值为 I 时
        ``odom`` 系的参考点就是地图原点；由此得到 T_O_B = inverse(T_M_W)⁻¹… 更直接地：

        ``T_O_B = T_O_M · T_M_B``，其中 ``T_O_M = inverse(T_M_O)``；
        出生时 ``T_M_O = I``（机器人静止在出生点），所以取 ``T_O_B = T_W_B`` 的平面部分。
        """
        if self.latest_gt is None:
            return None
        return self.latest_gt

    def make_initial_pose(self, position_error: tuple, yaw_error_deg: float):
        """按误差生成机器人位姿猜测与 T_M_O_init（方案 §5）。"""
        if self.latest_gt is None:
            raise RuntimeError("没有真值，无法生成初值")
        truth_map = self.t_map_world @ self.latest_gt
        yaw_truth = yaw_from_matrix(truth_map[:3, :3])
        guess = transform_from_pose(
            truth_map[0, 3] + position_error[0],
            truth_map[1, 3] + position_error[1],
            0.0,
            wrap_to_pi(yaw_truth + math.radians(yaw_error_deg)),
        )
        # T_M_O_init = T_M_B_guess · T_O_B(t_init)^-1；静止时 T_O_B(t)=T_M_W·T_M_B(t)
        t_o_b = invert(self.t_map_world @ self.latest_gt)
        return guess, guess @ invert(t_o_b)

    def publish_initial_pose(self, guess) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.config["frames"]["map"]
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(guess[0, 3])
        message.pose.pose.position.y = float(guess[1, 3])
        message.pose.pose.position.z = float(guess[2, 3])
        x, y, z, w = matrix_to_quaternion(guess[:3, :3])
        message.pose.pose.orientation.x = x
        message.pose.pose.orientation.y = y
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        self.initialpose_pub.publish(message)

    def run_case(self, case: dict) -> dict:
        self.current_case = case
        case_id = case["case_id"]
        self.get_logger().info(
            "[runner] === 轮次 %s：类别=%s 位置误差=(%.2f, %.2f) yaw 误差=%.1f° 重复=%d"
            % (
                case_id,
                case.get("category", ""),
                case["position_error"][0],
                case["position_error"][1],
                case["yaw_error_deg"],
                case.get("repeat", 1),
            )
        )
        record = {
            "case_id": case_id,
            "category": case.get("category", ""),
            "position_error": list(case["position_error"]),
            "yaw_error_deg": case["yaw_error_deg"],
            "repeat": case.get("repeat", 1),
            "passes": 0,
            "attempts": [],
        }

        for attempt in range(int(case.get("repeat", 1))):
            attempt_result = self.run_attempt(case, attempt)
            record["attempts"].append(attempt_result)
            if attempt_result.get("passed"):
                record["passes"] += 1
            if self.config.get("stop_on_failure") and not attempt_result.get("passed"):
                self.get_logger().warn("[runner] 配置为失败即停，结束本轮")
                break
        record["passed"] = record["passes"] == int(case.get("repeat", 1))
        self.case_results.append(record)
        self.append_case_summary(record)
        return record

    def run_attempt(self, case: dict, attempt: int) -> dict:
        cfg = self.config
        case_id = f"{case['case_id']}_a{attempt:02d}"
        category = case.get("category", "")

        self.stop_gicp()
        if not self.wait_for_inputs(float(cfg.get("input_ready_timeout", 30.0))):
            self.get_logger().error("[runner] %s 输入未就绪（点云/真值）" % case_id)
            return {"attempt": attempt, "passed": False, "failure_reason": "input_not_ready"}

        # 评估器按本轮 case 重新标记并开始计时
        self.call_service("gicp_eval/stop_run", timeout=5.0)
        case_message = String()
        case_message.data = json.dumps(
            {
                "run_id": self.run_id,
                "case_id": case_id,
                "category": category,
                "attempt_index": attempt,
            },
            ensure_ascii=False,
        )
        self.case_pub.publish(case_message)
        time.sleep(0.3)
        self.call_service("gicp_eval/start_run", timeout=5.0)

        # 机器人保持静止，等真值稳定
        time.sleep(float(cfg.get("settle_seconds", 1.0)))

        guess, map_to_odom = self.make_initial_pose(
            tuple(case["position_error"]), case["yaw_error_deg"]
        )

        restart_mode = case.get("restart_mode", cfg.get("restart_mode", "gicp_only"))
        if restart_mode == "cold_full":
            self.stop_stack()
            time.sleep(2.0)
            self.start_stack()

        # params_file 来自配置（试验参数副本），case 里可选覆盖
        params_file = case.get("params_file") or cfg.get("params_file")
        if params_file is None:
            params_file = cfg.get("artifacts", {}).get("params_file")
        if params_file is None:
            raise RuntimeError("配置缺少 params_file（或 artifacts.params_file）")
        self.start_gicp(params_file)

        # 冷启动等待：GICP 构造期间要等雷达外参 TF，之后才可能接受初值
        cold_wait = float(case.get("cold_init_wait", cfg.get("cold_init_wait", 0.0)))
        if cold_wait > 0.0:
            time.sleep(cold_wait)

        t_sent_wall = time.monotonic()
        t_sent_sim = self.get_clock().now().nanoseconds * 1e-9
        self.publish_initial_pose(guess)
        # 再发一次，避免首帧丢失（方案 §5 允许重复发布初值）
        time.sleep(0.2)
        self.publish_initial_pose(guess)

        timeout = float(cfg.get("observation_timeout", 10.0)) + float(
            cfg.get("post_success_observe", 0.0)
        )
        deadline = t_sent_wall + timeout + float(cfg.get("settle_seconds", 1.0))
        status = {}
        while time.monotonic() < deadline:
            status = self._last_status or {}
            if status.get("success") or status.get("failure_reason"):
                break
            time.sleep(0.2)

        summary = self.call_service("gicp_eval/stop_run", timeout=10.0)
        result = {
            "attempt": attempt,
            "case_id": case_id,
            "initial_guess_map": {
                "x": float(guess[0, 3]),
                "y": float(guess[1, 3]),
                "yaw": float(yaw_from_matrix(guess[:3, :3])),
            },
            "map_to_odom_init": {
                "x": float(map_to_odom[0, 3]),
                "y": float(map_to_odom[1, 3]),
                "yaw": float(yaw_from_matrix(map_to_odom[:3, :3])),
            },
            "sent_wall": t_sent_wall,
            "sent_sim": t_sent_sim,
        }
        if summary is not None:
            try:
                parsed = json.loads(summary.message)
            except json.JSONDecodeError:
                parsed = {}
            result["evaluation"] = parsed
            result["passed"] = bool(parsed.get("passed"))
            result["failure_reason"] = parsed.get("failure_reason", "")
        else:
            result["passed"] = False
            result["failure_reason"] = "evaluator_unavailable"
        if not self.config.get("keep_running", False):
            self.stop_gicp()
        return result

    def append_case_summary(self, record: dict) -> None:
        path = os.path.join(self.output_dir, self.run_id, "cases.jsonl")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def write_manifest(self) -> None:
        directory = os.path.join(self.output_dir, self.run_id)
        os.makedirs(directory, exist_ok=True)
        manifest = {
            "run_id": self.run_id,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "config": self.config,
            "spawn_world": {
                "x": float(self.spawn[0, 3]),
                "y": float(self.spawn[1, 3]),
                "yaw": float(yaw_from_matrix(self.spawn[:3, :3])),
            },
            "T_M_W": [[float(v) for v in row] for row in self.t_map_world],
            "spawn_check_error_m": self.spawn_check_error,
            "cases": self.case_results,
        }
        with open(os.path.join(directory, "manifest.yaml"), "w", encoding="utf-8") as stream:
            yaml.safe_dump(manifest, stream, allow_unicode=True, sort_keys=False)


def build_cases(matrix: dict) -> list:
    """把矩阵描述展开成逐轮 case 列表（方案 §7）。"""
    cases = []
    rng = random.Random(int(matrix.get("seed", 0)))
    for point in matrix["points"]:
        for position in matrix["position_errors"]:
            for yaw in matrix["yaw_errors_deg"]:
                repeat = int(matrix.get("repeat", 1))
                case_id = "%s_p%.2f_%.2f_y%+.0f" % (
                    point["name"],
                    position[0],
                    position[1],
                    yaw,
                )
                cases.append(
                    {
                        "case_id": case_id,
                        "category": matrix.get("category", ""),
                        "point": point,
                        "position_error": [float(position[0]), float(position[1])],
                        "yaw_error_deg": float(yaw),
                        "repeat": repeat,
                        "seed": rng.randrange(1 << 30),
                        "params_file": matrix.get("params_file"),
                        "restart_mode": matrix.get("restart_mode"),
                    }
                )
    return cases


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="GICP 初始化批量轮次驱动")
    parser.add_argument("--config", required=True, help="验证配置 YAML")
    parser.add_argument("--matrix", default=None, help="试验矩阵 YAML（默认取配置里的）")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--dry-run", action="store_true", help="只展开矩阵不启动仿真")
    parser.add_argument("--list-cases", action="store_true")
    known, remaining = parser.parse_known_args(argv)

    with open(known.config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    matrix_path = known.matrix or config.get("matrix")
    with open(matrix_path, "r", encoding="utf-8") as stream:
        matrix = yaml.safe_load(stream)

    output_dir = known.output_dir or config.get("output_dir", "/tmp/gicp_validation/results")
    run_id = known.run_id or datetime.now().strftime("%Y%m%d_%H%M%S")

    cases = build_cases(matrix)
    if known.list_cases:
        for case in cases:
            print(json.dumps(case, ensure_ascii=False))
        print(f"# 共 {len(cases)} 组，每组重复 {matrix.get('repeat', 1)} 次")
        return 0
    if known.dry_run:
        print(json.dumps({"cases": len(cases), "matrix": matrix}, ensure_ascii=False, indent=2))
        return 0

    rclpy.init(args=remaining)
    runner = ValidationRunner(config, output_dir, run_id)
    exit_code = 0
    try:
        os.makedirs(os.path.join(output_dir, run_id), exist_ok=True)
        runner.start_stack()
        for case in cases:
            runner.run_case(case)
        runner.write_manifest()
    except Exception as error:  # noqa: BLE001
        runner.get_logger().error("[runner] 运行失败：%s" % error)
        runner.write_manifest()
        exit_code = 1
    finally:
        if not config.get("keep_running", False):
            runner.stop_gicp()
            runner.stop_stack()
        runner.spin_stop.set()
        runner.spin_thread.join(timeout=2.0)
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
