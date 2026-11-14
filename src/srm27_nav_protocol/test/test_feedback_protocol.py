#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2026 SRM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""下位机反馈帧（0x0010 底盘速度包）的协议级回归。

这一层错了的表现非常分散，而且都不会报错：
* CRC 覆盖范围写错（漏掉 cmd_id）→ 偶发整帧被丢，看起来像"下位机没发"；
* 按"第几个包"解析而不是按 cmd_id → 丢一个槽位之后**永久错位**；
* 长度不自检 → 把 12 字节当成 20 字节读，vx/vy/wz/chassis_to_gimbal_yaw 变成帧里别的字节；
* 新旧帧不区分 → 下位机还没升级时（16 字节）把 CRC 字节当成云台转速读出来；
* 小端当大端 → 速度变成 1e-38 或天文数字。

本测试用**独立实现**的组帧/解析（`scripts/test/test_chassis_speed_feedback.py`，与 C++ 节点
是两套代码）互相校验；不启动 ROS、不需要串口。
"""

import importlib.util
import struct
import sys
from pathlib import Path

#: 本文件在 <ws>/src/srm27_nav_protocol/test/ 下。
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
TOOL = WORKSPACE_ROOT / "script" / "test_chassis_speed_feedback.py"


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def _load_tool():
    """加载下位机反馈帧工具（组帧/解析/CRC 的参考实现）。"""
    _assert(TOOL.is_file(), f"找不到参考实现: {TOOL}")
    spec = importlib.util.spec_from_file_location("chassis_feedback_tool", str(TOOL))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


TOOL_MODULE = _load_tool()
build_frame = TOOL_MODULE.build_frame
build_chassis_speed = TOOL_MODULE.build_chassis_speed
FrameParser = TOOL_MODULE.FrameParser
crc8 = TOOL_MODULE.crc8
crc16 = TOOL_MODULE.crc16
PKT_CHASSIS_SPEED = TOOL_MODULE.PKT_CHASSIS_SPEED
PKT_GAME_STATUS = TOOL_MODULE.PKT_GAME_STATUS
PKT_ROBOT_STATUS = TOOL_MODULE.PKT_ROBOT_STATUS
LEN_CHASSIS_SPEED = TOOL_MODULE.LEN_CHASSIS_SPEED          # 20（新帧）
LEN_CHASSIS_SPEED_V1 = TOOL_MODULE.LEN_CHASSIS_SPEED_V1    # 16（旧帧）


def test_crc_reference_vector_from_the_protocol_doc():
    """协议文档 §5 的上→下参考帧必须能通过 CRC8/CRC16 自检。

    这是唯一一条"外部给定的"向量：CRC 表或初值一旦被改错，这里立刻失败。
    """
    # A5 | 0C 00 | 01 | 26 | vx=1.0 vy=0 wz=0 (三个 float32) | 6B A3
    ref = bytes.fromhex("A50C0001" "26" "0000803F" "00000000" "00000000" "6BA3")
    _assert(len(ref) == 19, "参考帧长度应为 19 字节")
    _assert(crc8(ref[:4]) == ref[4], "CRC8 参考向量不符")
    _assert(crc16(ref[:17]) == struct.unpack("<H", ref[17:])[0], "CRC16 参考向量不符")


def test_chassis_speed_frame_shape():
    """0x0010 = 9 + 20 = 29 字节（2026-11-08 起），小端，五个 float32。"""
    frame = build_frame(
        PKT_CHASSIS_SPEED, build_chassis_speed(0.4, -0.1, 0.75, 45.0, 1.25), seq=3)
    _assert(len(frame) == 29, f"整帧应为 29 字节，实得 {len(frame)}")
    _assert(frame[0] == 0xA5, "帧首应为 0xA5")
    _assert(struct.unpack("<H", frame[1:3])[0] == LEN_CHASSIS_SPEED, "data_length 应为 20")
    _assert(frame[3] == 3, "seq 应写在偏移 3")
    _assert(struct.unpack("<H", frame[5:7])[0] == PKT_CHASSIS_SPEED, "cmd_id 应为 0x0010（小端）")
    vx, vy, wz, yaw, rate_imu = struct.unpack("<fffff", frame[7:27])
    _assert(abs(vx - 0.4) < 1e-6 and abs(vy + 0.1) < 1e-6, "vx/vy 编解码不一致")
    _assert(abs(wz - 0.75) < 1e-6 and abs(yaw - 45.0) < 1e-6, "wz/yaw 编解码不一致")
    _assert(abs(rate_imu - 1.25) < 1e-6, "云台陀螺仪转速必须落在数据段偏移 16")
    _assert(frame[7 + 16:7 + 20] == struct.pack("<f", 1.25), "陀螺仪转速偏移必须是 16")


def test_legacy_16_byte_frame_is_still_accepted():
    """旧固件（data_length=16）必须能解析：前 4 个字段位置不变，两路转速记为不可用。

    方案 §4.4.1：上下位机不是同时升级的 —— 遇到旧帧绝不报错，只是转速不可用
    （None，而不是 0 rad/s；0 会被下游当成"云台没转"的真实观测）。
    """
    parser = FrameParser()
    legacy = build_frame(PKT_CHASSIS_SPEED, build_chassis_speed(0.4, -0.1, 0.75, 45.0), seq=5)
    _assert(len(legacy) == 25, f"旧帧整帧应为 25 字节，实得 {len(legacy)}")
    _assert(struct.unpack("<H", legacy[1:3])[0] == LEN_CHASSIS_SPEED_V1,
            "旧帧 data_length 必须仍是 16")
    out = parser.feed(legacy)
    _assert(len(out) == 1, "旧帧本身是合法的，解析器应收到它")
    decoded = parser.decode(out[0][1])
    _assert(decoded is not None, "旧帧不能被当成非法长度")
    _assert(abs(decoded["vx"] - 0.4) < 1e-6 and abs(decoded["vy"] + 0.1) < 1e-6,
            "旧帧前 4 个字段的位置与语义必须不变")
    _assert(decoded["gimbal_yaw_rate_imu"] is None,
            "旧帧没有云台转速，必须是 None（不可用）")


def test_crc16_covers_cmd_id():
    """CRC16 覆盖范围必须**包含 cmd_id**（协议 §4.1）。"""
    frame = bytearray(build_frame(PKT_CHASSIS_SPEED, build_chassis_speed(0.1, 0.2, 0.3, 10.0)))
    original_crc = struct.unpack("<H", frame[-2:])[0]
    frame[5] ^= 0xFF  # 只改 cmd_id
    _assert(crc16(bytes(frame[:-2])) != original_crc,
            "改了 cmd_id 却还能算出同一个 CRC16 —— 说明 CRC 覆盖范围漏了 cmd_id")


def test_round_robin_with_skipped_slots():
    """三个包按 10 ms 槽位轮转、DMA 忙时整槽跳过；解析必须按 cmd_id 分派。

    这里刻意丢掉一批包（模拟"槽位被跳过"），解析结果仍必须逐帧正确。
    """
    parser = FrameParser()
    decoded = []
    good = 0
    seq = 0
    for slot in range(60):
        seq = (seq + 1) & 0xFF
        # 每 7 个槽位丢一个（不是重发，是直接消失）
        if slot % 7 == 3:
            continue
        kind = slot % 3
        if kind == 0:
            frame = build_frame(PKT_GAME_STATUS, b"\x00" * 11, seq=seq)
        elif kind == 1:
            frame = build_frame(PKT_ROBOT_STATUS, b"\x00" * 13, seq=seq)
        else:
            good += 1
            frame = build_frame(PKT_CHASSIS_SPEED,
                                build_chassis_speed(0.01 * good, 0.0, 0.5, float(good)), seq=seq)
        # 逐字节喂，模拟真实串口的分片读取（一次读到的不是整帧）
        for byte in frame:
            for cmd_id, payload, _ in parser.feed(bytes([byte])):
                if cmd_id == PKT_CHASSIS_SPEED:
                    decoded.append(parser.decode(payload))
    _assert(len(decoded) == good, f"0x0010 应解出 {good} 帧，实得 {len(decoded)}")
    for index, item in enumerate(decoded, start=1):
        _assert(item is not None, f"第 {index} 帧解码失败")
        _assert(abs(item["vx"] - 0.01 * index) < 1e-6,
                f"第 {index} 帧 vx 应为 {0.01 * index}，实得 {item['vx']}")
        _assert(abs(item["chassis_to_gimbal_yaw_deg"] - index) < 1e-6,
                f"第 {index} 帧 chassis_to_gimbal_yaw 应连续，"
                f"实得 {item['chassis_to_gimbal_yaw_deg']}")
    _assert(parser.stats["crc8_fail"] == 0 and parser.stats["crc16_fail"] == 0,
            f"正常流不该有 CRC 失败: {parser.stats}")


def test_corrupted_frames_are_dropped_and_stream_recovers():
    """CRC16 坏帧只丢自己，后面的帧必须能重新同步。"""
    parser = FrameParser()
    first = build_frame(PKT_CHASSIS_SPEED, build_chassis_speed(1.0, 0.0, 0.0, 0.0), seq=1)
    broken = bytearray(build_frame(PKT_CHASSIS_SPEED, build_chassis_speed(9.9, 0.0, 0.0, 0.0), seq=2))
    broken[-3] ^= 0x5A  # 改数据段，不动 CRC
    second = build_frame(PKT_CHASSIS_SPEED, build_chassis_speed(2.0, 0.0, 0.0, 0.0), seq=3)

    got = []
    for cmd_id, payload, _ in parser.feed(first + bytes(broken) + second):
        if cmd_id == PKT_CHASSIS_SPEED:
            got.append(parser.decode(payload))
    _assert(len(got) == 2, f"坏帧应被丢弃、好帧应解出 2 个，实得 {len(got)}")
    _assert(abs(got[0]["vx"] - 1.0) < 1e-6 and abs(got[1]["vx"] - 2.0) < 1e-6,
            "恢复后的帧数值不对")
    _assert(parser.stats["crc16_fail"] >= 1, "坏帧应被计入 crc16_fail")


def test_truncated_frame_waits_for_more_bytes():
    """断帧不能被当成整帧解析（否则会读出半截垃圾）。"""
    parser = FrameParser()
    frame = build_frame(PKT_CHASSIS_SPEED, build_chassis_speed(0.5, 0.25, 0.75, 30.0), seq=9)
    out = parser.feed(frame[:10])
    _assert(out == [], "不完整的帧不该产出解析结果")
    out = parser.feed(frame[10:])
    _assert(len(out) == 1, "剩余字节补齐后应解出 1 帧")
    decoded = parser.decode(out[0][1])
    _assert(abs(decoded["vx"] - 0.5) < 1e-6 and abs(decoded["wz"] - 0.75) < 1e-6, "补齐后数值不对")


def test_wrong_data_length_is_rejected():
    """0x0010 的 data_length 只能是 20（新）或 16（旧）；写成 12 时解码必须返回 None。

    节点侧对应的是 `receive_length_matches<T>()` 检查 —— 长度不符直接丢弃 +
    计数告警，否则 12 字节会被当成 20 字节读（多出来的字节是 CRC 与下一帧的开头）。
    """
    parser = FrameParser()
    bogus = build_frame(PKT_CHASSIS_SPEED, b"\x00" * 12, seq=1)
    _assert(len(bogus) == 21, "12 字节数据段的整帧应为 21 字节")
    out = parser.feed(bogus)
    _assert(len(out) == 1, "帧本身（CRC）是合法的，解析器应收到它")
    _assert(parser.decode(out[0][1]) is None, "data_length 既不是 20 也不是 16 时必须拒绝解码")


def test_seq_wraps_without_losing_frames():
    """seq 是逐帧自增的 uint8，回绕不该影响任何解析。"""
    parser = FrameParser()
    got = 0
    for index in range(300):
        frame = build_frame(PKT_CHASSIS_SPEED,
                            build_chassis_speed(0.1, 0.0, 0.0, 0.0), seq=index & 0xFF)
        for cmd_id, payload, _ in parser.feed(frame):
            if cmd_id == PKT_CHASSIS_SPEED and parser.decode(payload) is not None:
                got += 1
    _assert(got == 300, f"seq 回绕期间应解出 300 帧，实得 {got}")


def _run_all():
    failures = 0
    for name, function in sorted(globals().items()):
        if not name.startswith("test_") or not callable(function):
            continue
        try:
            function()
        except Exception as error:  # noqa: BLE001 - 测试入口需要报告任意失败
            failures += 1
            print(f"FAIL {name}: {error!r}")
        else:
            print(f"PASS {name}")
    return failures


if __name__ == "__main__":
    sys.exit(1 if _run_all() else 0)
