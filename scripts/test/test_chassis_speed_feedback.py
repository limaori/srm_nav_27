#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# SRM 下位机 → 上位机 反馈帧测试（当前只解析 0x0010 底盘速度包）
#
# 用途（三种模式，互不依赖）:
#   --self-test   只做协议自检（CRC8/CRC16 + 帧长 + 参考向量），不碰串口、不需要 ROS
#   --device      直连串口**只读**，按真实链路解析并打印统计（下位机到底发没发）
#   --pty         造一对伪串口，按 33.3 Hz 发合成的 0x0010 帧，并打印从端设备路径；
#                 用来在没有硬件的情况下验证 ROS 节点（见下面示例）
#
# 帧格式（下→上，协议 §4.1，全程小端）:
#   A5 | data_length(uint16 LE) | seq(uint8, 逐帧自增) | CRC8(uint8, 覆盖前 4 字节, init 0xFF)
#      | cmd_id(uint16 LE) | data(data_length 字节) | CRC16(uint16 LE, 覆盖帧首到数据段末尾,
#      **包含 cmd_id**, init 0xFFFF)
#   整帧长度 = 5 + 2 + data_length + 2 = 9 + data_length
#
# 多包复用: 0x0001(比赛状态, 11B) / 0x000B(机器人状态, 13B) / 0x0010(速度, 20B 新 / 16B 旧)
#   按 10 ms 槽位轮转（各自约 33.3 Hz）。串口 DMA 没发完时会**跳过该槽**（不是重发），
#   所以上位机必须按 cmd_id 分派，不能按"第几个包"解析。本工具演示的正是这种写法。
#
# 0x0010 数据段（2026-11-08 起 20 字节 = 5 个 float32；旧固件仍是 16 字节 = 4 个）:
#   vx  m/s   **云台系** +x 向前（云台的"前"）
#   vy  m/s   **云台系** +y 向左
#   wz  rad/s **底盘**自转，俯视逆时针为正
#   chassis_to_gimbal_yaw  度   底盘车头相对云台的偏角：云台指向为 0，范围 (-180,180]，
#                               俯视逆时针为正，由云台 yaw 编码器解算，与 IMU/上电零位无关。
#                               （2026-11-08 改的口径；它不是"底盘在世界系里的 yaw"）
#   gimbal_yaw_rate_imu  rad/s  云台陀螺仪 yaw 转速（相对地面）  ← 新增，yaw 外推用
#   （编码器转速不单独回传：它就是 chassis_to_gimbal_yaw 的变化率）
#   注意: 填充的是底盘速度观测器的融合结果 speed->fused，不是轮速正解；
#         也不要拿它当"指令已被执行"的证据（指令→观测有延迟与尺度差）。
#         上行 vx/vy 按**云台系**（下位机按云台前执行 vx/vy，2026-11-08 起）。
#   新旧帧靠 data_length 区分：20 = 新帧（有转速），16 = 旧帧（转速不可用，不报错）。
#
# 用法:
#   ./scripts/test/test_chassis_speed_feedback.py --self-test
#   ./scripts/test/test_chassis_speed_feedback.py --device /dev/ttyACM0
#   ./scripts/test/test_chassis_speed_feedback.py --device /dev/serial/by-id/usb-... --raw
#
#   # 没有硬件时，端到端验证 ROS 节点（两个终端）:
#   #   终端 A:
#   ./scripts/test/test_chassis_speed_feedback.py --pty --vx 0.4 --vy 0.1 --wz 0.8 --yaw 45
#   #     -> 打印 "从端设备: /dev/pts/N"，保持运行
#   #   终端 B（N 换成上面打印的）:
#   ros2 run srm27_nav_protocol srm27_nav_protocol_node --ros-args \\
#       -p device_name:=/dev/pts/N
#   ros2 topic echo /chassis_speed                 # TwistStamped
#   ros2 topic echo /chassis_to_gimbal_yaw_deg     # Float32 (度, 下位机原值)
#   ros2 topic echo /gimbal_joint_angle            # Float32 (rad, gimbal_yaw_joint)
#
# 参数:
#   -d, --device <路径>   串口设备（--device 模式必填）
#   -b, --baud <波特率>   默认 115200（必须与下位机一致；设错的表现是"一个字节都收不到"）
#       --pty             --pty 模式：造伪串口并周期发送合成帧
#       --rate <Hz>       --pty 模式的发送频率，默认 33.3（与真实轮转频率一致）
#       --vx/--vy/--wz/--yaw   --pty 模式合成的四个字段（m/s, m/s, rad/s, 度）；
#                           --yaw 合成的是 chassis_to_gimbal_yaw（底盘相对云台）
#       --rate-imu        --pty 模式合成的云台陀螺仪 yaw 转速（rad/s，默认 1.0）
#       --legacy          --pty 模式发旧帧（data_length=16，没有云台转速）
#       --seq-mode <inc|fixed>  流水号模式（inc=逐帧自增，默认；fixed=恒 0，用于排查）
#       --seconds <秒>    --pty 模式运行时长，默认 0=一直跑
#       --count <N>       收到 N 帧后退出（--device / --pty 均可用；0=不限）
#       --raw             打印每帧的原始 hex（排查错位/CRC 用，会刷屏）
#       --self-test       只跑协议自检
#   -h, --help            显示本帮助
#
# 退出码: 0 正常；1 自检失败/串口打不开；2 用法错误
# =============================================================

import argparse
import math
import os
import struct
import sys
import time

SOF = 0xA5
CRC8_INIT = 0xFF
CRC16_INIT = 0xFFFF

#: 与下位机逐字节一致的 CRC 表（协议 §2.2）。
_CRC8_TAB = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
]

_CRC16_TAB = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
]


def crc8(data, init=CRC8_INIT):
    crc = init
    for byte in data:
        crc = _CRC8_TAB[crc ^ byte]
    return crc & 0xFF


def crc16(data, init=CRC16_INIT):
    crc = init
    for byte in data:
        crc = (crc >> 8) ^ _CRC16_TAB[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF


def build_frame(cmd_id, payload, seq=0):
    """按协议 §4.1 组一帧（下→上）。"""
    header = struct.pack("<BHBB", SOF, len(payload), seq & 0xFF, 0)
    header = header[:4] + bytes([crc8(header[:4])])
    body = header + struct.pack("<H", cmd_id) + payload
    return body + struct.pack("<H", crc16(body))


#: 0x0010 的数据段：vx, vy, wz, chassis_to_gimbal_yaw(度)[, rate_imu]。
#: 给 4 个参数 = 旧帧（16 字节）；给 5 个 = 新帧（20 字节，方案 §4.4.1）。
def build_chassis_speed(vx, vy, wz, chassis_to_gimbal_yaw_deg, rate_imu=None):
    if rate_imu is None:
        return struct.pack("<ffff", vx, vy, wz, chassis_to_gimbal_yaw_deg)
    return struct.pack("<fffff", vx, vy, wz, chassis_to_gimbal_yaw_deg, float(rate_imu))


PKT_GAME_STATUS = 0x0001
PKT_ROBOT_STATUS = 0x000B
PKT_CHASSIS_SPEED = 0x0010
LEN_GAME_STATUS = 11
LEN_ROBOT_STATUS = 13
LEN_CHASSIS_SPEED = 20          # 2026-11-08 起；旧帧仍按 LEN_CHASSIS_SPEED_V1 解析
LEN_CHASSIS_SPEED_V1 = 16


def self_test():
    """协议自检：CRC 参考向量 + 帧长 + 0x0010 编解码往返。"""
    failures = []

    def check(ok, message):
        print(("[PASS] " if ok else "[FAIL] ") + message)
        if not ok:
            failures.append(message)

    # 文档 §5 的三条参考向量（上→下 19 字节帧，用来钉住 CRC 实现本身）。
    # 这里只校验 CRC 表/初值，不校验 0x0010 的组帧（下→上的参考向量由本测试现算）。
    ref = bytes.fromhex("A50C0001 26 0000803F 00000000 00000000 6BA3".replace(" ", ""))
    check(len(ref) == 19, "上→下参考帧长度应为 19")
    check(ref[:4] == bytes.fromhex("A50C0001"), "上→下参考帧头应为 A5 0C 00 01")
    check(crc8(ref[:4]) == ref[4], "CRC8 参考向量: 0x%02X == 0x%02X" % (crc8(ref[:4]), ref[4]))
    check(crc16(ref[:17]) == struct.unpack("<H", ref[17:])[0],
          "CRC16 参考向量: 0x%04X == 0x%04X"
          % (crc16(ref[:17]), struct.unpack("<H", ref[17:])[0]))

    # 0x0010：29 字节整帧、数据段 20 字节、CRC16 覆盖范围包含 cmd_id。
    payload = build_chassis_speed(0.4, -0.1, 0.75, 45.0, 1.25)
    frame = build_frame(PKT_CHASSIS_SPEED, payload, seq=7)
    check(len(frame) == 9 + LEN_CHASSIS_SPEED, "0x0010 整帧应为 29 字节（实得 %d）" % len(frame))
    check(frame[0] == SOF, "帧首应为 0xA5")
    check(struct.unpack("<H", frame[1:3])[0] == LEN_CHASSIS_SPEED,
          "data_length 应为 20（新帧）")
    check(frame[3] == 7, "seq 应为 7")
    check(crc8(frame[:4]) == frame[4], "CRC8 应覆盖前 4 字节")
    check(struct.unpack("<H", frame[5:7])[0] == PKT_CHASSIS_SPEED, "cmd_id 应小端为 0x0010")
    check(crc16(frame[:len(frame) - 2]) == struct.unpack("<H", frame[-2:])[0],
          "CRC16 应覆盖帧首到数据段末尾（含 cmd_id）")
    # 反例：把 cmd_id 改掉而不重算 CRC16 → 必须校验失败（证明 cmd_id 在覆盖范围内）。
    broken = bytearray(frame)
    broken[5] ^= 0xFF
    check(crc16(bytes(broken[:len(broken) - 2])) != struct.unpack("<H", frame[-2:])[0],
          "改动 cmd_id 必须导致 CRC16 失败（即 cmd_id 在 CRC16 覆盖范围内）")
    # 解码往返（新帧 5 个 float32）
    vx, vy, wz, yaw, rate_imu = struct.unpack("<fffff", frame[7:7 + LEN_CHASSIS_SPEED])
    check((abs(vx - 0.4) < 1e-6 and abs(vy + 0.1) < 1e-6 and abs(wz - 0.75) < 1e-6
           and abs(yaw - 45.0) < 1e-6 and abs(rate_imu - 1.25) < 1e-6),
          "0x0010 新帧解码往返一致")
    # 旧帧兼容：25 字节整帧、16 字节数据段，前 4 个字段位置不变。
    legacy = build_frame(PKT_CHASSIS_SPEED, build_chassis_speed(0.4, -0.1, 0.75, 45.0), seq=7)
    check(len(legacy) == 9 + LEN_CHASSIS_SPEED_V1,
          "0x0010 旧帧应为 25 字节（实得 %d）" % len(legacy))
    check(struct.unpack("<H", legacy[1:3])[0] == LEN_CHASSIS_SPEED_V1,
          "旧帧 data_length 应为 16")

    # 另外两包只校验长度，不解析（本车 RefereeInit 未调用，内容恒 0）。
    check(9 + LEN_GAME_STATUS == 20, "0x0001 整帧应为 20 字节")
    check(9 + LEN_ROBOT_STATUS == 22, "0x000B 整帧应为 22 字节")

    print()
    print("结论: %s" % ("全部通过" if not failures else "%d 项失败" % len(failures)))
    return 0 if not failures else 1


class FrameParser:
    """滑动窗口解析器（与 ROS 节点里的实现同一套规则）。

    刻意**按 cmd_id 分派**：三个包按 10 ms 槽位轮转，DMA 忙时该槽会被跳过，
    任何"第几个包"的假设都会在丢一个槽之后永久错位。
    """

    def __init__(self):
        self.buf = bytearray()
        self.stats = {
            "bytes": 0,
            "frames": 0,
            "crc8_fail": 0,
            "crc16_fail": 0,
            "bad_length": 0,
            "dropped_bytes": 0,
            "by_id": {},
        }

    def feed(self, data):
        self.stats["bytes"] += len(data)
        self.buf.extend(data)
        out = []
        while len(self.buf) >= 5:
            if self.buf[0] != SOF:
                self.buf.pop(0)
                self.stats["dropped_bytes"] += 1
                continue
            header = bytes(self.buf[:5])
            if crc8(header[:4]) != header[4]:
                self.buf.pop(0)
                self.stats["crc8_fail"] += 1
                self.stats["dropped_bytes"] += 1
                continue
            data_length = struct.unpack("<H", header[1:3])[0]
            if data_length > 2048:
                self.buf.pop(0)
                self.stats["bad_length"] += 1
                continue
            total = 9 + data_length
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            if crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
                self.buf.pop(0)
                self.stats["crc16_fail"] += 1
                self.stats["dropped_bytes"] += 1
                continue
            del self.buf[:total]
            cmd_id = struct.unpack("<H", frame[5:7])[0]
            payload = frame[7:7 + data_length]
            self.stats["frames"] += 1
            self.stats["by_id"][cmd_id] = self.stats["by_id"].get(cmd_id, 0) + 1
            out.append((cmd_id, payload, frame))
        return out

    @staticmethod
    def decode(payload):
        """只解析 0x0010；其它包返回 None（本车裁判系统包恒为 0，解析无意义）。

        新旧帧都认（2026-11-08，方案 §4.4.1）：旧帧 16 字节没有两路云台转速，
        此时 `gimbal_yaw_rate_imu` 为 None（不可用，而不是 0）。
        """
        if len(payload) == LEN_CHASSIS_SPEED:
            vx, vy, wz, chassis_to_gimbal_yaw, rate_imu = struct.unpack("<fffff", payload)
            return {
                "vx": vx, "vy": vy, "wz": wz,
                "chassis_to_gimbal_yaw_deg": chassis_to_gimbal_yaw,
                "gimbal_yaw_rate_imu": rate_imu,
                "legacy": False,
            }
        if len(payload) == LEN_CHASSIS_SPEED_V1:
            vx, vy, wz, chassis_to_gimbal_yaw = struct.unpack("<ffff", payload)
            return {
                "vx": vx, "vy": vy, "wz": wz,
                "chassis_to_gimbal_yaw_deg": chassis_to_gimbal_yaw,
                "gimbal_yaw_rate_imu": None,
                "legacy": True,
            }
        return None


def run_device(args):
    try:
        import termios
        import tty
    except ImportError:  # pragma: no cover
        print("错误: 需要 termios（Linux 专用）", file=sys.stderr)
        return 1

    try:
        fd = os.open(args.device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        print("错误: 打不开串口 %s: %s" % (args.device, error), file=sys.stderr)
        print("提示: 只读打开也需要读写权限（dialout 组）。", file=sys.stderr)
        return 1

    attrs = termios.tcgetattr(fd)
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag: 8N1, 无流控
    attrs[3] = 0  # lflag: raw
    baud = {9600: termios.B9600, 115200: termios.B115200, 921600: termios.B921600,
            1000000: termios.B1000000, 2000000: termios.B2000000}.get(args.baud)
    if baud is None:
        print("错误: 不支持的波特率 %d（支持 9600/115200/921600/1M/2M）" % args.baud, file=sys.stderr)
        return 2
    attrs[4] = attrs[5] = baud
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    tty.setraw(fd)

    parser = FrameParser()
    last_report = time.time()
    last_speed = None
    start = time.time()
    got = 0
    print("[只读] 监听 %s (%d 8N1)。Ctrl-C 退出。" % (args.device, args.baud))
    try:
        while True:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                chunk = b""
            except OSError as error:
                print("错误: 读串口失败: %s" % error, file=sys.stderr)
                return 1
            for cmd_id, payload, frame in parser.feed(chunk):
                if args.raw:
                    print("[RECV] ID=0x%04X len=%d: %s"
                          % (cmd_id, len(payload), frame.hex(" ")))
                if cmd_id == PKT_CHASSIS_SPEED:
                    decoded = parser.decode(payload)
                    if decoded is not None:
                        last_speed = decoded
                        got += 1
                        if args.count and got >= args.count:
                            print_last(last_speed, parser, got)
                            return 0
            now = time.time()
            if now - last_report >= 2.0:
                last_report = now
                elapsed = max(now - start, 1e-6)
                print("[统计] %.1fs: 帧 %d (%.1f 帧/s), 0x0010 %d; CRC8失败 %d, CRC16失败 %d, "
                      "长度异常 %d, 丢弃字节 %d"
                      % (elapsed, parser.stats["frames"], parser.stats["frames"] / elapsed,
                         parser.stats["by_id"].get(PKT_CHASSIS_SPEED, 0),
                         parser.stats["crc8_fail"], parser.stats["crc16_fail"],
                         parser.stats["bad_length"], parser.stats["dropped_bytes"]))
                if last_speed is not None:
                    if last_speed["gimbal_yaw_rate_imu"] is None:
                        rates = "（旧帧：云台转速不可用）"
                    else:
                        rates = ("gimbal_yaw_rate=%+.3f rad/s"
                                 % last_speed["gimbal_yaw_rate_imu"])
                    print("        最新 0x0010: vx=%+.3f m/s vy=%+.3f m/s wz=%+.3f rad/s "
                          "chassis_to_gimbal_yaw=%+.2f deg %s"
                          % (last_speed["vx"], last_speed["vy"], last_speed["wz"],
                             last_speed["chassis_to_gimbal_yaw_deg"], rates))
                else:
                    print("        还没收到 0x0010 —— 检查下位机是否在发、波特率、TX/RX 是否接反。")
                if args.seconds and elapsed >= args.seconds:
                    print_last(last_speed, parser, got)
                    return 0
            if not chunk:
                time.sleep(0.005)
    except KeyboardInterrupt:
        print()
        print_last(last_speed, parser, got)
        return 0
    finally:
        os.close(fd)


def print_last(speed, parser, got):
    if speed is None:
        print("[结果] 没有收到任何 0x0010 帧。")
        return
    deg = speed["chassis_to_gimbal_yaw_deg"]
    # 顺手给出 URDF gimbal_yaw_joint 的关节角（云台相对底盘 = 取反），方便和话题对照。
    joint_rad = math.atan2(math.sin(-math.radians(deg)), math.cos(-math.radians(deg)))
    print("[结果] 共解出 0x0010 帧 %d 个；最新: vx=%+.3f vy=%+.3f wz=%+.3f "
          "chassis_to_gimbal_yaw=%+.2f deg (gimbal_yaw_joint=%+.4f rad)"
          % (got, speed["vx"], speed["vy"], speed["wz"], deg, joint_rad))
    if speed["gimbal_yaw_rate_imu"] is None:
        print("[结果] 这是旧帧（data_length=16）：云台转速不可用，"
              "fake_vel_transform 请用 yaw_rate_source=imu")
    else:
        print("[结果] 云台陀螺仪转速 gimbal_yaw_rate=%+.3f rad/s（相对地面）"
              % speed["gimbal_yaw_rate_imu"])
    print("[统计] 总帧 %d, CRC8失败 %d, CRC16失败 %d, 丢弃字节 %d"
          % (parser.stats["frames"], parser.stats["crc8_fail"], parser.stats["crc16_fail"],
             parser.stats["dropped_bytes"]))


def run_pty(args):
    """造伪串口并按真实频率发合成帧（无硬件时验证 ROS 节点用）。"""
    import pty

    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    print("从端设备: %s" % slave_name)
    print("另开终端:")
    print("  ros2 run srm27_nav_protocol srm27_nav_protocol_node --ros-args -p device_name:=%s"
          % slave_name)
    print("  ros2 topic echo /chassis_speed     # TwistStamped")
    print("  ros2 topic echo /chassis_to_gimbal_yaw_deg   # Float32 (度)")
    print("  ros2 topic echo /gimbal_joint_angle          # Float32 (rad)")
    print("  ros2 topic echo /gimbal_yaw_rate             # TwistStamped (云台陀螺仪转速)")
    print("Ctrl-C 退出。")

    seq = 0
    period = 1.0 / args.rate if args.rate > 0 else 0.03
    start = time.time()
    try:
        while True:
            # 默认发**新帧**（20 字节，含云台转速）；--legacy 可以发旧帧验证兼容路径。
            if args.legacy:
                payload = build_chassis_speed(args.vx, args.vy, args.wz, args.yaw)
            else:
                payload = build_chassis_speed(
                    args.vx, args.vy, args.wz, args.yaw, args.rate_imu)
            frame = build_frame(PKT_CHASSIS_SPEED, payload,
                                seq if args.seq_mode == "inc" else 0)
            if args.raw:
                print("[SEND] " + frame.hex(" "))
            try:
                os.write(master, frame)
            except OSError as error:
                print("错误: 写伪串口失败: %s" % error, file=sys.stderr)
                return 1
            seq = (seq + 1) & 0xFF
            if args.seconds and time.time() - start >= args.seconds:
                print("[结果] 已发送 %d 帧后退出。" % (seq if args.seq_mode == "inc" else 0))
                return 0
            time.sleep(period)
    except KeyboardInterrupt:
        print()
        print("[结果] 已停止发送。")
        return 0
    finally:
        os.close(master)
        os.close(slave)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="SRM 下位机→上位机反馈帧测试（当前只解析 0x0010 底盘速度包）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  %(prog)s --self-test\n"
               "  %(prog)s --device /dev/ttyACM0\n"
               "  %(prog)s --pty --vx 0.4 --vy 0.1 --wz 0.8 --yaw 45\n",
    )
    parser.add_argument("-d", "--device", help="串口设备路径（--device 模式必填）")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument("--pty", action="store_true", help="造伪串口并周期发送合成 0x0010 帧")
    parser.add_argument("--rate", type=float, default=33.3, help="--pty 发送频率，默认 33.3 Hz")
    parser.add_argument("--vx", type=float, default=0.4, help="--pty 合成的 vx (m/s)")
    parser.add_argument("--vy", type=float, default=-0.1, help="--pty 合成的 vy (m/s)")
    parser.add_argument("--wz", type=float, default=0.8, help="--pty 合成的 wz (rad/s)")
    parser.add_argument("--rate-imu", type=float, default=1.0,
                        help="--pty: 云台陀螺仪 yaw 转速 (rad/s, 相对地面)")
    parser.add_argument("--legacy", action="store_true",
                        help="--pty: 发旧帧（data_length=16，没有云台转速）")
    parser.add_argument("--yaw", type=float, default=45.0,
                        help="--pty 合成的 chassis_to_gimbal_yaw（底盘相对云台, 度）")
    parser.add_argument("--seq-mode", choices=("inc", "fixed"), default="inc",
                        help="流水号模式（inc=逐帧自增）")
    parser.add_argument("--seconds", type=float, default=0.0, help="运行时长（0=一直跑）")
    parser.add_argument("--count", type=int, default=0, help="收到 N 帧后退出（0=不限）")
    parser.add_argument("--raw", action="store_true", help="打印每帧原始 hex")
    parser.add_argument("--self-test", action="store_true", help="只跑协议自检")
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()
    if args.pty:
        return run_pty(args)
    if args.device:
        return run_device(args)
    parser.print_help()
    print("\n错误: 需要 --self-test / --device <路径> / --pty 之一", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
