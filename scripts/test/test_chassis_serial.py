#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# SRM 底盘串口直连测试: 绕过 ROS / 导航栈, 直接向下位机 C 板发速度帧
#
# 用途:
#   判断"串口 → 下位机 → 电机"这条链路通不通, 并核对两个方向的报文是否
#   与《SRM 校内赛导航通信协议》一致。不依赖 ROS、不需要 source 工作空间、
#   不需要 pyserial, 只用 Python 标准库 + termios 直接操作串口。
#
#   默认动作: 发 0.2 的速度指令, 持续 2 秒, 然后发零速停车。
#
# ⚠ 安全须知 (务必先读):
#   1) 协议没有超时保护: 下位机一直使用最后一次收到的有效指令。发出去之后
#      车会一直按这个速度跑。所以本脚本在**任何**退出路径(正常结束 /
#      Ctrl-C / SIGTERM / SIGHUP / 异常)都会强制发零速帧, 并把最后一次
#      收到的速度清零后才关闭串口。
#   2) 尽管如此, 请务必把车架起来(轮子离地)或让人守着物理急停再做这个测试。
#      默认会等待 3 秒倒计时, 给你反应时间 (用 -y 跳过)。
#   3) 如果脚本被 kill -9 或断电, 零速帧不会发出, 车会保持原速度 —— 物理
#      急停始终是第一手段。也可以单独发零速: 本脚本 --stop-only
#
# 协议一致性 (对应《SRM 校内赛导航通信协议》):
#   上→下 (§2): 固定 19 字节 = 帧头 5 + 数据 12 + CRC16 2, data_length 必须为 12。
#                本方向没有 cmd_id —— 帧头之后直接是数据段。
#   下→上 (§4): 本脚本**只往下发, 不解析任何反馈** —— 收到什么直接丢掉,
#                只是为了不让 tty 接收缓冲堆满。要看回传请用
#                script/test_chassis_speed_feedback.py 或 srm27_nav_protocol。
#   CRC   (§2.2): CRC8(帧头前 4 字节, init 0xFF) / CRC16(帧头+数据段, init 0xFFFF)。
#   §3.3 量级: 上位机 1.0 ≈ 四分之一杆, 起步建议 ≤ 0.2。协议层不限幅、限幅是上位机的
#              责任; 本脚本**默认不限幅**(指令原样下发), 需要保守时才显式 --max-speed。
#   §6   发送: 100 Hz 定频单帧(默认), 帧间有空闲; 不发 NaN/Inf; 退出补零速。
#
# 自检 (协议 §5): 本脚本内置协议给出的三条参考向量与两个 CRC 自检值,
#   每次运行前自动核对。要单独跑:
#       ./script/test_chassis_serial.py --self-test
#   自检不通过就不该上机 —— 说明 CRC 表或打包函数与协议不符。
#
# 方向约定 (协议 §3.2 有 90° 轴向交叉, 重要):
#   下位机会把 vx 送到底盘 y 轴、把 vy 取负送到底盘 x 轴, wz 当前被忽略。
#   也就是说本脚本的 --direction forward (vx>0) **不一定**让车往前跑, 可能
#   是侧向。协议明确要求"逐轴实测确认, 不要靠推断"。
#   因此:
#     · 想严格按协议字段发, 直接用 --vx / --vy / --wz (最可靠);
#     · --direction 只是便捷写法, 它的名字描述的是**协议字段**, 不是车体方向;
#     · 实测发现方向反了用 --flip-y (翻转 vy 符号)。
#
# 用法:
#   ./script/test_chassis_serial.py                     # vx=0, vy=-0.2, 2 秒, 然后停车
#   ./script/test_chassis_serial.py --vx 0.2            # 直接指定协议字段
#   ./script/test_chassis_serial.py --vy -0.1 --wz 0.5  # 含转向指令
#   ./script/test_chassis_serial.py --duration 1.0      # 只跑 1 秒
#   ./script/test_chassis_serial.py --self-test         # 只做协议自检, 不碰串口
#   ./script/test_chassis_serial.py --dry-run           # 打印帧, 不开串口
#   ./script/test_chassis_serial.py --stop-only         # 只发零速(紧急停车)
#   ./script/test_chassis_serial.py --list-devices      # 列出候选串口设备
#   ./script/test_chassis_serial.py -y                  # 跳过 3 秒倒计时
#
# 参数:
#   -d, --device <路径>   串口设备 (默认自动选 /dev/serial/by-id/ 下唯一设备,
#                         否则回落到 /dev/ttyACM0; 协议 §1 推荐用 by-id 路径)
#   -b, --baud <波特率>   默认 115200 (必须与下位机一致, 设错的表现是"一个字节都收不到")
#   -s, --speed <值>      平移指令幅值, 默认 0.2 (取绝对值)
#       --vx/--vy/--wz    直接指定三个协议字段, 给出后忽略 --direction/--speed
#       --duration <秒>   运动持续时间, 默认 2.0
#       --rate <Hz>       发送频率, 默认 100 (协议 §6.1 推荐值)
#       --max-speed <值>  可选的上位机钳位上限; 不给 = 不限幅 (默认)
#       --direction <方向>  vy-(默认right) / vy+(left) / vx+(forward) / vx-(back)
#       --flip-y          翻转 vy 符号 (实测方向相反时用)
#       --lead-zero <秒>  运动前先发零速的时间, 默认 0.3 (先把状态钉死)
#       --hold-zero <秒>  运动后发零速的时间, 默认 1.0 (保证车停住)
#       --stop-only       只发零速后退出
#       --dry-run         只打印帧, 不打开串口
#       --self-test       只跑协议自检 (§2.2 + §5), 不打开串口
#       --list-devices    列出 /dev/serial/by-id 与 ttyACM/ttyUSB
#   -y, --yes             跳过倒计时确认
#   -h, --help            显示本帮助
#
# 帧格式 (上→下, 固定 19 字节):
#   A5 | len=0C 00 | seq | crc8 | vx(f32) vy(f32) wz(f32) | crc16
#   - len 字段 = 数据段字节数 = 12, 必须严格等于 12, 否则下位机整帧丢弃
#     (即使 CRC 全对)。协议 §2.1 明确"只接受数据段长度为 12 的帧"。
#   - 帧头 5 字节之后**直接**是数据段, 本方向没有 cmd_id, 不要插字节。
#   - crc8 : CRC8_table, init 0xFF, 覆盖偏移 0-3, 放在偏移 4
#   - crc16: wCRC_table, init 0xFFFF, 覆盖偏移 0-16 (17 字节), 小端放在 17-18
#   两张表逐字节取自 src/srm27_nav_protocol/src/crc_func.c。
#
# 结果怎么读 (本脚本只看车动没动, 不读回传):
#   - 车动了   → 串口/接线/板子/电机/使能全通, 问题只在上位机软件层
#   - 车不动   → 查使能/模式/固件协议; 先用 --vx/--vy 确认平移链路, 再单独试 --wz
#                (要看底盘回传的 vx/vy/wz, 用 script/test_chassis_speed_feedback.py)
#
# ⚠ 如果板子上是 control-2026 a6e1638 那份固件: 它的
#   UserApp/robot/sentry_omni_chassis/robot.c:490 把 navigator_init(&huart1)
#   注释掉了, 即接收回调从未注册 —— 那种情况下无论帧多正确车都不会动,
#   必须先恢复该行并重新编译烧录 C 板。
#
# ✅ 一致性: 上位机 ROS 节点 src/srm27_nav_protocol 发的也是同样的 19 字节帧
#   (data_length=12)。两边的打包结果已逐字节比对一致, 且都等于协议 §5 的
#   三条参考向量 —— 所以"脚本能开车、ROS 节点也能开"。
#   历史提醒: 该节点曾经多发一个 is_recovering 字节 (整帧 20 字节 / len=13),
#   会被下位机静默丢弃; 现在 packet_typedef.hpp 里有 static_assert 钉死 19 字节,
#   再往数据段里加字段会直接编译失败, 不会重演。
#
# 运行前记得先让底盘节点松开串口 (它独占该设备):
#   ./script/start_real_nav.sh --stop
# =============================================================

import argparse
import errno
import glob
import math
import os
import signal
import struct
import sys
import time

try:
    import termios
except ImportError:  # pragma: no cover
    print("[错误] 本脚本依赖 POSIX termios, 只能在 Linux 上运行。", file=sys.stderr)
    raise SystemExit(2)

# ---------------------------------------------------------------
# CRC 表: 逐字节取自 src/srm27_nav_protocol/src/crc_func.c
# ---------------------------------------------------------------
CRC8_TABLE_HEX = (
    "005EBCE2613FDD83C29C7E20A3FD1F419DC3217FFCA2401E5F01E3BD3E6082DC237D9FC1421CFEA0E1BF5D0380DE3C62BEE0025CDF81633D7C22C09E1D43A1FF4618FAA427799BC584DA3866E5BB5907DB856739BAE406581947A5FB7826C49A653BD987045AB8E6A7F91B45C6987A24F8A6441A99C7257B3A6486D85B05E7B98CD2306EEDB3510F4E10F2AC2F7193CD114FADF3702ECC92D38D6F31B2EC0E50AFF1134DCE90722C6D33D18F0C52B0EE326C8ED0530DEFB1F0AE4C1291CF2D73CA947628ABF517490856B4EA6937D58B5709EBB536688AD495CB2977F4AA4816E9B7550B88D6346A2B7597C94A14F6A8742AC896154BA9F7B6E80A54D7896B35"
)
CRC16_TABLE_HEX = (
    "000011892312329B462457AD653674BF8C489DC1AF5ABED3CA6CDBE5E97EF8F7108101083393221A56A5472C75B7643E9CC98D40BFDBAE52DAEDCB64F9FFE8762102308B02101399672676AF443455BDAD4ABCC38E589FD1EB6EFAE7C87CD9F53183200A1291031877A7662E54B5453CBDCBAC429ED98F50FBEFEA66D8FDC9744204538D6116709F042015A9273236BBCE4CDFC5ED5EFCD7886899E1AB7ABAF35285430C7197601E14A1052837B3263ADECDCF44FDDFEC5698E98960BBFBAA726306728F4014519D252234AB063017B9EF4EFEC7CC5CDDD5A96AB8E38A789BF17387620E5095411C35A3242A16B10738FFCFEE46DCDDCD54B9EBA8629AF98B7084089581A71AB693C22CD3A5E13EF0B7084019C92B523ADB4E645FED6D767CFF94898500B79BA612D2ADC324F1BFE03618C109483BD32A5A5EE54F6C7DF76C7EA50AB48386189791E32EF2A7C03CD1B5294238CB0A501BD96F667EEF4C745DFDB58BA40296998710F3AFE226D0BDC13439C3284A1AD10B587FE76E6E5CF54D7CC60CD785E51EF497802891A1A33AB2B34A445BCD695678DF0C601DE92F723EFBD68DC704F59FE41690A98120B3BBA2325AC54B4C79D7685E1CE10D683FF32E7AE70EF687C41CD595A12AB0A3823893B16B467ACF485459DD2D623CEB0E701FF9F78FE606D49DC514B1ABA02292B983307BC76A4E58D5495C3DE32C6A1EF10F78"
)

CRC8_TABLE = list(bytes.fromhex(CRC8_TABLE_HEX))
CRC16_TABLE = [int(CRC16_TABLE_HEX[i:i + 4], 16) for i in range(0, len(CRC16_TABLE_HEX), 4)]

# ---------------------------------------------------------------
# 协议常量 (《SRM 校内赛导航通信协议》)
# ---------------------------------------------------------------
SOF = 0xA5
DATA_LEN = 12                 # 上→下 数据段长度, 固定 12 (§2.1)
FRAME_TOTAL = 19              # 上→下 整帧长度 = 5 + 12 + 2 (§2.1)
CRC16_UPSTREAM_OFF = 17       # 上→下 crc16 偏移 = 5 + data_length (§2.1)

# 协议 §5 参考向量 (seq = 0x01)
REF_ZERO_FRAME = "A5 0C 00 01 26 00 00 00 00 00 00 00 00 00 00 00 00 18 9F"
REFERENCE_FRAMES = (
    ("vx=1, vy=0, wz=0", 1.0, 0.0, 0.0,
     "A5 0C 00 01 26 00 00 80 3F 00 00 00 00 00 00 00 00 6B A3"),
    ("零速帧 vx=vy=wz=0", 0.0, 0.0, 0.0, REF_ZERO_FRAME),
    ("vx=1, vy=-1, wz=3", 1.0, -1.0, 3.0,
     "A5 0C 00 01 26 00 00 80 3F 00 00 80 BF 00 00 40 40 D3 96"),
)


def crc8(data, crc=0xFF):
    """协议 §2.2: crc = T8[crc ^ byte], init 0xFF, 反射。"""
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc


def crc16(data, crc=0xFFFF):
    """协议 §2.2: crc = (crc >> 8) ^ T16[(crc ^ byte) & 0xFF], init 0xFFFF, 反射。"""
    for byte in data:
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]) & 0xFFFF
    return crc


def rebuild_tables():
    """按协议 §2.2 的多项式重新生成两张表, 用于校验内嵌表没被抄错/改坏。

    CRC8 : 反向多项式 0x8C (x^8+x^5+x^4+1), 反射 / LSB-first
    CRC16: 反向多项式 0x8408 (= 0x1021 的反射形式), 反射 / LSB-first

    只验参考向量是不够的: 帧头只有 4 字节且 SOF 固定, 命中不了几个表项,
    所以某个表项被抄错时参考向量仍可能全过。这里逐项比对 256 项。
    """
    table8, table16 = [], []
    for index in range(256):
        value = index
        for _ in range(8):
            value = (value >> 1) ^ 0x8C if (value & 1) else (value >> 1)
        table8.append(value)
        value = index
        for _ in range(8):
            value = (value >> 1) ^ 0x8408 if (value & 1) else (value >> 1)
        table16.append(value)
    return table8, table16


def build_frame(vx, vy, wz, seq):
    """构造一帧 上→下 速度指令, 严格 19 字节 (协议 §2.1 / §2.3)。

    布局: A5 | data_length=12 (u16 LE) | seq | crc8 | vx vy wz (f32 LE) | crc16
    - 本方向没有 cmd_id, 帧头 5 字节之后直接是数据段。
    - crc8  覆盖偏移 0-3; crc16 覆盖偏移 0-16, 不含自身。
    """
    if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
        raise ValueError("vx/vy/wz 含 NaN/Inf, 协议 §6.4 禁止发送")
    body = struct.pack("<fff", float(vx), float(vy), float(wz))
    assert len(body) == DATA_LEN, "数据段必须严格 12 字节"
    header = struct.pack("<BH", SOF, len(body)) + bytes([seq & 0xFF])
    header += bytes([crc8(header)])                       # CRC8 覆盖帧头前 4 字节
    packet = header + body
    assert len(packet) == CRC16_UPSTREAM_OFF, "帧头+数据段必须 17 字节"
    return packet + struct.pack("<H", crc16(packet))      # CRC16 覆盖前 17 字节


def hexs(data):
    return " ".join(f"{b:02X}" for b in data)



# ---------------------------------------------------------------
# 协议自检 (协议 §2.2 自检值 + §5 参考向量)
# ---------------------------------------------------------------
def self_test(verbose=True):
    """核对 CRC 自检值、三条 19 字节参考向量、以及反馈帧解析。返回是否全部通过。"""
    failures = []
    lines = []

    gen8, gen16 = rebuild_tables()
    table8_ok = gen8 == CRC8_TABLE
    table16_ok = gen16 == CRC16_TABLE
    lines.append(f"  CRC8  表 256 项 vs 多项式 0x8C    "
                 f"{'✓' if table8_ok else '✗ 有表项与多项式不符'}")
    lines.append(f"  CRC16 表 256 项 vs 多项式 0x8408  "
                 f"{'✓' if table16_ok else '✗ 有表项与多项式不符'}")
    if not table8_ok:
        failures.append("CRC8 表与多项式不符")
    if not table16_ok:
        failures.append("CRC16 表与多项式不符")

    probe = b"123456789"
    got8, got16 = crc8(probe), crc16(probe)
    lines.append(f"  CRC8 (\"123456789\")  = 0x{got8:02X}   协议 §2.2 期望 0x0B   "
                 f"{'✓' if got8 == 0x0B else '✗'}")
    lines.append(f"  CRC16(\"123456789\")  = 0x{got16:04X} 协议 §2.2 期望 0x6F91 "
                 f"{'✓' if got16 == 0x6F91 else '✗'}")
    if got8 != 0x0B:
        failures.append("CRC8 自检值不符")
    if got16 != 0x6F91:
        failures.append("CRC16 自检值不符")

    for label, vx, vy, wz, expect in REFERENCE_FRAMES:
        frame = build_frame(vx, vy, wz, 1)                # 协议参考向量 seq=1
        ok = hexs(frame) == expect
        lines.append(f"  {label:<20} {hexs(frame)}  {'✓' if ok else '✗'}")
        if not ok:
            lines.append(f"  {'':<20} 期望 {expect}")
            failures.append(f"参考向量不符: {label}")
        if len(frame) != FRAME_TOTAL:
            failures.append(f"{label} 帧长 {len(frame)} != {FRAME_TOTAL}")

    if verbose:
        print("=" * 62)
        print(" 协议自检 (《SRM 校内赛导航通信协议》§2.2 / §5)")
        print("=" * 62)
        for line in lines:
            print(line)
        print("=" * 62)
        print("  自检通过 ✓ 打包/校验与协议逐字节一致。" if not failures
              else f"  自检失败 ✗ {len(failures)} 项: {', '.join(failures)}")
        print("=" * 62)
    return not failures


def list_devices():
    print("候选串口设备:")
    print("\n  /dev/serial/by-id/ (协议 §1 推荐用它, 避免 ttyACM 编号变化):")
    entries = sorted(glob.glob("/dev/serial/by-id/*"))
    if entries:
        for path in entries:
            try:
                print(f"    {path} -> {os.path.realpath(path)}")
            except OSError:
                print(f"    {path}")
    else:
        print("    (无)")
    print("\n  /dev/ttyACM* /dev/ttyUSB*:")
    nodes = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if nodes:
        for path in nodes:
            try:
                st = os.stat(path)
                print(f"    {path}  权限={oct(st.st_mode & 0o777)}  属主={st.st_uid}:{st.st_gid}")
            except OSError:
                print(f"    {path}")
    else:
        print("    (无)")
    print("\n  当前用户组:", end=" ", flush=True)
    os.system("id -nG")


def resolve_device(explicit):
    """协议 §1: 优先用 /dev/serial/by-id 路径, 避免重新插拔后 ttyACM 编号变化。"""
    if explicit:
        return explicit, "命令行指定"
    entries = sorted(glob.glob("/dev/serial/by-id/*"))
    if len(entries) == 1:
        return entries[0], "自动选用 by-id 下唯一设备"
    if len(entries) > 1:
        print(f"[提示] /dev/serial/by-id/ 下有 {len(entries)} 个设备, 无法自动判断哪个是下位机:",
              file=sys.stderr)
        for path in entries:
            print(f"         {path}", file=sys.stderr)
        print("[提示] 用 -d 显式指定, 或先跑 --list-devices。回落到 /dev/ttyACM0。",
              file=sys.stderr)
    return "/dev/ttyACM0", "by-id 不可用, 回落到 /dev/ttyACM0"


def _pid_cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            raw = handle.read()
    except OSError:
        return "?"
    parts = [chunk.decode("utf-8", "replace") for chunk in raw.split(b"\0") if chunk]
    return " ".join(parts) if parts else "?"


def find_serial_holders(device):
    """找出还开着同一个 tty 的其它进程。

    Linux **不会**阻止第两个进程 open 同一个 /dev/ttyACM*（没有排他锁），而 tty 的
    接收数据是"读走就没了"：两个读者各抢一半，结果两边都掉帧。所以必须在打开前
    自己查一遍 /proc/<pid>/fd，否则就会出现"下位机在发、ROS 节点收不全、
    plot_vxvywz 波形断断续续"的现象。

    返回 [(pid, cmdline, fd 指向的路径)]。
    """
    try:
        rdev = os.stat(device).st_rdev
    except OSError:
        return []
    me = os.getpid()
    holders = []
    for pid_dir in glob.glob("/proc/[0-9]*"):
        try:
            pid = int(os.path.basename(pid_dir))
        except ValueError:
            continue
        if pid == me:
            continue
        fd_dir = os.path.join(pid_dir, "fd")
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for name in fds:
            try:
                target = os.readlink(os.path.join(fd_dir, name))
            except OSError:
                continue
            if not target.startswith("/dev/"):
                continue
            try:
                if os.stat(target).st_rdev != rdev:
                    continue
            except OSError:
                continue
            holders.append((pid, _pid_cmdline(pid), target))
            break
    return holders


def open_serial(device, baud):
    """打开并配置 8N1 raw 模式 (协议 §1)。失败时给出可操作的排查提示。"""
    baud_const = getattr(termios, f"B{baud}", None)
    if baud_const is None:
        raise SystemExit(f"[错误] 不支持波特率 {baud} (协议要求 115200)")

    try:
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY)
    except FileNotFoundError:
        print(f"[错误] 设备不存在: {device}", file=sys.stderr)
        print("[提示] 用 --list-devices 看当前有哪些串口; 用 -d 指定正确设备。", file=sys.stderr)
        raise SystemExit(1)
    except PermissionError:
        print(f"[错误] 没有权限打开 {device}", file=sys.stderr)
        print("[提示] 二选一:", file=sys.stderr)
        print(f"[提示]   临时: sudo chmod 666 {device}", file=sys.stderr)
        print("[提示]   长期: sudo usermod -aG dialout $USER 后重新登录", file=sys.stderr)
        raise SystemExit(1)
    except OSError as exc:
        if exc.errno == errno.EBUSY:
            print(f"[错误] 设备被占用: {device}", file=sys.stderr)
            print("[提示] 底盘节点独占该串口, 先停掉它:", file=sys.stderr)
            print("[提示]   ./script/start_real_nav.sh --stop", file=sys.stderr)
            print("[提示]   或 pkill -f '[s]rm27_nav_protocol'", file=sys.stderr)
        else:
            print(f"[错误] 打开 {device} 失败: {exc}", file=sys.stderr)
        raise SystemExit(1)

    try:
        attrs = termios.tcgetattr(fd)
    except termios.error as exc:
        os.close(fd)
        raise SystemExit(f"[错误] {device} 不是 tty 设备? ({exc})")

    attrs[0] = 0                                              # iflag: 不做任何输入处理
    attrs[1] = 0                                              # oflag: 不做输出处理
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL    # cflag: 8N1, 无流控
    attrs[3] = 0                                              # lflag: raw, 无回显
    attrs[4] = baud_const                                     # ispeed
    attrs[5] = baud_const                                     # ospeed
    attrs[6][termios.VMIN] = 0                                # 读: 立即返回
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_available(fd, sink):
    """把串口上能读到的字节全部读走, 追加到 sink。用于判断板子有没有在发。"""
    got = 0
    while True:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            break
        except OSError:
            break
        if not chunk:
            break
        sink.extend(chunk)
        got += len(chunk)
    return got


def resolve_command(args):
    """算出要发的 (vx, vy, wz)。--vx/--vy/--wz 优先, 否则用 --direction/--speed。"""
    if args.vx is not None or args.vy is not None or args.wz is not None:
        vx = args.vx if args.vx is not None else 0.0
        vy = args.vy if args.vy is not None else 0.0
        wz = args.wz if args.wz is not None else 0.0
        label = "手动指定字段"
        if args.flip_y:
            vy = -vy
            label += "(已翻转 vy)"
        return vx, vy, wz, label

    speed = abs(args.speed)
    if args.direction == "right":       # 名字描述的是协议字段, 不是车体方向 (§3.2)
        vx, vy = 0.0, -speed
    elif args.direction == "left":
        vx, vy = 0.0, speed
    elif args.direction == "forward":
        vx, vy = speed, 0.0
    else:
        vx, vy = -speed, 0.0
    if args.flip_y:
        vy = -vy
    label = {"right": "vy<0 (right)", "left": "vy>0 (left)",
             "forward": "vx>0 (forward)", "back": "vx<0 (back)"}[args.direction]
    if args.flip_y:
        label += " 已翻转 vy"
    return vx, vy, 0.0, label


def clamp_or_die(vx, vy, wz, limit):
    """协议 §3.3 / §6.4 / §6.5: 先查 NaN/Inf; limit=None 时**不限幅**(默认)。

    2026-11-11: 默认不再限幅。原来是拿 --max-speed(默认 0.2) 把 vx/vy/wz 一起钳,
    结果 `--wz 3` 实际只发出 0.2, 看起来像"下位机转不动"。要保守测试时可以显式给
    `--max-speed <值>` 重新打开这一层钳位。
    """
    for name, value in (("vx", vx), ("vy", vy), ("wz", wz)):
        if not math.isfinite(value):
            raise SystemExit(f"[错误] {name}={value} 不是有限数, 协议 §6.4 禁止发送 NaN/Inf")
    if limit is None:
        return vx, vy, wz
    clamped = []
    for name, value in (("vx", vx), ("vy", vy), ("wz", wz)):
        if abs(value) > limit:
            print(f"[限幅] {name}={value:+.4f} 超过 --max-speed {limit:+.4f}, 已截到 "
                  f"{math.copysign(limit, value):+.4f}", file=sys.stderr)
            value = math.copysign(limit, value)
        clamped.append(value)
    return tuple(clamped)


def main():
    parser = argparse.ArgumentParser(
        add_help=False,
        description="SRM 底盘串口直连测试: 直接向下位机发速度帧, 判断串口控制通不通。",
    )
    parser.add_argument("-d", "--device", default=None)
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-s", "--speed", type=float, default=0.2)
    parser.add_argument("--vx", type=float, default=None)
    parser.add_argument("--vy", type=float, default=None)
    parser.add_argument("--wz", type=float, default=None)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=100.0,
                        help="发送频率, 协议 §6.1 推荐 100 Hz (默认)")
    parser.add_argument("--max-speed", type=float, default=None,
                        help="可选的上位机钳位上限; 不给 = 不限幅(默认, 指令原样下发)")
    parser.add_argument("--direction", choices=["right", "left", "forward", "back"], default="right")
    parser.add_argument("--flip-y", action="store_true")
    parser.add_argument("--lead-zero", type=float, default=0.3)
    parser.add_argument("--hold-zero", type=float, default=1.0)
    parser.add_argument("--stop-only", action="store_true")
    parser.add_argument("--force", action="store_true",
                        help="串口被其它进程占着也照样打开(会互相偷帧, 只用于排查)")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("-y", "--yes", action="store_true")
    parser.add_argument("-h", "--help", action="store_true")
    args = parser.parse_args()

    # 是否在终端里跑 —— 决定进度是原地刷新还是按行输出
    progress_line = sys.stdout.isatty()

    if args.help:
        # 打印文件头部注释块 (跳过 shebang 与 coding 声明)
        with open(__file__, encoding="utf-8") as stream:
            for line in stream:
                if line.startswith("#!") or "coding:" in line:
                    continue
                if not line.startswith("#"):
                    break
                print(line[2:].rstrip())
        return 0

    if args.list_devices:
        list_devices()
        return 0

    # 串口占用检查放在自检之前: 被占着就没必要再往屏幕上刷一堆自检行
    device, device_why = resolve_device(args.device)
    if not args.dry_run:
        # 同一个 tty 被两个进程同时打开时 Linux 不报错, 但两边会互相偷走接收字节
        # (表现: 下位机明明在发, ROS 节点/plot_vxvywz 的波形却断断续续)。
        holders = find_serial_holders(device)
        if holders:
            print(f"[错误] 串口 {device} 已被下列进程打开:", file=sys.stderr)
            for pid, cmdline, path in holders:
                print(f"         pid {pid}  {path}\n            {cmdline}", file=sys.stderr)
            print("[提示] 两个进程同时读同一个 tty 会互相抢字节 —— 你这边的波形断、"
                  "对方的 /chassis_speed 也会断。", file=sys.stderr)
            print("[提示] 先停掉它再测: ./script/start_real_nav.sh --stop", file=sys.stderr)
            print("[提示]   (或 pkill -f '[s]rm27_nav_protocol')", file=sys.stderr)
            print("[提示] 确认没有别的读取者后重跑; 确实要抢过来加 --force (会互相偷帧, 慎用)。",
                  file=sys.stderr)
            if not args.force:
                return 1
            print("[警告] --force: 仍然打开, 两边都会掉帧, 读到的波形不可信。",
                  file=sys.stderr)

    # 每次运行都先做协议自检 —— 自检不过就不该上机
    ok = self_test(verbose=True)
    if args.self_test:
        return 0 if ok else 1
    if not ok:
        print("[中止] 协议自检未通过, 打包函数与协议不符, 拒绝上机。", file=sys.stderr)
        return 1

    vx, vy, wz, dir_label = resolve_command(args)
    limit = None if args.max_speed is None else abs(args.max_speed)
    vx, vy, wz = clamp_or_die(vx, vy, wz, limit)

    period = 1.0 / max(args.rate, 0.1)

    print()
    print("=" * 62)
    print(" SRM 底盘串口直连测试")
    print("=" * 62)
    print(f"  设备      : {device} @ {args.baud} 8N1  ({device_why})")
    print(f"  指令      : {dir_label}   vx={vx:+.4f}  vy={vy:+.4f}  wz={wz:+.4f}")
    print(f"  时长      : {args.duration} s @ {args.rate} Hz   限幅 "
          f"{'关闭(原样下发)' if limit is None else format(limit, '.4f')}")
    print(f"  帧格式    : {FRAME_TOTAL} 字节, A5 | len={DATA_LEN:02X} 00 | seq | crc8 | "
          f"vx vy wz | crc16   (协议 §2.1)")
    if limit is None and max(abs(vx), abs(vy), abs(wz)) > 0.2:
        print("  ⚠ 无限幅  : 本次指令超过协议 §3.3 的起步建议 0.2, 且脚本/下位机都没有斜坡,")
        print("              轮子离地或留人守急停; 末尾若被 kill -9, 车会保持该速度。")
    if wz != 0.0:
        print("  注意      : 协议 §3.2 说 wz 目前被下位机忽略, 不产生转向。")
    zero_frame = build_frame(0.0, 0.0, 0.0, 1)
    move_frame = build_frame(vx, vy, wz, 1)
    print(f"  零速帧    : {hexs(zero_frame)}")
    print(f"  运动帧    : {hexs(move_frame)}")
    print(f"  零速帧核验: {'✓ 与协议 §5 参考向量逐字节一致' if hexs(zero_frame) == REF_ZERO_FRAME else '✗ 与协议不一致!'}")
    print("  方向提示  : 协议 §3.2 存在 90° 轴向交叉 (vx→底盘 y, vy 取负→底盘 x),")
    print("              上面的 forward/right 描述的是协议字段, 实际运动方向需逐轴实测。")
    print("=" * 62)

    if args.dry_run:
        print("\n[--dry-run] 只打印, 不打开串口。前 5 帧(seq 递增):")
        for seq in range(5):
            print(f"  seq={seq:02X}  {hexs(build_frame(vx, vy, wz, seq))}")
        print("\n[--dry-run] 未发送任何数据。")
        return 0

    if not args.stop_only and not args.yes:
        print("\n⚠  这个测试会让车真的动起来。请确认: 轮子已离地, 或有人守着物理急停。")
        print("    同时确认已停掉底盘节点(它独占串口): ./script/start_real_nav.sh --stop")
        for remaining in (3, 2, 1):
            print(f"   {remaining} ...", flush=True)
            time.sleep(1.0)

    fd = open_serial(device, args.baud)
    rx = bytearray()
    seq = 0
    sent = 0
    stop_requested = [False]

    def request_stop(signum, _frame):
        stop_requested[0] = True
        print(f"\n[信号] 收到信号 {signum}, 立即停车...", flush=True)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, request_stop)
        except (ValueError, OSError):
            pass

    # 本脚本只往下发: 板子回传的字节读掉就丢, 不解析、不统计、不打印。
    # 留着这个读取动作只是为了不让 tty 接收缓冲堆满 (堆满后可能触发流控, 影响发送节奏)。
    def discard_rx():
        read_available(fd, bytearray())

    def send(frame):
        nonlocal seq, sent
        seq = (seq + 1) & 0xFF
        payload = bytearray(frame)
        payload[3] = seq                                  # 更新 seq 字节
        payload[4] = crc8(bytes(payload[:4]))             # 重算帧头 CRC8
        packet = bytes(payload[:CRC16_UPSTREAM_OFF])
        packet += struct.pack("<H", crc16(packet))        # 重算整包 CRC16
        os.write(fd, packet)
        sent += 1
        discard_rx()
        return packet

    try:
        # 1) 先发零速, 把底盘状态钉死在"停" (协议 §6.3: 进程启动即开始发送)
        deadline = time.monotonic() + max(args.lead_zero, 0.0)
        while time.monotonic() < deadline:
            send(zero_frame)
            time.sleep(period)

        if args.stop_only:
            print(f"[停止] 已发送 {sent} 个零速帧, 底盘应保持静止。")
        else:
            # 2) 发运动帧
            print(f"[运动] 开始发送 ({dir_label}) vx={vx:+.4f} vy={vy:+.4f} wz={wz:+.4f} ...",
                  flush=True)
            deadline = time.monotonic() + max(args.duration, 0.0)
            while time.monotonic() < deadline and not stop_requested[0]:
                send(move_frame)
                left = deadline - time.monotonic()
                # 终端里用 \r 原地刷新进度; 重定向到文件时改成每 100 帧一行,
                # 否则日志会被回车刷成一大坨。
                if progress_line:
                    print(
                        f"\r  已发送 {sent:5d} 帧   剩余 {max(left, 0):4.1f} s",
                        end="", flush=True)
                elif sent % 100 == 0:
                    print(
                        f"  已发送 {sent:5d} 帧   剩余 {max(left, 0):4.1f} s", flush=True)
                time.sleep(period)
            if progress_line:
                print()
    finally:
        # 3) 无论正常结束、Ctrl-C 还是异常, 都必须把速度清零 (协议 §6.3)
        print("[停车] 发送零速帧...", flush=True)
        try:
            deadline = time.monotonic() + max(args.hold_zero, 0.5)
            while time.monotonic() < deadline:
                send(zero_frame)
                time.sleep(period)
            termios.tcflush(fd, termios.TCOFLUSH)
        except OSError as exc:
            print(f"[警告] 发送零速帧时出错: {exc}", file=sys.stderr)
            print("[警告] 请立即用物理急停确认底盘已停!", file=sys.stderr)
        finally:
            os.close(fd)

    print("\n" + "=" * 62)
    print(f" 共发送 {sent} 帧 (最后 {max(args.hold_zero, 0.5):.1f} s 为零速)")
    print(f" 上→下 每帧 {FRAME_TOTAL} 字节, 数据段 12 字节, @ {args.rate} Hz (协议 §2.1 / §6.1)")
    print(" 下→上: 不解析 (本脚本只往下发; 要看回传用 test_chassis_speed_feedback.py)")
    print("=" * 62)
    print(" 结果判读:")
    print("   车动了   → 串口/接线/板子/电机/使能全通, 问题在上位机软件层")
    print("   车不动   → 查使能/模式/固件协议; 先用 --vx/--vy 确认平移链路通不通")
    return 0


if __name__ == "__main__":
    sys.exit(main())
