#!/usr/bin/env bash
# =============================================================
# 一键判断「下位机速度反馈 → /chassis_speed」断在哪一段
#
# 背景：ros2 topic hz /chassis_speed 没输出，只有两种可能
#   A. 上位机这段断了：节点没起、串口没打开、解析/发布有问题
#   B. 下位机根本没发 0x0010：固件还是老版本，或波特率/接线不对
#
# 本脚本只隔离验证 A：用**伪串口**(pty) 按 33.3 Hz 喂合成的 0x0010 帧给
# **真节点**，然后看 /chassis_speed 出不出 33 Hz，顺带核对云台偏角两条话题
# （/chassis_to_gimbal_yaw_deg 与 /gimbal_joint_angle）的值与符号。
#   · 出 33 Hz  -> 上位机链路 OK，问题在串口/下位机（B），去查固件
#   · 没出      -> 上位机自己有问题（A），脚本会把节点日志打出来
#
# 隔离措施（可以放心在跑车的时候跑）：
#   · 用独立 ROS_DOMAIN_ID（默认 88），不和你正在跑的导航栈抢话题；
#   · 节点接的是伪串口，**不碰 /dev/ttyACM0**，不影响正在跑的车；
#   · 退出时清进程，不改动你的 ROS_DOMAIN_ID。
#
# 用法:
#   ./script/check_chassis_speed_chain.sh              # 上位机自检
#   ./script/check_chassis_speed_chain.sh --domain 91  # 换一个隔离用的 domain
#   ./script/check_chassis_speed_chain.sh -h
#
# 下位机那一段（真串口、只读、不需要 ROS）:
#   python3 script/test_chassis_speed_feedback.py --device /dev/ttyACM0
#     每秒打印一行；其中 "0x0010 N" 的 N 恒为 0 => 固件还没发这个包，
#     这时上位机怎么改都没用，先去改固件（协议见 docs/导航通信协议-下位机接口说明.md §4）。
#
# 退出码: 0 = 上位机链路正常；1 = 上位机链路有问题；2 = 用法/环境错误
# =============================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# ---------- 环境消毒 ----------
# 海康 MVS 客户端会把 /opt/MVS/lib/64 前置进 LD_LIBRARY_PATH（老 libusb 缺符号，
# 会让链接 PCL 的节点以 exit 127 静默退出）。本脚本起测试节点、调 ros2 CLI，
# 环境保持一致更安全。只影响本脚本及其子进程。
# 关掉: SRM_KEEP_MVS_LIB_PATH=1
. "$SCRIPT_DIR/lib/srm_lib_path_guard.sh"

TEST_DOMAIN=88
KEEP_LOG=0

usage() {
  sed -n '2,32p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage ;;
    --domain) TEST_DOMAIN="${2:?--domain 需要参数}"; shift 2 ;;
    --keep-log) KEEP_LOG=1; shift ;;
    *) echo "未知参数: $1（-h 看用法）" >&2; exit 2 ;;
  esac
done

say()  { echo "[检查] $*"; }
ok()   { echo "  [OK]   $*"; }
bad()  { echo "  [FAIL] $*"; }
warn() { echo "  [WARN] $*"; }

# ----------------------------------------------------------- 0. 环境
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[错误] 找不到 /opt/ros/humble/setup.bash" >&2; exit 2
fi
# ROS 的 setup.* 会引用未定义变量，source 期间必须关掉 nounset
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE_DIR/install/setup.bash"
else
  echo "[错误] 工作空间还没构建: $WORKSPACE_DIR/install/setup.bash 不存在" >&2
  echo "       先 colcon build --packages-select srm27_nav_protocol" >&2
  exit 2
fi
set -u

PTY_TOOL="$SCRIPT_DIR/test_chassis_speed_feedback.py"
PARAM_FILE="$WORKSPACE_DIR/src/srm27_nav_protocol/config/srm27_nav_protocol.yaml"
NODE_PKG_DIR="$WORKSPACE_DIR/install/srm27_nav_protocol"
for f in "$PTY_TOOL" "$PARAM_FILE"; do
  [[ -f "$f" ]] || { echo "[错误] 缺少文件: $f" >&2; exit 2; }
done
[[ -d "$NODE_PKG_DIR" ]] || {
  echo "[错误] srm27_nav_protocol 没装到 install/，先 colcon build" >&2; exit 2; }

TMP_DIR="$(mktemp -d)"
PTY_LOG="$TMP_DIR/pty.log"
NODE_LOG="$TMP_DIR/node.log"
NODE_PID=""
PTY_PID=""

cleanup() {
  [[ -n "$NODE_PID" ]] && kill "$NODE_PID" 2>/dev/null
  [[ -n "$PTY_PID" ]] && kill "$PTY_PID" 2>/dev/null
  wait 2>/dev/null
  if [[ "$KEEP_LOG" == "1" ]]; then
    echo "[日志] 保留在 $TMP_DIR"
  else
    rm -rf "$TMP_DIR"
  fi
}
trap cleanup EXIT

# ----------------------------------------------------------- 1. 伪串口
say "1/4 起伪串口（合成 0x0010: vx=0.42 vy=0.05 wz=0.31 chassis_to_gimbal_yaw=45° 与云台转速, 33.3 Hz）"
PYTHONUNBUFFERED=1 python3 -u "$PTY_TOOL" --pty \
  --vx 0.42 --vy 0.05 --wz 0.31 --yaw 45 --rate-imu 1.0 \
  --seconds 40 >"$PTY_LOG" 2>&1 &
PTY_PID=$!

PTY_DEV=""
for _ in $(seq 1 50); do
  PTY_DEV="$(grep -o '/dev/pts/[0-9]*' "$PTY_LOG" | head -1 || true)"
  [[ -n "$PTY_DEV" ]] && break
  sleep 0.1
done
if [[ -z "$PTY_DEV" ]]; then
  bad "伪串口没起来（日志见下）"; sed 's/^/         /' "$PTY_LOG" | head -10
  exit 1
fi
ok "伪串口: $PTY_DEV"

# ----------------------------------------------------------- 2. 真节点
say "2/4 用真节点接伪串口（ROS_DOMAIN_ID=$TEST_DOMAIN，不碰 /dev/ttyACM0）"
ROS_DOMAIN_ID="$TEST_DOMAIN" ros2 run srm27_nav_protocol srm27_nav_protocol_node \
  --ros-args -r __node:=srm27_nav_protocol --params-file "$PARAM_FILE" \
  -p device_name:="$PTY_DEV" >"$NODE_LOG" 2>&1 &
NODE_PID=$!

NODE_READY=0
for _ in $(seq 1 60); do
  if ROS_DOMAIN_ID="$TEST_DOMAIN" ros2 node list 2>/dev/null | grep -q srm27_nav_protocol; then
    NODE_READY=1; break
  fi
  kill -0 "$NODE_PID" 2>/dev/null || break
  sleep 0.2
done
if [[ "$NODE_READY" != "1" ]]; then
  bad "节点没起来（日志见下）"; sed 's/^/         /' "$NODE_LOG" | tail -10
  exit 1
fi
ok "节点已就绪"

BOOT_OK=0
BOOT_WARN=""
if grep -q "接收侧 0x0010" "$NODE_LOG"; then BOOT_OK=1; fi
if grep -q "publish_chassis_speed=false" "$NODE_LOG"; then
  BOOT_WARN="节点把速度反馈关了（yaml 里 publish_chassis_speed: false）"
fi
[[ "$BOOT_OK" == "1" ]] && ok "启动日志确认接收侧 0x0010 已开启" \
                         || warn "启动日志里没看到接收侧 0x0010（可能被关掉了）"

# ----------------------------------------------------------- 3. 量频率
say "3/4 量 /chassis_speed 频率（约 6 s）"
HZ_LOG="$TMP_DIR/hz.log"
ROS_DOMAIN_ID="$TEST_DOMAIN" timeout 8 ros2 topic hz /chassis_speed --window 20 \
  >"$HZ_LOG" 2>&1
RATE="$(grep -o 'average rate: [0-9.]*' "$HZ_LOG" | tail -1 | awk '{print $3}')"

COUNT_LINE="$(grep -o '累计 [0-9]* 帧' "$NODE_LOG" | tail -1 || true)"

echo
if [[ -n "$RATE" ]]; then
  ok "/chassis_speed 频率 = ${RATE} Hz  （期望 ≈ 33.2 Hz）"
  if grep -q "长度不符" "$NODE_LOG"; then
    warn "节点报过「0x0010 长度不符」：固件里 data_length 既不是 20（新帧）也不是 16（旧帧）"
  fi
  if grep -q "含 NaN/Inf" "$NODE_LOG"; then
    warn "节点报过「含 NaN/Inf」：下位机算出了非法值"
  fi

  # ------------------------------------------------- 4. 云台偏角/关节角
  say "4/4 核对云台关节角（合成 chassis_to_gimbal_yaw=45°，默认 sign=-1）"
  DEG="$(ROS_DOMAIN_ID="$TEST_DOMAIN" timeout 6 \
    ros2 topic echo /chassis_to_gimbal_yaw_deg --once --field data 2>/dev/null \
    | head -1 | tr -d '\r' || true)"
  JOINT="$(ROS_DOMAIN_ID="$TEST_DOMAIN" timeout 6 \
    ros2 topic echo /gimbal_joint_angle --once --field data 2>/dev/null \
    | head -1 | tr -d '\r' || true)"
  SIGN="$(sed -n 's/^[[:space:]]*gimbal_joint_sign:[[:space:]]*//p' "$PARAM_FILE" \
    | head -1 | tr -d ' \r')"
  SIGN="${SIGN:--1.0}"
  EXPECT_JOINT="$(awk -v s="$SIGN" 'BEGIN { printf "%.4f", s * 45 * 3.14159265358979 / 180 }')"

  if [[ -z "$DEG" || -z "$JOINT" ]]; then
    bad "云台偏角话题没发出来（度数='${DEG:-缺失}' 关节角='${JOINT:-缺失}'）"
    echo
    echo "结论: 【上位机链路有问题】/chassis_speed 正常，但云台关节角话题缺失。"
    echo "      这两个话题是 base_link -> gimbal_yaw 的 TF 数据源，缺了导航会一直"
    echo "      报 TF 查询失败。最常见原因：install/ 还是旧二进制。"
    echo "        colcon build --packages-select srm27_nav_protocol"
    exit 1
  fi
  ok "/chassis_to_gimbal_yaw_deg = $DEG  （期望 45）"
  if awk -v v="$JOINT" -v e="$EXPECT_JOINT" \
      'BEGIN { exit (v - e < 5e-3 && e - v < 5e-3) ? 0 : 1 }'; then
    ok "/gimbal_joint_angle = $JOINT rad  （期望 $EXPECT_JOINT = sign $SIGN × 45 × π/180）"
  else
    warn "/gimbal_joint_angle = $JOINT rad，按 yaml 的 sign=$SIGN 期望 $EXPECT_JOINT rad"
    warn "  差 2×45° 通常是 sign 接反；若 yaml 没改过，先看是不是旧二进制"
  fi

  echo
  echo "结论: 【上位机链路正常】0x0010 一到位就能发出来，问题在串口或下位机那头。"
  echo "      下一步（真车、只读、不需要 ROS）:"
  echo "        python3 script/test_chassis_speed_feedback.py --device /dev/ttyACM0"
  echo "      看输出里 '0x0010 N' 的 N："
  echo "        · N 一直是 0        -> 固件还没发这个包（最常见），去改固件"
  echo "        · N 有、但频率很低  -> 串口带宽/轮转槽位不够"
  echo "        · 一个帧都没有      -> 串口没通（波特率/接线/权限/设备名）"
  echo "      实时波形: ./debug_tools/plot_vxvywz.py"
  exit 0
fi

bad "/chassis_speed 没有频率输出"
[[ -n "$BOOT_WARN" ]] && warn "$BOOT_WARN"
echo
echo "节点日志（最后 15 行）:"
sed 's/^/         /' "$NODE_LOG" | tail -15
echo
echo "结论: 【上位机链路有问题】伪串口喂了 33.3 Hz 的 0x0010，节点却没发话题。"
echo "      常见原因："
echo "        · install/ 是旧的 -> colcon build --packages-select srm27_nav_protocol"
echo "        · yaml 里 publish_chassis_speed: false"
echo "        · 串口没打开（上面日志里找 ERROR/串口）"
exit 1
