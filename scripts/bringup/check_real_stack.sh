#!/usr/bin/env bash
# =============================================================
# 实车导航栈「起栈后 60 秒自检」——只读，不发任何速度、不改任何参数。
#
# 用法：
#   cd ~/srm_nav_27
#   source scripts/setup/env_srm27.sh
#   ./scripts/bringup/check_real_stack.sh            # 全量检查
#   ./scripts/bringup/check_real_stack.sh --quick    # 跳过频率测量（3 秒出结果）
#
# 判据顺序 = 数据流顺序：环境 → 雷达/IMU → TF → 定位输出 → 唯一 TF owner
#            → Nav2 生命周期 → 速度链 → 后端状态。
#
# 退出码：0 = 全部 PASS；1 = 有 FAIL（看输出里最后那张表）
# =============================================================

set -uo pipefail

QUICK=0
for argument in "$@"; do
  case "$argument" in
    --quick) QUICK=1 ;;
    -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "[错误] 未知参数: $argument" >&2; exit 2 ;;
  esac
done

WS_DIR="${SRM27_WS_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

declare -a ROWS=()
FAILS=0

row() { # row <PASS|FAIL|WARN|SKIP> <名称> <细节>
  ROWS+=("$1|$2|$3")
  [ "$1" = "FAIL" ] && FAILS=$((FAILS + 1))
  return 0
}

# ros2 topic hz 的稳定读数（取最后一行的 average rate）
topic_rate() {
  local topic="$1" seconds="${2:-6}"
  timeout "$seconds" ros2 topic hz "$topic" 2>/dev/null | awk '/average rate/{r=$3} END{if(r!="")print r}'
}

# 在两个 frame 之间能否查到变换
tf_ok() {
  timeout 4 ros2 run tf2_ros tf2_echo "$1" "$2" >/dev/null 2>&1
}

# 打印一张对齐的表
report() {
  printf '\n%-6s %-34s %s\n' "结果" "检查项" "细节"
  printf '%s\n' "--------------------------------------------------------------------------------"
  for entry in "${ROWS[@]}"; do
    IFS='|' read -r status name detail <<<"$entry"
    printf '%-6s %-34s %s\n' "$status" "$name" "$detail"
  done
  printf '%s\n' "--------------------------------------------------------------------------------"
  if [ "$FAILS" -eq 0 ]; then
    echo "结论：全部通过 ✅"
  else
    echo "结论：$FAILS 项失败 ❌  —— 先解决最靠前的失败项（数据流顺序，前面的会连带后面）"
  fi
}

echo "== SRM27 实车导航栈自检（只读） =="
echo "工作区: $WS_DIR"

# ---------------------------------------------------------------- 1. 环境
bad_ws=$(printf '%s' "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep -c -E 'srm_auto_sentry|srm_nav_27test' || true)
if [ "${bad_ws:-0}" = "0" ]; then
  row PASS "环境：只指向本工作区" "AMENT_PREFIX_PATH 无其它工作区"
else
  row FAIL "环境：只指向本工作区" "AMENT_PREFIX_PATH 含其它工作区 $bad_ws 条 → source scripts/setup/env_srm27.sh"
fi
for pkg in srm27_nav_bringup srm27_localization_adapter; do
  prefix=$(ros2 pkg prefix "$pkg" 2>/dev/null || true)
  case "$prefix" in
    "$WS_DIR"/*) row PASS "环境：$pkg" "$prefix" ;;
    "")          row FAIL "环境：$pkg" "未找到" ;;
    *)           row FAIL "环境：$pkg" "$prefix（不是本工作区）" ;;
  esac
done

# ---------------------------------------------------------------- 2. 传感器
if [ "$QUICK" = "1" ]; then LIDAR_WINDOW=3; else LIDAR_WINDOW=6; fi
lidar_rate=$(topic_rate /livox/lidar "$LIDAR_WINDOW" | head -1)
if [ -n "$lidar_rate" ]; then
  if awk -v r="$lidar_rate" 'BEGIN{exit !(r>=8 && r<=12)}'; then
    row PASS "雷达 /livox/lidar" "${lidar_rate} Hz"
  else
    row WARN "雷达 /livox/lidar" "${lidar_rate} Hz（期望 ~10，偏低查网络/丢包）"
  fi
else
  row FAIL "雷达 /livox/lidar" "没有数据（驱动没起 / 网络不通）"
fi

if [ "$QUICK" = "0" ]; then
  imu_rate=$(topic_rate /livox/imu 5 | head -1)
  if [ -n "$imu_rate" ]; then
    if awk -v r="$imu_rate" 'BEGIN{exit !(r>=150 && r<=250)}'; then
      row PASS "IMU /livox/imu" "${imu_rate} Hz"
    else
      row WARN "IMU /livox/imu" "${imu_rate} Hz（期望 ~200）"
    fi
  else
    row FAIL "IMU /livox/imu" "没有数据"
  fi
fi

# ---------------------------------------------------------------- 3. TF 链
for pair in "base_link livox_imu" "gimbal_yaw livox_imu"; do
  set -- $pair
  if tf_ok "$1" "$2"; then row PASS "TF $1 ← $2" "可查询"; else row FAIL "TF $1 ← $2" "查不到（车体 TF 没起/URDF 参数不对）"; fi
done
if tf_ok odom gimbal_yaw; then
  row PASS "TF odom ← gimbal_yaw" "可查询"
else
  row FAIL "TF odom ← gimbal_yaw" "查不到（sensor_scan_generation 没起 / 定位前端没数据）"
fi

# ---------------------------------------------------------------- 4. 定位输出
for topic in /registered_scan /lidar_odometry /odometry; do
  if [ "$QUICK" = "1" ]; then
    info=$(timeout 4 ros2 topic info "$topic" 2>/dev/null | head -1 || true)
    case "$info" in
      *"Publisher count: 0"*|"") row FAIL "定位输出 $topic" "没有发布者" ;;
      *) row PASS "定位输出 $topic" "有发布者" ;;
    esac
  else
    rate=$(topic_rate "$topic" 5 | head -1)
    if [ -n "$rate" ]; then row PASS "定位输出 $topic" "${rate} Hz"; else row FAIL "定位输出 $topic" "没有数据"; fi
  fi
done

# ---------------------------------------------------------------- 5. 唯一 TF owner
owners=$(timeout 8 ros2 node list 2>/dev/null | grep -E "static_transform_publisher_map2odom|small_gicp_relocalization|slam_toolbox|backend_node" || true)
owner_count=$(printf '%s\n' "$owners" | grep -c . || true)
if [ "${owner_count:-0}" = "1" ]; then
  row PASS "唯一 map→odom owner" "$(printf '%s' "$owners" | tr '\n' ' ')"
elif [ "${owner_count:-0}" = "0" ]; then
  row FAIL "唯一 map→odom owner" "一个都没有（map→odom 链断了）"
else
  row FAIL "唯一 map→odom owner" "$owner_count 个：$(printf '%s' "$owners" | tr '\n' ' ')"
fi

# ---------------------------------------------------------------- 6. Nav2 生命周期
lifecycle_ok=0
lifecycle_total=0
for node in controller_server planner_server bt_navigator behavior_server; do
  lifecycle_total=$((lifecycle_total + 1))
  state=$(timeout 4 ros2 lifecycle get "/$node" 2>/dev/null | head -1 || true)
  case "$state" in
    *active*) lifecycle_ok=$((lifecycle_ok + 1)) ;;
  esac
done
if [ "$lifecycle_ok" = "$lifecycle_total" ]; then
  row PASS "Nav2 生命周期" "$lifecycle_ok/$lifecycle_total active"
elif [ "$lifecycle_ok" = "0" ]; then
  row FAIL "Nav2 生命周期" "0/$lifecycle_total active（看导航栈终端日志）"
else
  row WARN "Nav2 生命周期" "$lifecycle_ok/$lifecycle_total active（部分没起来）"
fi

# ---------------------------------------------------------------- 7. 速度链
cmd_info=$(timeout 5 ros2 topic info /cmd_vel_chassis 2>/dev/null | head -2 || true)
cmd_pub=$(printf '%s' "$cmd_info" | awk -F': ' '/Publisher count/{print $2}')
cmd_sub=$(printf '%s' "$cmd_info" | awk -F': ' '/Subscription count/{print $2}')
if [ "${cmd_pub:-0}" = "1" ] && awk -v n="${cmd_sub:-0}" 'BEGIN{exit !(n+0 >= 1)}'; then
  row PASS "速度链 /cmd_vel_chassis" "发布者=$cmd_pub 订阅者=$cmd_sub"
elif [ "${cmd_pub:-0}" = "0" ]; then
  row FAIL "速度链 /cmd_vel_chassis" "没有发布者（srm_cmd_mux 没起）"
else
  row WARN "速度链 /cmd_vel_chassis" "发布者=${cmd_pub:-?} 订阅者=${cmd_sub:-?}（串口节点可能没起）"
fi
if [ "$QUICK" = "0" ]; then
  speed_rate=$(topic_rate /chassis_speed 4 | head -1)
  if [ -n "$speed_rate" ]; then row PASS "串口反馈 /chassis_speed" "${speed_rate} Hz"; else row FAIL "串口反馈 /chassis_speed" "没有数据（串口/协议节点）"; fi
fi

# ---------------------------------------------------------------- 8. 后端（可选）
state=$(timeout 4 ros2 topic echo /backend_node/localization_state --once 2>/dev/null | head -3 | tr '\n' ' ' || true)
if [ -z "$state" ]; then
  row SKIP "后端状态（未启用 backend 模式）" ""
else
  case "$state" in
    *NORMAL*)    row PASS "后端状态" "$state" ;;
    *INITIALIZING*) row WARN "后端状态" "$state（等首帧；检查 frontend/frame 是否有数据）" ;;
    *)           row FAIL "后端状态" "$state" ;;
  esac
fi

report
exit $(( FAILS > 0 ? 1 : 0 ))
