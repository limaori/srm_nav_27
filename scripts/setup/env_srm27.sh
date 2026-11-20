#!/usr/bin/env bash
# =============================================================
# 让**当前 shell**只指向本工作区（~/.bashrc 里另一个工作区会被剔除）。
#
# 为什么需要
# ==========
# 本机 ~/.bashrc 还 source 了 `srm_auto_sentry`。它里面有**同名包**：
#   * livox_ros_driver2 —— 会被优先解析到那边（驱动/参数可能不是你启动的那份）；
#   * point_lio        —— `--lio-frontend point_lio` 也会串。
# 更隐蔽的是 LD_LIBRARY_PATH：那边 install/livox_ros_driver2/lib 里有
# liblivox_lidar_sdk_shared.so，于是本工作区**缺这个库**的问题被掩盖了 ——
# 看起来能跑，实际链接的是另一个工作区的产物。
#
# 用法
# ====
#   cd ~/srm_nav_27
#   source scripts/setup/env_srm27.sh
#   env | grep -c srm_auto_sentry      # 应为 0
#
# 想永久解决：把 ~/.bashrc 里 source 另一个工作区的那几行注释掉。
#
# 环境变量
# ========
#   SRM27_STRIP_WORKSPACES  要剔除的工作区关键字（默认 "srm_auto_sentry srm_nav_27test"）
#   SRM27_WS_DIR            本工作区根目录（默认脚本所在仓库根）
#   SRM27_KEEP_MVS=1        保留 /opt/MVS（默认剔除：它的老 libusb 会让 PCL 节点 exit 127）
# =============================================================

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "[错误] 这是给 source 用的脚本：source scripts/setup/env_srm27.sh" >&2
  exit 2
fi

_SRM27_WS_DIR="${SRM27_WS_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
_SRM27_STRIP="${SRM27_STRIP_WORKSPACES:-srm_auto_sentry srm_nav_27test}"

# 逐个路径变量剔除指定关键字（以及可选的 /opt/MVS）。
_srm27_strip_path_var() {
  local name="$1" value="${!1:-}" pattern="$2"
  [ -n "$value" ] || return 0
  local kept
  kept=$(printf '%s' "$value" | tr ':' '\n' | grep -v -E "$pattern" | paste -sd:)
  export "$name=$kept"
}

# 两个模式分开：
#   * 工作区关键字 —— 从所有路径变量（含 PATH）里剔除；
#   * /opt/MVS    —— **只**从 LD_LIBRARY_PATH 剔除。MVCAM_* / ALLUSERSPROFILE 里的
#     /opt/MVS 是海康相机 SDK 的正常配置，不能动。
_srm27_ws_pattern=""
for _pkg_ws in $_SRM27_STRIP; do
  _srm27_ws_pattern="${_srm27_ws_pattern:+$_srm27_ws_pattern|}$_pkg_ws"
done
_srm27_mvs_pattern=""
if [ "${SRM27_KEEP_MVS:-0}" != "1" ]; then
  _srm27_mvs_pattern="/opt/MVS"
fi

_srm27_managed_vars="AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
LD_LIBRARY_PATH PKG_CONFIG_PATH ROS_PACKAGE_PATH GZ_SIM_RESOURCE_PATH
IGN_GAZEBO_RESOURCE_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH PATH"

for _var in $_srm27_managed_vars; do
  [ -n "$_srm27_ws_pattern" ] && _srm27_strip_path_var "$_var" "$_srm27_ws_pattern"
done
if [ -n "$_srm27_mvs_pattern" ]; then
  _srm27_strip_path_var LD_LIBRARY_PATH "$_srm27_mvs_pattern"
fi

# 先 ROS，再本工作区：顺序反了会被 /opt/ros 的旧包盖住。
# 这里用 local_setup.bash 而不是 setup.bash：local_setup 只加本工作区的前缀，
# **不会**把生成 setup.bash 时恰好 source 过的其它工作区再拉进来（那些链是写死的）。
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
if [ -f "$_SRM27_WS_DIR/install/local_setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$_SRM27_WS_DIR/install/local_setup.bash"
else
  # shellcheck source=/dev/null
  source "$_SRM27_WS_DIR/install/setup.bash"
fi

# 本工作区 install/livox_ros_driver2/lib 里没有 SDK 动态库（该包 CMake 用绝对路径
# 链接源码树那份，但没装），所以显式指到源码树；否则运行期会去别的工作区借同名库。
_SRM27_SDK_DIR="$_SRM27_WS_DIR/src/localization/frontend/livox_ros_driver2/Livox-SDK2/lib/amd64"
if [ -d "$_SRM27_SDK_DIR" ]; then
  case ":$LD_LIBRARY_PATH:" in
    *":$_SRM27_SDK_DIR:"*) ;;
    *) export LD_LIBRARY_PATH="$_SRM27_SDK_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
  esac
fi

# GTSAM 4.2.x：只有 srm27_zlocalization_backend 需要。安装脚本默认放
# ~/.local/gtsam-4.2.0，但本机现有安装在工作区 .deps/gtsam-4.2.0（沙箱只能写工作区，
# backend 的 build 缓存 GTSAM_DIR 也指着它）。这里按 显式变量 → 家目录 → 工作区
# 依次探测：找到就导出 GTSAM_DIR / LD_LIBRARY_PATH，之后 colcon build / pytest
# 不必再 source 第二个 env.sh。
_srm27_gtsam_prefix=""
for _cand in "${SRM27_GTSAM_PREFIX:-}" "$HOME/.local/gtsam-4.2.0" \
             "$_SRM27_WS_DIR/.deps/gtsam-4.2.0" \
             "$_SRM27_WS_DIR/.deps/gtsam-4.2.0/opt/ros/humble"; do
  [ -n "$_cand" ] || continue
  if [ -f "$_cand/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
    _srm27_gtsam_prefix="$_cand"
    break
  fi
done
if [ -n "$_srm27_gtsam_prefix" ]; then
  export GTSAM_PREFIX="$_srm27_gtsam_prefix"
  export GTSAM_DIR="$_srm27_gtsam_prefix/lib/cmake/GTSAM"
  _srm27_gtsam_lib="$(find -L "$_srm27_gtsam_prefix/lib" -maxdepth 2 -name 'libgtsam.so' \
    -printf '%h\n' 2>/dev/null | head -1)"
  if [ -n "$_srm27_gtsam_lib" ]; then
    case ":$LD_LIBRARY_PATH:" in
      *":$_srm27_gtsam_lib:"*) ;;
      *) export LD_LIBRARY_PATH="$_srm27_gtsam_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
    esac
  fi
fi

# 实车验收常用路径（各阶段命令直接引用）。
export SRM27_LOG_DIR="${SRM27_LOG_DIR:-$HOME/srm_nav_27_data/logs}"
mkdir -p "$SRM27_LOG_DIR" 2>/dev/null || true
export PARAMS="$_SRM27_WS_DIR/src/bringup/srm27_nav_bringup/config/real/current/nav2_params_srm_minco_fakeyaw.yaml"
export MAP="$_SRM27_WS_DIR/src/bringup/srm27_nav_bringup/map/real/srm_site_01.yaml"
export ZLIO="$_SRM27_WS_DIR/src/bringup/srm27_nav_bringup/config/real/zlio_mid360.yaml"
export LIDAR_CFG="$_SRM27_WS_DIR/src/bringup/srm27_nav_bringup/config/real/mid360_user_config.json"

# ---- 自检（只看我们管理的路径变量，不误伤 MVCAM_* 之类的相机 SDK 配置）----
_srm27_bad=0
_srm27_hits=0
for _var in $_srm27_managed_vars; do
  _value="${!_var:-}"
  [ -n "$_value" ] || continue
  [ -n "$_srm27_ws_pattern" ] && _n=$(printf '%s' "$_value" | tr ':' '\n' | grep -c -E "$_srm27_ws_pattern" || true) || _n=0
  _n=${_n:-0}
  _srm27_hits=$((_srm27_hits + _n))
done
_srm27_mvs_hits=0
if [ -n "$_srm27_mvs_pattern" ]; then
  _srm27_mvs_hits=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -c -E "$_srm27_mvs_pattern" || true)
  _srm27_mvs_hits=${_srm27_mvs_hits:-0}
fi
echo "[env_srm27] 本工作区 : $_SRM27_WS_DIR"
echo "[env_srm27] 其它工作区路径残留: $_srm27_hits 个（应为 0）；LD_LIBRARY_PATH 里 /opt/MVS: $_srm27_mvs_hits 个（应为 0）"
[ "$_srm27_hits" = "0" ] || _srm27_bad=1
[ "$_srm27_mvs_hits" = "0" ] || _srm27_bad=1
for _pkg in srm27_nav_bringup srm27_localization_adapter frontend livox_ros_driver2 point_lio; do
  _prefix=$(ros2 pkg prefix "$_pkg" 2>/dev/null)
  case "$_prefix" in
    "$_SRM27_WS_DIR"/*) printf '[env_srm27] %-30s -> %s\n' "$_pkg" "$_prefix" ;;
    "")                 printf '[env_srm27] %-30s -> 未找到\n' "$_pkg"; _srm27_bad=1 ;;
    *)                  printf '[env_srm27] %-30s -> %s  ⚠ 不是本工作区\n' "$_pkg" "$_prefix"; _srm27_bad=1 ;;
  esac
done
if [ -f "$_SRM27_SDK_DIR/liblivox_lidar_sdk_shared.so" ]; then
  echo "[env_srm27] livox SDK : $_SRM27_SDK_DIR"
else
  echo "[env_srm27] livox SDK : ⚠ 源码树里没找到 liblivox_lidar_sdk_shared.so"
  _srm27_bad=1
fi
if [ -n "$_srm27_gtsam_prefix" ]; then
  _srm27_gtsam_version=$(sed -n 's/^set(PACKAGE_VERSION "\(.*\)")/\1/p' \
    "$GTSAM_DIR/GTSAMConfigVersion.cmake" 2>/dev/null | head -1)
  echo "[env_srm27] GTSAM    : $_srm27_gtsam_prefix (${_srm27_gtsam_version:-版本未知})"
else
  echo "[env_srm27] GTSAM    : 未找到（只影响 srm27_zlocalization_backend；见 scripts/setup/install_gtsam_4_2.sh）"
fi
if [ "$_srm27_bad" = "0" ]; then
  echo "[env_srm27] 就绪 ✅（PARAMS/MAP/ZLIO/LIDAR_CFG/SRM27_LOG_DIR 已导出）"
else
  echo "[env_srm27] ⚠ 仍不一致，先解决上面标 ⚠ 的项再启动实车链路"
fi

unset _var _pkg _prefix _srm27_bad _srm27_hits _srm27_ws_pattern _srm27_mvs_pattern \
      _srm27_mvs_hits _n _value _srm27_managed_vars _pkg_ws _srm27_strip_path_var \
      _cand _srm27_gtsam_prefix _srm27_gtsam_lib _srm27_gtsam_version
