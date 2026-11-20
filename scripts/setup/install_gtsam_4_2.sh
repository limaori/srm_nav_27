#!/usr/bin/env bash
# =============================================================
# 把 ROS 2 Humble 的 GTSAM 4.2.x 装到用户目录（不需要 root）。
#
# 为什么需要它
# ============
# `srm27_zlocalization_backend`（Z-Localization backend 迁入版）的算法库硬依赖
# GTSAM 4.2.x：CMakeLists 里 `find_package(GTSAM 4.2 CONFIG REQUIRED)` +
# `if(GTSAM_VERSION VERSION_GREATER_EQUAL 4.3) FATAL_ERROR`，package.xml 也写了
# `version_gte="4.2.0" version_lt="4.3.0"`。
#
# 本机没有系统级 GTSAM，普通用户也没有 sudo 权限，所以这里下载 ROS 官方 deb 并用
# `dpkg-deb -x` 解到用户目录 —— 不装进系统、不动 dpkg 数据库，删目录即可完全回退。
#
# 若将来有 root，等价的一条命令是：
#   sudo apt install ros-humble-gtsam        # 4.2.0-3jammy
# 这时不需要本脚本，直接 source /opt/ros/humble/setup.bash 即可。
#
# 用法
# ====
#   ./scripts/setup/install_gtsam_4_2.sh                    # 默认前缀 ~/.local/gtsam-4.2.0
#   GTSAM_PREFIX=/abs/path ./scripts/setup/install_gtsam_4_2.sh
#   GTSAM_MIRROR=file:///abs/dir ./scripts/setup/install_gtsam_4_2.sh
#                                                           # 复用已下载的 deb，不联网
#   ./scripts/setup/install_gtsam_4_2.sh --force            # 前缀已存在时重装
#
# 装完会打印 `source <前缀>/env.sh`；构建与运行都需要它（或等价的 GTSAM_DIR /
# LD_LIBRARY_PATH）。
# =============================================================

set -euo pipefail

GTSAM_PACKAGE="ros-humble-gtsam"
GTSAM_VERSION="4.2.0-3jammy.20260226.005743"
MIRROR="${GTSAM_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/pool/main/r/ros-humble-gtsam}"
DEB="${GTSAM_PACKAGE}_${GTSAM_VERSION}_amd64.deb"

PREFIX="${GTSAM_PREFIX:-$HOME/.local/gtsam-4.2.0}"
FORCE=0
for argument in "$@"; do
  case "$argument" in
    --force) FORCE=1 ;;
    -h|--help)
      sed -n '2,32p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "[错误] 未知参数: $argument" >&2
      exit 2
      ;;
  esac
done

case "$PREFIX" in
  /*) ;;
  *) echo "[错误] GTSAM_PREFIX 必须是绝对路径: $PREFIX" >&2; exit 2 ;;
esac

for tool in curl dpkg-deb; do
  command -v "$tool" >/dev/null 2>&1 || { echo "[错误] 缺少 $tool" >&2; exit 1; }
done

if [ -f "$PREFIX/lib/cmake/GTSAM/GTSAMConfig.cmake" ] && [ "$FORCE" != "1" ]; then
  echo "[跳过] $PREFIX 下已有 GTSAM:"
  find "$PREFIX" -name GTSAMConfig.cmake -maxdepth 6 | head -1
  echo "[提示] 要重装加 --force。环境变量:"
  echo "       source $PREFIX/env.sh"
  exit 0
fi

WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

echo "==> 下载 $DEB"
curl -sSL --fail -o "$WORK_DIR/$DEB" "$MIRROR/$DEB"

echo "==> 解包到 $PREFIX（不装进系统）"
mkdir -p "$PREFIX"
dpkg-deb -x "$WORK_DIR/$DEB" "$PREFIX"

# deb 内部是 /opt/ros/humble/... 的布局，用一个稳定的软链接把前缀收敛成
# <prefix>/include、<prefix>/lib，后续 GTSAM_DIR / RPATH 都指向它。
INNER="$PREFIX/opt/ros/humble"
if [ ! -d "$INNER" ]; then
  echo "[错误] 解包后没有 $INNER，deb 布局可能变了" >&2
  exit 1
fi
link_or_replace() {
  local target="$1" name="$2"
  if [ -L "$PREFIX/$name" ]; then
    rm -f "$PREFIX/$name"
  elif [ -e "$PREFIX/$name" ]; then
    echo "[错误] $PREFIX/$name 已存在且不是软链接，拒绝覆盖" >&2
    exit 1
  fi
  ln -s "$INNER/$name" "$PREFIX/$name"
}
link_or_replace . include
link_or_replace . lib

# 运行时目录：libgtsam.so.4 与 libmetis-gtsam.so 同在 lib/x86_64-linux-gnu。
# 前缀里的 lib 是软链接（deb 内部是 /opt/ros/humble/lib），find 必须 -L 才会进去。
LIB_DIR="$(find -L "$PREFIX/lib" -maxdepth 2 -name 'libgtsam.so' -printf '%h\n' | head -1)"
if [ -z "$LIB_DIR" ]; then
  echo "[错误] 在 $PREFIX/lib 下找不到 libgtsam.so" >&2
  exit 1
fi

cat > "$PREFIX/env.sh" <<EOF
# 由 scripts/setup/install_gtsam_4_2.sh 生成：GTSAM 4.2.x 用户级安装。
# 构建：GTSAM_DIR 指向 CMake package 目录。
# 运行：RPATH 已由 backend 的 CMake 写进可执行文件；这里再补一份 LD_LIBRARY_PATH，
#       供 pytest/工具在非安装目录下直接跑时使用。
export GTSAM_PREFIX="$PREFIX"
export GTSAM_DIR="$PREFIX/lib/cmake/GTSAM"
export LD_LIBRARY_PATH="$LIB_DIR\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
EOF

echo "==> 校验版本"
set +e
VERSION_HEADER="$(find "$PREFIX/include" -name GTSAMConfigVersion.cmake -o -name ConfigVersion.cmake 2>/dev/null | head -1)"
set -e
if [ -n "$VERSION_HEADER" ]; then
  grep -m1 -E 'PACKAGE_VERSION ' "$VERSION_HEADER" || true
fi
echo "    GTSAM_DIR=$PREFIX/lib/cmake/GTSAM"
echo "    libs=$LIB_DIR"

echo
echo "装好了。构建/运行前执行："
echo "    source $PREFIX/env.sh"
echo
echo "然后："
echo "    colcon build --packages-select srm27_zlocalization_backend"
