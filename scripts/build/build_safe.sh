#!/usr/bin/env bash
# 受限并发的 colcon 构建入口（防死机）。
#
# 背景：`colcon build --parallel-workers N` 只限制**同时构建几个包**，
# 每个包内部仍然用 `make -j$(nproc)`（本机 16 并发）。两个内存大户并行时
# （如 `frontend`(ZLIO) + `srm27_zlocalization_backend`(GTSAM)）会出现
# 32 个编译进程，15 GiB 内存 + 1 GiB swap 直接打满 → 系统假死/重启。
# 2026-11-24 已复现一次。
#
# 本脚本同时限制两层并发：
#   * MAKEFLAGS=-j<jobs>：每个包内部最多 jobs 个编译进程；
#   * --parallel-workers <packages>：同时最多构建 packages 个包。
# 默认 jobs=2、packages=1（编译注意事项.md 的"限制并发到 2"）。
#
# 用法：
#   ./scripts/build/build_safe.sh --packages-select srm27_map_core
#   ./scripts/build/build_safe.sh --packages-up-to srm27_nav_bringup
#   JOBS=3 PKG_WORKERS=2 ./scripts/build/build_safe.sh --packages-select a b
#   其余参数原样透传给 colcon build（--cmake-args 等）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
JOBS="${JOBS:-2}"
PKG_WORKERS="${PKG_WORKERS:-1}"
COLCON="${COLCON:-colcon}"

if ! [[ "$JOBS" =~ ^[0-9]+$ ]] || ! [[ "$PKG_WORKERS" =~ ^[0-9]+$ ]]; then
  echo "[错误] JOBS/PKG_WORKERS 必须是正整数（当前 JOBS=$JOBS PKG_WORKERS=$PKG_WORKERS）" >&2
  exit 2
fi
if [ "$JOBS" -lt 1 ] || [ "$PKG_WORKERS" -lt 1 ]; then
  echo "[错误] JOBS/PKG_WORKERS 必须 >= 1" >&2
  exit 2
fi

export MAKEFLAGS="-j${JOBS} ${MAKEFLAGS:-}"
# CMake 3.12+ 的 `cmake --build` 也认这个变量；colcon 走 make 时以 MAKEFLAGS 为准。
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"

cd "$WS_DIR"
echo "[build_safe] 每包并发 -j${JOBS}，同时构建 ${PKG_WORKERS} 个包（防止 OOM/死机）"
exec "$COLCON" build --parallel-workers "$PKG_WORKERS" \
  --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL="$JOBS" "$@"
