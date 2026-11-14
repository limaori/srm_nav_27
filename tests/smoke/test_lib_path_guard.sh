#!/usr/bin/env bash
# 验证 srm_lib_path_guard.sh 的行为（不启动任何 ROS 节点）
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GUARD="$ROOT/scripts/lib/srm_lib_path_guard.sh"
fail=0

run_case() {
  local name="$1" setup="$2" expect_has="$3" expect_not="$4"
  out=$(bash -c "set -uo pipefail; eval '$setup'; . '$GUARD'; echo \"RESULT=\${LD_LIBRARY_PATH-<unset>}\"" 2>&1)
  res=$(printf '%s\n' "$out" | grep '^RESULT=' | sed 's/^RESULT=//')
  local ok=1
  case "$res" in *"$expect_not"*) ok=0 ;; esac
  case "$res" in *"$expect_has"*) ;; *) [ -z "$expect_has" ] || ok=0 ;; esac
  [ "$ok" = "1" ] && echo "  [OK]   $name -> $res" || { echo "  [FAIL] $name"; echo "$out" | sed 's/^/      /'; fail=1; }
}

echo "== 1. 典型污染 (MVS 重复两次) =="
run_case "strip-mvs" \
  "export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:/opt/ros/humble/lib:/opt/MVS/lib/64" \
  "/opt/ros/humble/lib" "/opt/MVS"

echo "== 2. SRM_KEEP_MVS_LIB_PATH=1 不动 =="
run_case "keep" \
  "export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/ros/humble/lib; export SRM_KEEP_MVS_LIB_PATH=1" \
  "/opt/MVS/lib/64" "__nothing_removed__"

echo "== 3. 没有污染时不改写（只去重也不改写） =="
run_case "clean-untouched" \
  "export LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu" \
  "/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu" "/opt/MVS"

echo "== 4. LD_LIBRARY_PATH 未设置时不出错 (set -u) =="
out=$(bash -c "set -euo pipefail; unset LD_LIBRARY_PATH; . '$GUARD'; echo OK" 2>&1)
case "$out" in *OK*) echo "  [OK]   unset 情况通过" ;; *) echo "  [FAIL] $out"; fail=1 ;; esac

echo "== 5. 全是 MVS 时整条 unset =="
run_case "all-mvs" \
  "export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32" \
  "<unset>" "/opt/MVS"

echo "== 6. 重复 source 幂等 =="
out=$(bash -c "set -uo pipefail; export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/ros/humble/lib; . '$GUARD'; . '$GUARD'; echo \"RESULT=\$LD_LIBRARY_PATH\"" 2>&1)
res=$(printf '%s\n' "$out" | grep '^RESULT=' | sed 's/^RESULT=//')
case "$res" in "/opt/ros/humble/lib") echo "  [OK]   幂等 -> $res" ;; *) echo "  [FAIL] 幂等 -> $res"; fail=1 ;; esac

echo "== 7. SRM_LIB_PATH_STRIP_EXTRA =="
run_case "extra" \
  "export LD_LIBRARY_PATH=/home/srm/srm_auto_sentry/install/fast_lio/lib:/opt/ros/humble/lib; export SRM_LIB_PATH_STRIP_EXTRA=/home/srm/srm_auto_sentry/install" \
  "/opt/ros/humble/lib" "srm_auto_sentry"

echo "== 8. quiet 模式不打印 =="
out=$(bash -c "export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/ros/humble/lib; export SRM_QUIET_LIB_PATH_GUARD=1; . '$GUARD'; echo DONE" 2>&1)
case "$out" in "DONE") echo "  [OK]   quiet 无输出" ;; *) echo "  [FAIL] $out"; fail=1 ;; esac

echo
[ "$fail" = "0" ] && echo "全部通过" || echo "有失败用例"
exit "$fail"
