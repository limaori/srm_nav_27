#!/usr/bin/env bash
# =============================================================================
# LD_LIBRARY_PATH 消毒：剔除海康 MVS SDK 的库路径（可重复 source，幂等）
#
# 为什么需要它
# ------------
# 海康 MVS（机器视觉客户端）启动时会把
#     /opt/MVS/lib/64:/opt/MVS/lib/32
# **前置**到 LD_LIBRARY_PATH。这个目录里带了一份 2023 年的 libusb-1.0.so.0，
# 里面没有 libusb_set_option() 这个符号，而本机 PCL 1.12 的 libpcl_io.so 要用它。
#
# 于是任何链接 PCL 的程序（point_lio 最典型）一加载就：
#     symbol lookup error: /lib/x86_64-linux-gnu/libpcl_io.so.1.12:
#                         undefined symbol: libusb_set_option
# 进程当场退出，在 ROS launch 里只表现为一行
#     process has died [pid ..., exit code 127, cmd '.../pointlio_mapping ...']
# —— 不说原因，看起来像"节点自己崩了"。
#
# 2026-11-09 实车就是这么卡住的：开过 MVS 之后再起导航栈，point_lio 静默死掉
# → 没有 /aft_mapped_to_init → 没有 /odometry → 整条导航链"没有任何反应"，
# 而 map_server / 代价地图 / RViz 全都正常，非常容易误判成 TF 或地图问题。
#
# 用法
# ----
#   . "$SCRIPT_DIR/lib/srm_lib_path_guard.sh"      # source 即生效（在算完 SCRIPT_DIR 之后）
#
# 只改**本脚本进程及其子进程**的环境（包括它开的标签页），
# 不碰用户当前终端 —— 需要继续用 MVS 相机的终端不受影响。
#
# 开关（环境变量）
# ----------------
#   SRM_KEEP_MVS_LIB_PATH=1     跳过本消毒，保持原样（默认 0）
#   SRM_QUIET_LIB_PATH_GUARD=1  不打印提示（默认 0）
#   SRM_LIB_PATH_STRIP_EXTRA    额外要剔除的路径前缀，逗号分隔
#                               例: SRM_LIB_PATH_STRIP_EXTRA=/home/srm/srm_auto_sentry/install
# =============================================================================

srm_sanitize_ld_library_path() {
    local raw="${LD_LIBRARY_PATH:-}"
    [ -n "${raw}" ] || return 0
    [ "${SRM_KEEP_MVS_LIB_PATH:-0}" = "1" ] && return 0

    # 额外前缀（逗号分隔，允许为空）
    local -a extra=()
    if [ -n "${SRM_LIB_PATH_STRIP_EXTRA:-}" ]; then
        IFS=',' read -r -a extra <<< "${SRM_LIB_PATH_STRIP_EXTRA}"
    fi

    local kept="" dropped="" seen=":"
    local rest="${raw}" entry ex drop
    while :; do
        case "${rest}" in
            *:*) entry="${rest%%:*}"; rest="${rest#*:}" ;;
            *)   entry="${rest}";        rest=""           ;;
        esac

        if [ -n "${entry}" ]; then
            drop=0
            case "${entry}" in
                /opt/MVS|/opt/MVS/*) drop=1 ;;
            esac
            if [ "${drop}" = "0" ] && [ "${#extra[@]}" -gt 0 ]; then
                for ex in "${extra[@]}"; do
                    [ -n "${ex}" ] || continue
                    case "${entry}" in
                        "${ex}"|"${ex}"/*) drop=1; break ;;
                    esac
                done
            fi

            if [ "${drop}" = "1" ]; then
                dropped="${dropped:+${dropped}:}${entry}"
            else
                # 去重（同一路径重复出现没有意义，还会让 PATH 越来越长）
                case "${seen}" in
                    *":${entry}:"*) : ;;
                    *) kept="${kept:+${kept}:}${entry}"; seen="${seen}${entry}:" ;;
                esac
            fi
        fi

        [ -z "${rest}" ] && break
    done

    # 什么都没剔到：不动 LD_LIBRARY_PATH，避免无谓地改写用户环境
    [ -n "${dropped}" ] || return 0

    if [ -z "${kept}" ]; then
        unset LD_LIBRARY_PATH
    else
        export LD_LIBRARY_PATH="${kept}"
    fi

    if [ "${SRM_QUIET_LIB_PATH_GUARD:-0}" != "1" ]; then
        printf '[提示] LD_LIBRARY_PATH 已剔除 MVS 库路径: %s\n' "${dropped}" >&2
        printf '        原因: MVS 自带的老 libusb-1.0 缺 libusb_set_option(), 会让 point_lio/PCL 以 exit 127 退出\n' >&2
    fi
    return 0
}

# 报告当前 libusb 解析到哪一个（排查时用；不在正常路径上调用）
srm_report_libusb_resolution() {
    local exe="${1:-}"
    [ -n "${exe}" ] && [ -f "${exe}" ] || return 0
    local line
    line="$(ldd "${exe}" 2>/dev/null | grep 'libusb-1.0.so.0' || true)"
    [ -n "${line}" ] && printf '[提示] %s ->%s\n' "$(basename "${exe}")" "${line}" >&2
    return 0
}

srm_sanitize_ld_library_path
