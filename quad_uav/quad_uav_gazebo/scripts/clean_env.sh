#!/usr/bin/env bash

set -uo pipefail

# 清理 ROS 1、PX4 SITL、Gazebo Classic 和 MAVROS 环境

readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly RED='\033[0;31m'
readonly NC='\033[0m'

GRACE_PERIOD_SEC="${GRACE_PERIOD_SEC:-5}"
DRY_RUN=false

declare -A TARGETS=()
declare -A START_TIMES=()
declare -A ANCESTORS=()

log() {
    printf '%b[clean_env]%b %s\n' "$YELLOW" "$NC" "$*"
}

warn() {
    printf '%b[clean_env] 警告:%b %s\n' "$RED" "$NC" "$*" >&2
}

parse_args() {
    while (($# > 0)); do
        case "$1" in
            --dry-run)
                DRY_RUN=true
                ;;
            *)
                warn "仅支持 --dry-run，未知参数: $1"
                exit 2
                ;;
        esac
        shift
    done

    [[ "$GRACE_PERIOD_SEC" =~ ^[0-9]+$ ]] || {
        warn "GRACE_PERIOD_SEC 必须是非负整数，当前值: $GRACE_PERIOD_SEC"
        exit 2
    }
}

proc_uid() {
    stat -c '%u' "/proc/$1" 2>/dev/null
}

proc_ppid() {
    local key value
    while read -r key value _; do
        if [[ "$key" == "PPid:" ]]; then
            printf '%s\n' "$value"
            return 0
        fi
    done < "/proc/$1/status" 2>/dev/null
    return 1
}

proc_start_time() {
    # /proc/PID/stat 第 22 列是进程启动时钟，防止等待期间 PID 被复用。
    awk '{print $22}' "/proc/$1/stat" 2>/dev/null
}

proc_command() {
    local comm
    local -a argv=()
    mapfile -d '' -t argv < "/proc/$1/cmdline" 2>/dev/null || true
    if ((${#argv[@]} > 0)); then
        printf '%q ' "${argv[@]}"
    else
        IFS= read -r comm < "/proc/$1/comm" 2>/dev/null || comm=unknown
        printf '[%s]' "$comm"
    fi
}

remember_ancestors() {
    local pid=$$
    local parent

    while ((pid > 1)); do
        ANCESTORS["$pid"]=1
        parent="$(proc_ppid "$pid" 2>/dev/null || true)"
        [[ "$parent" =~ ^[0-9]+$ ]] || break
        ((parent < pid || parent > 1)) || break
        pid=$parent
    done
    ANCESTORS[1]=1
}

is_named_target() {
    local pid=$1
    local comm token base
    local -a argv=()

    IFS= read -r comm < "/proc/$pid/comm" 2>/dev/null || comm=""
    mapfile -d '' -t argv < "/proc/$pid/cmdline" 2>/dev/null || true

    # 精确匹配可执行文件/argv token，避免按整条命令行做子串匹配而误杀。
    for token in "$comm" "${argv[@]}"; do
        base=${token##*/}
        case "$base" in
            roscore|rosmaster|roslaunch|rosout|rosrun|rosnode|\
            gzserver|gzclient|gazebo|spawn_model|\
            px4|px4-mavlink|px4-commander|px4-listener|px4-uorb|\
            mavros_node|mavcmd|micrortps_agent|MicroXRCEAgent|uxrce_dds_agent|\
            rspx4sitl.sh|rspx4sitl-fix.sh|rspx4sitl_wsl.sh)
                return 0
                ;;
        esac
    done
    return 1
}

add_target() {
    local pid=$1
    local reason=$2
    local start_time

    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    [[ -d "/proc/$pid" ]] || return 0
    [[ "$(proc_uid "$pid" 2>/dev/null || true)" == "$UID" ]] || return 0
    [[ -z "${ANCESTORS[$pid]+x}" ]] || return 0

    start_time="$(proc_start_time "$pid" 2>/dev/null || true)"
    [[ -n "$start_time" ]] || return 0
    TARGETS["$pid"]=$reason
    START_TIMES["$pid"]=$start_time
}

discover_roots() {
    local path pid
    for path in /proc/[0-9]*; do
        pid=${path##*/}
        [[ "$(proc_uid "$pid" 2>/dev/null || true)" == "$UID" ]] || continue
        [[ -z "${ANCESTORS[$pid]+x}" ]] || continue
        if is_named_target "$pid"; then
            add_target "$pid" "ROS/PX4/Gazebo 进程"
        fi
    done
}

discover_descendants() {
    local changed=true
    local path pid parent

    # roslaunch 下的插件和自定义节点名称不可预知，因此递归加入所有本地子进程。
    while [[ "$changed" == true ]]; do
        changed=false
        for path in /proc/[0-9]*; do
            pid=${path##*/}
            [[ -z "${TARGETS[$pid]+x}" ]] || continue
            [[ "$(proc_uid "$pid" 2>/dev/null || true)" == "$UID" ]] || continue
            parent="$(proc_ppid "$pid" 2>/dev/null || true)"
            if [[ -n "${TARGETS[$parent]+x}" ]]; then
                add_target "$pid" "PID $parent 的子进程"
                changed=true
            fi
        done
    done
}

same_process_is_alive() {
    local pid=$1
    local current_start
    [[ -d "/proc/$pid" ]] || return 1
    current_start="$(proc_start_time "$pid" 2>/dev/null || true)"
    [[ -n "$current_start" && "$current_start" == "${START_TIMES[$pid]:-}" ]]
}

sorted_target_pids() {
    if ((${#TARGETS[@]} > 0)); then
        printf '%s\n' "${!TARGETS[@]}" | sort -n
    fi
}

print_targets() {
    local pid
    ((${#TARGETS[@]} > 0)) || return 0

    log "发现 ${#TARGETS[@]} 个相关进程"
    [[ "$DRY_RUN" == true ]] || return 0

    while read -r pid; do
        [[ -n "$pid" ]] || continue
        printf '  PID=%-7s PPID=%-7s %-24s %s\n' \
            "$pid" "$(proc_ppid "$pid" 2>/dev/null || printf '?')" \
            "${TARGETS[$pid]}" "$(proc_command "$pid")"
    done < <(sorted_target_pids)
}

signal_targets() {
    local signal=$1
    local pid
    local -a live=()

    while read -r pid; do
        [[ -n "$pid" ]] || continue
        same_process_is_alive "$pid" && live+=("$pid")
    done < <(sorted_target_pids)

    ((${#live[@]} > 0)) || return 0
    kill -s "$signal" "${live[@]}" 2>/dev/null || true
}

wait_for_exit() {
    local deadline=$((SECONDS + GRACE_PERIOD_SEC))
    local pid any_alive

    while ((SECONDS < deadline)); do
        any_alive=false
        while read -r pid; do
            [[ -n "$pid" ]] || continue
            if same_process_is_alive "$pid"; then
                any_alive=true
                break
            fi
        done < <(sorted_target_pids)
        [[ "$any_alive" == false ]] && return 0
        sleep 0.2
    done
}

clean_processes() {
    ((${#TARGETS[@]} > 0)) || return 0
    if [[ "$DRY_RUN" == true ]]; then
        return 0
    fi

    signal_targets TERM
    wait_for_exit

    local remaining=0
    local pid
    while read -r pid; do
        [[ -n "$pid" ]] || continue
        same_process_is_alive "$pid" && ((remaining += 1))
    done < <(sorted_target_pids)

    if ((remaining > 0)); then
        signal_targets KILL
        sleep 0.2
    fi
}

remove_owned_path() {
    local path=$1
    [[ -e "$path" || -S "$path" ]] || return 0
    [[ "$(stat -c '%u' "$path" 2>/dev/null || true)" == "$UID" ]] || {
        return 0
    }

    if [[ "$DRY_RUN" == true ]]; then
        printf '  将删除: %s\n' "$path"
    else
        rm -rf -- "$path"
    fi
}

clean_runtime_files() {
    local path

    for path in /tmp/px4-sock-*; do
        [[ -S "$path" ]] || continue
        remove_owned_path "$path"
    done

    for path in "/tmp/gazebo-$USER-rtshaderlibcache" /dev/shm/gazebo-"$USER"-*; do
        [[ -e "$path" ]] || continue
        remove_owned_path "$path"
    done

}

verify_cleanup() {
    local path pid count=0
    TARGETS=()
    START_TIMES=()
    discover_roots
    discover_descendants

    for pid in "${!TARGETS[@]}"; do
        same_process_is_alive "$pid" && ((count += 1))
    done

    ((count == 0)) || return 1

    for path in /tmp/px4-sock-*; do
        if [[ -S "$path" && "$(stat -c '%u' "$path" 2>/dev/null || true)" == "$UID" ]]; then
            return 1
        fi
    done
    return 0
}

main() {
    parse_args "$@"
    remember_ancestors

    printf '%b>>> 开始清理当前用户的 PX4 SITL、Gazebo Classic、ROS 1 和 MAVROS 残留...%b\n' \
        "$YELLOW" "$NC"

    discover_roots
    discover_descendants
    print_targets
    clean_processes
    clean_runtime_files

    if [[ "$DRY_RUN" == true ]]; then
        exit 0
    fi

    if verify_cleanup; then
        printf '%b>>> 清理完成。%b\n' "$GREEN" "$NC"
    else
        warn "清理未完成"
        exit 1
    fi
}

main "$@"
