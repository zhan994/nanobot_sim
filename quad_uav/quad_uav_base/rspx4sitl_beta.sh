#!/usr/bin/env bash

set -Eeuo pipefail

# --------------------------- 用户配置 ---------------------------

# PX4 工程路径
PX4_DIR="${PX4_DIR:-$HOME/px4_dev}"

# 额外工作空间环境；未设置时保持为空
EXTRA_WS_SETUP="${EXTRA_WS_SETUP:-}"

#  world、模型、GUI 和仿真速度默认值
WORLD_NAME="${WORLD_NAME:-empty.world}"
SDF_NAME="${SDF_NAME:-iris_lidar/iris_lidar.sdf}"
GUI_ENABLE="${GUI_ENABLE:-true}"
PX4_SIM_SPEED="${PX4_SIM_SPEED:-1.0}"
LAUNCH_FILE="${LAUNCH_FILE:-mavros_posix_sitl.launch}"

# 0: 不配置；1: PX4 shell 的 mavlink stream；2: MAVROS CommandLong 服务
STREAM_CONFIG_METHOD="${STREAM_CONFIG_METHOD:-2}"
STREAM_TARGET_RATE_HZ="${STREAM_TARGET_RATE_HZ:-200}"


# 方法 1 使用 PX4 MAVLink 实例的本地 UDP 端口，不是 MAVROS 的监听端口
# 当前常见单机 SITL offboard 实例通常为 14580，但不同 PX4 配置/实例可能不同
# 使用方法 1 前务必在 PX4 shell 运行 `mavlink status` 核实，避免改到 QGC 实例
# 建议直接用 2 
PX4_MAVLINK_UDP_PORT="${PX4_MAVLINK_UDP_PORT:-14580}"

MAVROS_CONNECT_TIMEOUT_SEC="${MAVROS_CONNECT_TIMEOUT_SEC:-60}"
MAVROS_SERVICE_TIMEOUT_SEC="${MAVROS_SERVICE_TIMEOUT_SEC:-10}"
STREAM_CONFIG_RETRIES="${STREAM_CONFIG_RETRIES:-3}"
STREAM_RETRY_DELAY_SEC="${STREAM_RETRY_DELAY_SEC:-1}"






# --------------------------- 内部路径 ---------------------------

PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_MAVLINK_BIN="$PX4_BUILD_DIR/bin/px4-mavlink"
GAZEBO_SETUP_SCRIPT="$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash"
ROSLAUNCH_PID=""

log() {
    printf '[rspx4sitl] %s\n' "$*"
}

die() {
    printf '[rspx4sitl] 错误: %s\n' "$*" >&2
    exit 1
}

is_positive_integer() {
    [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM

    if [[ -n "$ROSLAUNCH_PID" ]]; then
        # roslaunch 由 setsid 创建独立进程组，只清理本脚本启动的进程树。
        if kill -0 "$ROSLAUNCH_PID" 2>/dev/null || kill -0 -- "-$ROSLAUNCH_PID" 2>/dev/null; then
            log "正在结束 roslaunch 进程组 $ROSLAUNCH_PID ..."
            kill -TERM -- "-$ROSLAUNCH_PID" 2>/dev/null || true

            local i
            for ((i = 0; i < 50; i++)); do
                kill -0 -- "-$ROSLAUNCH_PID" 2>/dev/null || break
                sleep 0.1
            done

            if kill -0 -- "-$ROSLAUNCH_PID" 2>/dev/null; then
                log "进程组未及时退出，发送 SIGKILL"
                kill -KILL -- "-$ROSLAUNCH_PID" 2>/dev/null || true
            fi
        fi
        wait "$ROSLAUNCH_PID" 2>/dev/null || true
    fi

    exit "$exit_code"
}

on_signal() {
    local signal_name=$1
    local exit_code=$2
    log "收到 $signal_name，准备退出"
    exit "$exit_code"
}

trap cleanup EXIT
trap 'on_signal INT 130' INT
trap 'on_signal TERM 143' TERM

validate_configuration() {
    is_positive_integer "$STREAM_TARGET_RATE_HZ" || \
        die "STREAM_TARGET_RATE_HZ 必须是正整数，当前值: $STREAM_TARGET_RATE_HZ"
    is_positive_integer "$MAVROS_CONNECT_TIMEOUT_SEC" || \
        die "MAVROS_CONNECT_TIMEOUT_SEC 必须是正整数"
    is_positive_integer "$MAVROS_SERVICE_TIMEOUT_SEC" || \
        die "MAVROS_SERVICE_TIMEOUT_SEC 必须是正整数"
    is_positive_integer "$STREAM_CONFIG_RETRIES" || \
        die "STREAM_CONFIG_RETRIES 必须是正整数"
    is_positive_integer "$STREAM_RETRY_DELAY_SEC" || \
        die "STREAM_RETRY_DELAY_SEC 必须是正整数"

    [[ "$STREAM_CONFIG_METHOD" =~ ^[012]$ ]] || \
        die "STREAM_CONFIG_METHOD 只能是 0、1 或 2"
    [[ -d "$PX4_DIR" ]] || die "PX4 路径不存在: $PX4_DIR"
    [[ -f "$GAZEBO_SETUP_SCRIPT" ]] || die "Gazebo setup 脚本不存在: $GAZEBO_SETUP_SCRIPT"
    [[ -x "$PX4_MAVLINK_BIN" ]] || die "PX4 mavlink 可执行文件不存在或不可执行: $PX4_MAVLINK_BIN"
    [[ -f "$PX4_DIR/launch/$LAUNCH_FILE" ]] || die "PX4 launch 文件不存在: $PX4_DIR/launch/$LAUNCH_FILE"

    if [[ -n "$EXTRA_WS_SETUP" ]]; then
        [[ -f "$EXTRA_WS_SETUP" ]] || die "EXTRA_WS_SETUP 不存在: $EXTRA_WS_SETUP"
    fi

    command -v roslaunch >/dev/null 2>&1 || die "未找到 roslaunch，请先 source ROS 1 环境"
    command -v rostopic >/dev/null 2>&1 || die "未找到 rostopic，请先 source ROS 1 环境"
    command -v rosservice >/dev/null 2>&1 || die "未找到 rosservice，请先 source ROS 1 环境"
    command -v rospack >/dev/null 2>&1 || die "未找到 rospack，请先 source ROS 1 环境"
    command -v timeout >/dev/null 2>&1 || die "未找到 timeout 命令"
    command -v setsid >/dev/null 2>&1 || die "未找到 setsid 命令"

    if [[ "$STREAM_CONFIG_METHOD" == "1" ]]; then
        is_positive_integer "$PX4_MAVLINK_UDP_PORT" || \
            die "方法 1 要求 PX4_MAVLINK_UDP_PORT 是正整数"
        ((PX4_MAVLINK_UDP_PORT <= 65535)) || die "PX4_MAVLINK_UDP_PORT 超出 UDP 端口范围"
    fi
}

load_environment() {
    if [[ -n "$EXTRA_WS_SETUP" ]]; then
        log "加载额外工作空间: $EXTRA_WS_SETUP"
        # shellcheck disable=SC1090
        source "$EXTRA_WS_SETUP"
    fi

    export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
    export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}"

    # shellcheck disable=SC1090
    source "$GAZEBO_SETUP_SCRIPT" "$PX4_DIR" "$PX4_BUILD_DIR"
    export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
}

resolve_simulation_files() {
    local gazebo_pkg_path
    gazebo_pkg_path="$(rospack find mavlink_sitl_gazebo 2>/dev/null)" || \
        die "ROS package 'mavlink_sitl_gazebo' 未找到"

    WORLD_PATH="$gazebo_pkg_path/worlds/$WORLD_NAME"
    SDF_PATH="$gazebo_pkg_path/models/$SDF_NAME"
    [[ -f "$WORLD_PATH" ]] || die "world 文件不存在: $WORLD_PATH"
    [[ -f "$SDF_PATH" ]] || die "SDF 文件不存在: $SDF_PATH"
}

start_sitl() {
    log "启动 PX4 SITL: world=$WORLD_PATH, sdf=$SDF_PATH, gui=$GUI_ENABLE"

    setsid env PX4_SIM_SPEED_FACTOR="$PX4_SIM_SPEED" \
        roslaunch px4 "$LAUNCH_FILE" \
        world:="$WORLD_PATH" \
        sdf:="$SDF_PATH" \
        gui:="$GUI_ENABLE" \
        interactive:=false \
        x:=0.0 \
        y:=0.0 \
        z:=0.1 &
    ROSLAUNCH_PID=$!
    log "roslaunch PID/进程组: $ROSLAUNCH_PID"
}

wait_for_mavros_connection() {
    local deadline state_output
    deadline=$((SECONDS + MAVROS_CONNECT_TIMEOUT_SEC))
    log "等待 /mavros/state 的 connected: True（超时 ${MAVROS_CONNECT_TIMEOUT_SEC}s）..."

    while ((SECONDS < deadline)); do
        if ! kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
            wait "$ROSLAUNCH_PID" 2>/dev/null || true
            die "roslaunch 在 MAVROS 建立连接前提前退出"
        fi

        state_output="$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
        if [[ -n "$state_output" ]]; then
            if [[ "$state_output" == *"connected: True"* ]]; then
                log "MAVROS 连接状态: connected: True"
                return 0
            fi
            log "MAVROS 连接状态: connected: False"
        fi
        sleep 1
    done

    die "等待 MAVROS 连接超时；请检查 PX4 启动日志、fcu_url 和 UDP 端口"
}

wait_for_command_service() {
    local deadline
    deadline=$((SECONDS + MAVROS_SERVICE_TIMEOUT_SEC))
    log "等待服务 /mavros/cmd/command ..."

    while ((SECONDS < deadline)); do
        kill -0 "$ROSLAUNCH_PID" 2>/dev/null || die "roslaunch 在等待 MAVROS 服务时提前退出"
        if timeout 2 rosservice info /mavros/cmd/command >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    die "服务 /mavros/cmd/command 在 ${MAVROS_SERVICE_TIMEOUT_SEC}s 内不可用"
}

configure_with_px4_stream() {
    local message_name=$1
    local message_id=$2
    local attempt

    log "请求消息 ID $message_id ($message_name): ${STREAM_TARGET_RATE_HZ} Hz"
    for ((attempt = 1; attempt <= STREAM_CONFIG_RETRIES; attempt++)); do
        if "$PX4_MAVLINK_BIN" stream \
            -u "$PX4_MAVLINK_UDP_PORT" \
            -s "$message_name" \
            -r "$STREAM_TARGET_RATE_HZ"; then
            return 0
        fi
        printf '[rspx4sitl] 警告: ID %s 配置失败（第 %s/%s 次）\n' \
            "$message_id" "$attempt" "$STREAM_CONFIG_RETRIES" >&2
        ((attempt < STREAM_CONFIG_RETRIES)) && sleep "$STREAM_RETRY_DELAY_SEC"
    done

    die "PX4 mavlink stream 配置失败: ID=$message_id, target=${STREAM_TARGET_RATE_HZ}Hz"
}

configure_with_command_long() {
    local message_id=$1
    local message_name=$2
    local interval_us=$3
    local attempt response call_status
    local request

    # MAV_CMD_SET_MESSAGE_INTERVAL: param1=消息 ID，param2=间隔 us，其余参数为 0。
    request="{broadcast: false, command: 511, confirmation: 0, param1: ${message_id}.0, param2: ${interval_us}.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
    log "请求消息 ID $message_id ($message_name): ${STREAM_TARGET_RATE_HZ} Hz, interval=${interval_us} us"

    for ((attempt = 1; attempt <= STREAM_CONFIG_RETRIES; attempt++)); do
        call_status=0
        response="$(timeout "$MAVROS_SERVICE_TIMEOUT_SEC" \
            rosservice call /mavros/cmd/command "$request" 2>&1)" || call_status=$?

        if ((call_status == 0)) && \
            [[ "$response" == *"success: True"* ]] && \
            [[ "$response" == *"result: 0"* ]]; then
            log "消息 ID $message_id 配置成功: success=True, result=0 (ACCEPTED)"
            return 0
        fi

        printf '[rspx4sitl] 警告: ID=%s, target=%sHz 配置失败（第 %s/%s 次，调用状态=%s）\n' \
            "$message_id" "$STREAM_TARGET_RATE_HZ" "$attempt" "$STREAM_CONFIG_RETRIES" "$call_status" >&2
        printf '[rspx4sitl] 服务返回: %s\n' "${response:-<无返回>}" >&2
        ((attempt < STREAM_CONFIG_RETRIES)) && sleep "$STREAM_RETRY_DELAY_SEC"
    done

    die "MAV_CMD_SET_MESSAGE_INTERVAL 最终失败: ID=$message_id, target=${STREAM_TARGET_RATE_HZ}Hz"
}

configure_streams() {
    local interval_us
    # STREAM_TARGET_RATE_HZ 已验证为正整数，不会除零。
    interval_us=$((1000000 / STREAM_TARGET_RATE_HZ))
    ((interval_us > 0)) || die "目标频率过高，整数微秒间隔变为 0"

    case "$STREAM_CONFIG_METHOD" in
        0)
            log "STREAM_CONFIG_METHOD=0，不修改 MAVLink 消息频率"
            ;;
        1)
            log "方法 1: 修改 PX4 本地 UDP 端口 $PX4_MAVLINK_UDP_PORT 对应的 MAVLink 实例"
            log "请用 PX4 shell 的 'mavlink status' 确认该端口属于 MAVROS/offboard 链路，不要选择 QGC 实例"
            configure_with_px4_stream HIGHRES_IMU 105
            configure_with_px4_stream ATTITUDE_QUATERNION 31
            configure_with_px4_stream LOCAL_POSITION_NED 32
            ;;
        2)
            log "方法 2: 通过 MAVROS 当前已连接的 FCU MAVLink 链路发送 CommandLong"
            wait_for_command_service
            configure_with_command_long 105 HIGHRES_IMU "$interval_us"
            configure_with_command_long 31 ATTITUDE_QUATERNION "$interval_us"
            configure_with_command_long 32 LOCAL_POSITION_NED "$interval_us"
            ;;
    esac
}

print_diagnostics() {
    cat <<'EOF'
------------------------ 配置完成 ------------------------
EOF
}

main() {
    validate_configuration
    load_environment
    resolve_simulation_files
    start_sitl
    wait_for_mavros_connection
    configure_streams
    print_diagnostics

    log "SITL 正在运行；按 Ctrl+C 后仅清理本脚本启动的进程"
    set +e
    wait "$ROSLAUNCH_PID"
    local roslaunch_status=$?
    set -e
    if ((roslaunch_status != 0)); then
        die "roslaunch 已退出，状态码: $roslaunch_status"
    fi
}

main "$@"
