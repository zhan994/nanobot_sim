
set -Eeuo pipefail

# -----------------------------------------------------------------------------
# 用户配置
# 用户配置（也可以在启动脚本前用同名环境变量覆盖）
# -----------------------------------------------------------------------------

# PX4 工程路径
PX4_DIR="${PX4_DIR:-$HOME/px4_dev}"

# 额外工作空间环境；未设置时保持为空
EXTRA_WS_SETUP="${EXTRA_WS_SETUP:-}"

# Gazebo world 与模型配置
WORLD_NAME="${WORLD_NAME:-empty.world}"
SDF_NAME="${SDF_NAME:-iris_lidar/iris_lidar.sdf}"

# Gazebo 启动参数
GUI_ENABLE="${GUI_ENABLE:-true}"
PX4_SIM_SPEED="${PX4_SIM_SPEED:-1.0}"

# roslaunch 文件
LAUNCH_FILE="${LAUNCH_FILE:-mavros_posix_sitl.launch}"

# MAVLink 消息频率配置方式
# 0: 不配置
# 1: 使用 PX4 shell 的 mavlink stream（Gazebo 仿真）
# 2: 使用 MAVROS CommandLong 服务（真机或仿真）
# 0: 不配置；1: px4-mavlink stream；2: MAVROS mavcmd
STREAM_CONFIG_METHOD="${STREAM_CONFIG_METHOD:-2}"

# 控制器目标频率（Hz）
STREAM_TARGET_RATE_HZ="${STREAM_TARGET_RATE_HZ:-200}"

# 方法 1 使用 PX4 MAVLink 实例的本地 UDP 端口，而不是 MAVROS 监听端口。
# 单机 SITL offboard 实例通常为 14580；使用前请通过 `mavlink status` 核实。
PX4_MAVLINK_UDP_PORT="${PX4_MAVLINK_UDP_PORT:-14580}"


# 等待 PX4、Gazebo 和 MAVROS 启动的时间
STARTUP_DELAY_SEC="${STARTUP_DELAY_SEC:-6}"

MAVROS_CONNECT_TIMEOUT_SEC="${MAVROS_CONNECT_TIMEOUT_SEC:-60}"
MAVROS_SERVICE_TIMEOUT_SEC="${MAVROS_SERVICE_TIMEOUT_SEC:-10}"
STREAM_CONFIG_RETRIES="${STREAM_CONFIG_RETRIES:-3}"
STREAM_RETRY_DELAY_SEC="${STREAM_RETRY_DELAY_SEC:-1}"

# -----------------------------------------------------------------------------
# 内部路径与运行状态
# 内部路径
# -----------------------------------------------------------------------------

PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_MAVLINK_BIN="$PX4_BUILD_DIR/bin/px4-mavlink"
GAZEBO_SETUP_SCRIPT="$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash"
ROSLAUNCH_PID=""

# -----------------------------------------------------------------------------
# 通用工具函数
# -----------------------------------------------------------------------------

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

# roslaunch 在独立进程组中运行，退出脚本时只清理本脚本启动的进程。
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
    if [[ -n "$ROSLAUNCH_PID" ]] && kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
        echo ">> 关闭 PX4 SITL、Gazebo 和 MAVROS..."
        kill -TERM -- "-$ROSLAUNCH_PID" 2>/dev/null || true
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
trap 'exit 130' INT
trap 'exit 143' TERM

# -----------------------------------------------------------------------------
# 配置检查与环境初始化
# 配置检查
# -----------------------------------------------------------------------------

validate_configuration() {
    is_positive_integer "$MAVROS_CONNECT_TIMEOUT_SEC" || \
        die "MAVROS_CONNECT_TIMEOUT_SEC 必须是正整数"
[[ -d "$PX4_DIR" ]] || { echo "Error: PX4 路径不存在: $PX4_DIR" >&2; exit 1; }
[[ -f "$GAZEBO_SETUP_SCRIPT" ]] || { echo "Error: Gazebo setup 不存在: $GAZEBO_SETUP_SCRIPT" >&2; exit 1; }
[[ "$STREAM_CONFIG_METHOD" =~ ^[012]$ ]] || { echo "Error: STREAM_CONFIG_METHOD 只能是 0、1 或 2" >&2; exit 1; }
[[ "$STREAM_TARGET_RATE_HZ" =~ ^[1-9][0-9]*$ ]] || { echo "Error: STREAM_TARGET_RATE_HZ 必须是正整数" >&2; exit 1; }

    [[ "$STREAM_CONFIG_METHOD" =~ ^[012]$ ]] || \
        die "STREAM_CONFIG_METHOD 只能是 0、1 或 2"
if [[ -n "$EXTRA_WS_SETUP" ]]; then
    [[ -f "$EXTRA_WS_SETUP" ]] || { echo "Error: 工作空间环境不存在: $EXTRA_WS_SETUP" >&2; exit 1; }
    # shellcheck disable=SC1090
    source "$EXTRA_WS_SETUP"
fi

    if [[ "$STREAM_CONFIG_METHOD" != "0" ]]; then
        is_positive_integer "$STREAM_TARGET_RATE_HZ" || \
            die "STREAM_TARGET_RATE_HZ 必须是正整数，当前值: $STREAM_TARGET_RATE_HZ"
        is_positive_integer "$STREAM_CONFIG_RETRIES" || \
            die "STREAM_CONFIG_RETRIES 必须是正整数"
        is_positive_integer "$STREAM_RETRY_DELAY_SEC" || \
            die "STREAM_RETRY_DELAY_SEC 必须是正整数"
    fi
# PX4 的 setup 脚本会追加这些变量，set -u 下需要先初始化。
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}"

    [[ -d "$PX4_DIR" ]] || die "PX4 路径不存在: $PX4_DIR"
    [[ -f "$GAZEBO_SETUP_SCRIPT" ]] || die "Gazebo setup 脚本不存在: $GAZEBO_SETUP_SCRIPT"
    [[ -f "$PX4_DIR/launch/$LAUNCH_FILE" ]] || die "PX4 launch 文件不存在: $PX4_DIR/launch/$LAUNCH_FILE"
# shellcheck disable=SC1090
source "$GAZEBO_SETUP_SCRIPT" "$PX4_DIR" "$PX4_BUILD_DIR"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

    if [[ -n "$EXTRA_WS_SETUP" ]]; then
        [[ -f "$EXTRA_WS_SETUP" ]] || die "EXTRA_WS_SETUP 不存在: $EXTRA_WS_SETUP"
    fi

    command -v roslaunch >/dev/null 2>&1 || die "未找到 roslaunch，请先 source ROS 1 环境"
    command -v rostopic >/dev/null 2>&1 || die "未找到 rostopic，请先 source ROS 1 环境"
    command -v rospack >/dev/null 2>&1 || die "未找到 rospack，请先 source ROS 1 环境"
    command -v timeout >/dev/null 2>&1 || die "未找到 timeout 命令"
    command -v setsid >/dev/null 2>&1 || die "未找到 setsid 命令"

    if [[ "$STREAM_CONFIG_METHOD" == "1" ]]; then
        [[ -x "$PX4_MAVLINK_BIN" ]] || \
            die "PX4 mavlink 可执行文件不存在或不可执行: $PX4_MAVLINK_BIN"
        is_positive_integer "$PX4_MAVLINK_UDP_PORT" || \
            die "方法 1 要求 PX4_MAVLINK_UDP_PORT 是正整数"
        ((PX4_MAVLINK_UDP_PORT <= 65535)) || die "PX4_MAVLINK_UDP_PORT 超出 UDP 端口范围"
    fi

    if [[ "$STREAM_CONFIG_METHOD" == "2" ]]; then
        is_positive_integer "$MAVROS_SERVICE_TIMEOUT_SEC" || \
            die "MAVROS_SERVICE_TIMEOUT_SEC 必须是正整数"
        command -v rosservice >/dev/null 2>&1 || \
            die "未找到 rosservice，请先 source ROS 1 环境"
    fi
GAZEBO_PKG_PATH="$(rospack find mavlink_sitl_gazebo 2>/dev/null)" || {
    echo "Error: ROS package 'mavlink_sitl_gazebo' 未找到" >&2
    exit 1
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
WORLD_PATH="$GAZEBO_PKG_PATH/worlds/$WORLD_NAME"
SDF_PATH="$GAZEBO_PKG_PATH/models/$SDF_NAME"

resolve_simulation_files() {
    local gazebo_pkg_path
    gazebo_pkg_path="$(rospack find mavlink_sitl_gazebo 2>/dev/null)" || \
        die "ROS package 'mavlink_sitl_gazebo' 未找到"

    WORLD_PATH="$gazebo_pkg_path/worlds/$WORLD_NAME"
    SDF_PATH="$gazebo_pkg_path/models/$SDF_NAME"
    [[ -f "$WORLD_PATH" ]] || die "world 文件不存在: $WORLD_PATH"
    [[ -f "$SDF_PATH" ]] || die "SDF 文件不存在: $SDF_PATH"
}
[[ -f "$WORLD_PATH" ]] || { echo "Error: world 文件不存在: $WORLD_PATH" >&2; exit 1; }
[[ -f "$SDF_PATH" ]] || { echo "Error: SDF 文件不存在: $SDF_PATH" >&2; exit 1; }

# -----------------------------------------------------------------------------
# 启动与等待 PX4 SITL
# 启动 PX4 SITL + Gazebo + MAVROS
# -----------------------------------------------------------------------------

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
echo ">> 启动 PX4 SITL"
echo ">> world: $WORLD_PATH"
echo ">> model: $SDF_PATH"

wait_for_mavros_connection() {
    local deadline state_output
    deadline=$((SECONDS + MAVROS_CONNECT_TIMEOUT_SEC))
    log "等待 /mavros/state 的 connected: True（超时 ${MAVROS_CONNECT_TIMEOUT_SEC}s）..."
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
sleep "$STARTUP_DELAY_SEC"
kill -0 "$ROSLAUNCH_PID" 2>/dev/null || {
    wait "$ROSLAUNCH_PID" || true
    echo "Error: roslaunch 启动失败或提前退出" >&2
    exit 1
}

# -----------------------------------------------------------------------------
# MAVLink 消息频率配置
# 配置 MAVLink 消息频率
# -----------------------------------------------------------------------------

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
if [[ "$STREAM_CONFIG_METHOD" == "1" ]]; then
    echo ">> 使用 px4-mavlink stream 配置消息频率"
    [[ -x "$PX4_MAVLINK_BIN" ]] || {
        echo "Error: 找不到可执行文件: $PX4_MAVLINK_BIN" >&2
        exit 1
    }

        printf '[rspx4sitl] 警告: ID=%s, target=%sHz 配置失败（第 %s/%s 次，调用状态=%s）\n' \
            "$message_id" "$STREAM_TARGET_RATE_HZ" "$attempt" "$STREAM_CONFIG_RETRIES" "$call_status" >&2
        printf '[rspx4sitl] 服务返回: %s\n' "${response:-<无返回>}" >&2
        ((attempt < STREAM_CONFIG_RETRIES)) && sleep "$STREAM_RETRY_DELAY_SEC"
    done

    die "MAV_CMD_SET_MESSAGE_INTERVAL 最终失败: ID=$message_id, target=${STREAM_TARGET_RATE_HZ}Hz"
}

configure_streams() {
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
            local interval_us
            interval_us=$((1000000 / STREAM_TARGET_RATE_HZ))
            ((interval_us > 0)) || die "目标频率过高，整数微秒间隔变为 0"
    "$PX4_MAVLINK_BIN" stream -u "$PX4_MAVLINK_UDP_PORT" -s HIGHRES_IMU -r "$STREAM_TARGET_RATE_HZ"
    "$PX4_MAVLINK_BIN" stream -u "$PX4_MAVLINK_UDP_PORT" -s ATTITUDE_QUATERNION -r "$STREAM_TARGET_RATE_HZ"
    "$PX4_MAVLINK_BIN" stream -u "$PX4_MAVLINK_UDP_PORT" -s LOCAL_POSITION_NED -r "$STREAM_TARGET_RATE_HZ"

            log "方法 2: 通过 MAVROS 当前已连接的 FCU MAVLink 链路发送 CommandLong"
            wait_for_command_service
            configure_with_command_long 105 HIGHRES_IMU "$interval_us"
            configure_with_command_long 31 ATTITUDE_QUATERNION "$interval_us"
            configure_with_command_long 32 LOCAL_POSITION_NED "$interval_us"
            ;;
    esac
}
elif [[ "$STREAM_CONFIG_METHOD" == "2" ]]; then
    echo ">> 使用 MAVROS mavcmd 配置消息频率"
    STREAM_TARGET_INTERVAL_US=$((1000000 / STREAM_TARGET_RATE_HZ))
    ((STREAM_TARGET_INTERVAL_US > 0)) || {
        echo "Error: STREAM_TARGET_RATE_HZ 过高" >&2
        exit 1
    }

# -----------------------------------------------------------------------------
# 主流程
# -----------------------------------------------------------------------------
    rosrun mavros mavcmd long 511 105 "$STREAM_TARGET_INTERVAL_US" 0 0 0 0 0
    rosrun mavros mavcmd long 511 31 "$STREAM_TARGET_INTERVAL_US" 0 0 0 0 0
    rosrun mavros mavcmd long 511 32 "$STREAM_TARGET_INTERVAL_US" 0 0 0 0 0

main() {
    validate_configuration
    load_environment
    resolve_simulation_files
    start_sitl
    wait_for_mavros_connection
    configure_streams
else
    echo ">> 不修改 MAVLink 消息频率"
fi

    printf '%s\n' '------------------------ 配置完成 ------------------------'
    log "SITL 正在运行；按 Ctrl+C 后仅清理本脚本启动的进程"
    set +e
    wait "$ROSLAUNCH_PID"
    local roslaunch_status=$?
    set -e
    if ((roslaunch_status != 0)); then
        die "roslaunch 已退出，状态码: $roslaunch_status"
    fi
}
echo "------------------------ 启动完成 ------------------------"
echo ">> 按 Ctrl+C 退出"

main
wait "$ROSLAUNCH_PID"