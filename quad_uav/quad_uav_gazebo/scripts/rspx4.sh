#!/usr/bin/env bash

set -Eeuo pipefail

# -----------------------------------------------------------------------------
# 1. 读取用户配置
# -----------------------------------------------------------------------------

PX4_DIR="${PX4_DIR:-$HOME/px4_dev}"
EXTRA_WS_SETUP="${EXTRA_WS_SETUP:-}"

WORLD_NAME="${WORLD_NAME:-empty.world}"
SDF_NAME="${SDF_NAME:-iris_lidar/iris_lidar.sdf}"
GUI_ENABLE="${GUI_ENABLE:-true}"
PX4_SIM_SPEED="${PX4_SIM_SPEED:-1.0}"
LAUNCH_FILE="${LAUNCH_FILE:-mavros_posix_sitl.launch}"

# 0: 不配置；1: px4-mavlink stream；2: MAVROS mavcmd
STREAM_CONFIG_METHOD="${STREAM_CONFIG_METHOD:-2}"
STREAM_TARGET_RATE_HZ="${STREAM_TARGET_RATE_HZ:-200}"
PX4_MAVLINK_UDP_PORT="${PX4_MAVLINK_UDP_PORT:-14580}"

# 等待 MAVROS 与 PX4 建立连接的最长时间
MAVROS_CONNECT_TIMEOUT_SEC="${MAVROS_CONNECT_TIMEOUT_SEC:-60}"

# -----------------------------------------------------------------------------
# 2. 计算 PX4 内部路径，并注册退出清理逻辑
# -----------------------------------------------------------------------------

PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_MAVLINK_BIN="$PX4_BUILD_DIR/bin/px4-mavlink"
GAZEBO_SETUP_SCRIPT="$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash"
ROSLAUNCH_PID=""

# roslaunch 在独立进程组中运行，退出脚本时只清理本脚本启动的进程。
cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM

    if [[ -n "$ROSLAUNCH_PID" ]] && kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
        echo ">> 关闭 PX4 SITL、Gazebo 和 MAVROS..."
        kill -TERM -- "-$ROSLAUNCH_PID" 2>/dev/null || true
        wait "$ROSLAUNCH_PID" 2>/dev/null || true
    fi

    exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# -----------------------------------------------------------------------------
# 3. 检查配置值以及 PX4、launch 
# -----------------------------------------------------------------------------

[[ -d "$PX4_DIR" ]] || { echo "Error: PX4 路径不存在: $PX4_DIR" >&2; exit 1; }
[[ -f "$GAZEBO_SETUP_SCRIPT" ]] || { echo "Error: Gazebo setup 不存在: $GAZEBO_SETUP_SCRIPT" >&2; exit 1; }
[[ -f "$PX4_DIR/launch/$LAUNCH_FILE" ]] || { echo "Error: PX4 launch 文件不存在: $PX4_DIR/launch/$LAUNCH_FILE" >&2; exit 1; }
[[ "$STREAM_CONFIG_METHOD" =~ ^[012]$ ]] || { echo "Error: STREAM_CONFIG_METHOD 只能是 0、1 或 2" >&2; exit 1; }
[[ "$STREAM_TARGET_RATE_HZ" =~ ^[1-9][0-9]*$ ]] || { echo "Error: STREAM_TARGET_RATE_HZ 必须是正整数" >&2; exit 1; }
[[ "$MAVROS_CONNECT_TIMEOUT_SEC" =~ ^[1-9][0-9]*$ ]] || { echo "Error: MAVROS_CONNECT_TIMEOUT_SEC 必须是正整数" >&2; exit 1; }

if [[ -n "$EXTRA_WS_SETUP" ]]; then
    [[ -f "$EXTRA_WS_SETUP" ]] || { echo "Error: 工作空间环境不存在: $EXTRA_WS_SETUP" >&2; exit 1; }
    # shellcheck disable=SC1090
    source "$EXTRA_WS_SETUP"
fi

# -----------------------------------------------------------------------------
# 4. 加载 PX4/Gazebo 环境，检查 ROS 命令，并解析 world 和模型路径
# -----------------------------------------------------------------------------

# PX4 的 setup 脚本会追加这些变量，set -u 下需要先初始化。
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}"

# shellcheck disable=SC1090
source "$GAZEBO_SETUP_SCRIPT" "$PX4_DIR" "$PX4_BUILD_DIR"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

command -v roslaunch >/dev/null 2>&1 || { echo "Error: 未找到 roslaunch，请先 source ROS 1 环境" >&2; exit 1; }
command -v rospack >/dev/null 2>&1 || { echo "Error: 未找到 rospack，请先 source ROS 1 环境" >&2; exit 1; }
command -v rostopic >/dev/null 2>&1 || { echo "Error: 未找到 rostopic，请先 source ROS 1 环境" >&2; exit 1; }
command -v setsid >/dev/null 2>&1 || { echo "Error: 未找到 setsid 命令" >&2; exit 1; }
command -v timeout >/dev/null 2>&1 || { echo "Error: 未找到 timeout 命令" >&2; exit 1; }

if [[ "$STREAM_CONFIG_METHOD" == "2" ]]; then
    command -v rosrun >/dev/null 2>&1 || { echo "Error: 未找到 rosrun，请先 source ROS 1 环境" >&2; exit 1; }
fi

GAZEBO_PKG_PATH="$(rospack find mavlink_sitl_gazebo 2>/dev/null)" || {
    echo "Error: ROS package 'mavlink_sitl_gazebo' 未找到" >&2
    exit 1
}

WORLD_PATH="$GAZEBO_PKG_PATH/worlds/$WORLD_NAME"
SDF_PATH="$GAZEBO_PKG_PATH/models/$SDF_NAME"

[[ -f "$WORLD_PATH" ]] || { echo "Error: world 文件不存在: $WORLD_PATH" >&2; exit 1; }
[[ -f "$SDF_PATH" ]] || { echo "Error: SDF 文件不存在: $SDF_PATH" >&2; exit 1; }

# -----------------------------------------------------------------------------
# 5. 在独立进程组中启动 PX4 SITL、Gazebo 和 MAVROS
# -----------------------------------------------------------------------------

echo ">> 启动 PX4 SITL"
echo ">> world: $WORLD_PATH"
echo ">> model: $SDF_PATH"

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

# -----------------------------------------------------------------------------
# 6. 等待 MAVROS 确认连接 PX4；超时或 roslaunch 退出则终止流程
# -----------------------------------------------------------------------------

echo ">> 等待 MAVROS 连接, 最长 ${MAVROS_CONNECT_TIMEOUT_SEC}s..."
MAVROS_CONNECTED=false
CONNECT_DEADLINE=$((SECONDS + MAVROS_CONNECT_TIMEOUT_SEC))

while ((SECONDS < CONNECT_DEADLINE)); do
    if ! kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
        wait "$ROSLAUNCH_PID" 2>/dev/null || true
        echo "Error: roslaunch 在 MAVROS 建立连接前退出" >&2
        exit 1
    fi

    MAVROS_STATE="$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
    if [[ "$MAVROS_STATE" == *"connected: True"* ]]; then
        MAVROS_CONNECTED=true
        echo ">> MAVROS 已连接 PX4"
        break
    fi

    sleep 1
done

if [[ "$MAVROS_CONNECTED" != true ]]; then
    echo "Error: MAVROS 连接超时，请检查 PX4 日志、fcu_url 和 UDP 端口" >&2
    exit 1
fi

# -----------------------------------------------------------------------------
# 7. 按选择的方法配置 MAVLink 消息频率
# -----------------------------------------------------------------------------

if [[ "$STREAM_CONFIG_METHOD" == "1" ]]; then
    echo ">> 使用 px4-mavlink stream 配置消息频率"
    [[ -x "$PX4_MAVLINK_BIN" ]] || {
        echo "Error: 找不到可执行文件: $PX4_MAVLINK_BIN" >&2
        exit 1
    }

    "$PX4_MAVLINK_BIN" stream -u "$PX4_MAVLINK_UDP_PORT" -s HIGHRES_IMU -r "$STREAM_TARGET_RATE_HZ"
    "$PX4_MAVLINK_BIN" stream -u "$PX4_MAVLINK_UDP_PORT" -s ATTITUDE_QUATERNION -r "$STREAM_TARGET_RATE_HZ"
    "$PX4_MAVLINK_BIN" stream -u "$PX4_MAVLINK_UDP_PORT" -s LOCAL_POSITION_NED -r "$STREAM_TARGET_RATE_HZ"

elif [[ "$STREAM_CONFIG_METHOD" == "2" ]]; then
    echo ">> 使用 MAVROS mavcmd 配置消息频率"
    STREAM_TARGET_INTERVAL_US=$((1000000 / STREAM_TARGET_RATE_HZ))
    ((STREAM_TARGET_INTERVAL_US > 0)) || {
        echo "Error: STREAM_TARGET_RATE_HZ 过高" >&2
        exit 1
    }

    rosrun mavros mavcmd long 511 105 "$STREAM_TARGET_INTERVAL_US" 0 0 0 0 0
    rosrun mavros mavcmd long 511 31 "$STREAM_TARGET_INTERVAL_US" 0 0 0 0 0
    rosrun mavros mavcmd long 511 32 "$STREAM_TARGET_INTERVAL_US" 0 0 0 0 0

else
    echo ">> 不修改 MAVLink 消息频率"
fi

# -----------------------------------------------------------------------------
# 8. 启动完成后持续等待 roslaunch, 按 Ctrl+C 时触发清理逻辑
# -----------------------------------------------------------------------------

echo "------------------------ 启动完成 ------------------------"
echo ">> 按 Ctrl+C 退出"

wait "$ROSLAUNCH_PID"
