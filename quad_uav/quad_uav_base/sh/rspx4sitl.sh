#!/bin/bash

# PX4 工程路径
PX4_DIR=~/px4_dev

# 额外工作空间环境
# EXTRA_WS_SETUP=~/Livox_ws/devel/setup.bash 

# Gazebo world 与模型配置
# WORLD_NAME="iscas_museum.world"
WORLD_NAME="empty.world"

# SDF_NAME="iris/iris.sdf"
SDF_NAME="iris_lidar/iris_lidar.sdf"

# Gazebo启动参数
GUI_ENABLE=true        # 是否启用 Gazebo GUI
# GUI_ENABLE=false        # 是否启用 Gazebo GUI
PX4_SIM_SPEED="1.0"     # 仿真速度倍率，1.0 = 实时，2.0 = 2倍速

# 选择 roslaunch 文件
LAUNCH_FILE="mavros_posix_sitl.launch"

# 频率配置方式
# 0: 不配置
# 1: 使用 px4-mavlink stream    (***gazebo仿真)
# 2: 使用 mavros mavcmd         (真机或仿真)
STREAM_CONFIG_METHOD=2

# px4-mavlink 目标 UDP 端口, QGC使用 mavlink status 查看
PX4_MAVLINK_UDP_PORT=14580

# 控制器目标频率（Hz）
STREAM_TARGET_RATE_HZ=200


PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_MAVLINK_BIN="$PX4_BUILD_DIR/bin/px4-mavlink"
GAZEBO_SETUP_SCRIPT="$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash"


# 加载额外工作空间环境
if [ -f "$EXTRA_WS_SETUP" ]; then
    source "$EXTRA_WS_SETUP"
fi

# 加载 PX4 Gazebo 环境
source "$GAZEBO_SETUP_SCRIPT" "$PX4_DIR" "$PX4_BUILD_DIR"
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:"$PX4_DIR"
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:"$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"



GAZEBO_PKG_PATH=$(rospack find mavlink_sitl_gazebo 2>/dev/null)

if [ -z "$GAZEBO_PKG_PATH" ]; then
    echo "Error: ROS package 'mavlink_sitl_gazebo' not found."
    return 1 2>/dev/null || exit 1
fi

WORLD_PATH="$GAZEBO_PKG_PATH/worlds/$WORLD_NAME"
SDF_PATH="$GAZEBO_PKG_PATH/models/$SDF_NAME"



echo ">> 启动 PX4 SITL..."


PX4_SIM_SPEED_FACTOR="$PX4_SIM_SPEED" roslaunch px4 "$LAUNCH_FILE" \
    world:="$WORLD_PATH" \
    sdf:="$SDF_PATH" \
    gui:="$GUI_ENABLE" \
    interactive:=false \
    x:=0.0 \
    y:=0.0 \
    z:=0.1 &
ROSLAUNCH_PID=$!

sleep 6

#-----------
# 使用 px4-mavlink stream 改频率
#-----------

if [ "$STREAM_CONFIG_METHOD" = "1" ]; then
    echo ">> 使用 px4-mavlink stream 配置消息频率"

    "$PX4_MAVLINK_BIN" stream -u "$PX4_MAVLINK_UDP_PORT" -s ATTITUDE -r "$STREAM_TARGET_RATE_HZ"
    "$PX4_MAVLINK_BIN" stream -u "$PX4_MAVLINK_UDP_PORT" -s HIGHRES_IMU -r "$STREAM_TARGET_RATE_HZ"
    "$PX4_MAVLINK_BIN" stream -u "$PX4_MAVLINK_UDP_PORT" -s LOCAL_POSITION_NED -r "$STREAM_TARGET_RATE_HZ"

elif [ "$STREAM_CONFIG_METHOD" = "2" ]; then
    echo ">> 使用 MAVROS mavcmd 配置消息频率"
    STREAM_TARGET_INTERVAL_US=$((1000000 / STREAM_TARGET_RATE_HZ))

    rosrun mavros mavcmd long 511 105 "$STREAM_TARGET_INTERVAL_US" 0 0 0 0 0 && sleep 0.2
    rosrun mavros mavcmd long 511 31 "$STREAM_TARGET_INTERVAL_US" 0 0 0 0 0 && sleep 0.2
    rosrun mavros mavcmd long 511 32 "$STREAM_TARGET_INTERVAL_US" 0 0 0 0 0 && sleep 0.2

fi


echo  "  --------------------------------------------------------------    "
wait "$ROSLAUNCH_PID"
