# quad_uav

[简体中文](README_CN.md) | [English](README.md)

- **quad_uav_gazebo**：PX4-SITL Gazebo 仿真。
- **quad_uav_planner**：Diff-Planner 的基本使用示例。

## PX4-SITL

### 构建

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y git cmake build-essential libssl-dev libusb-1.0-0-dev \
                    libprotobuf-dev protobuf-compiler libeigen3-dev libxml2-utils \
                    python3-pip python3-setuptools python3-wheel python3-numpy \
                    python3-matplotlib python3-pytest python3-pytest-cov \
                    gawk wget zip unzip tar bzip2 flex bison libgstreamer1.0-dev \
                    libgstreamer-plugins-base1.0-dev libsdl2-dev libsdl2-image-dev \
                    libopenjp2-7 libtiff5 libjpeg-dev

pip3 install kconfiglib jsonschema jinja2 future lxml pyros-genmsg empy==3.3.4 pyyaml

# MAVROS
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
cd /opt/ros/noetic/lib/mavros
sudo chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# PX4 `origin/dev_nanobot`
cd ~
git clone -b dev_nanobot https://github.com/zhan994/PX4-Autopilot.git --recursive px4_dev

cd px4_dev
sudo chmod +x ./Tools/setup/ubuntu.sh

# 将 ./Tools/setup/requirements.txt 中的 `matplotlib>=3.0.*` 改为：
matplotlib>=3.0

# 可选：在 ./Tools/setup/ubuntu.sh 约第 176 行替换下载源
wget -O /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 https://mirrors.tuna.tsinghua.edu.cn/armbian-releases/_toolchains/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-${INSTALL_ARCH}-linux.tar.bz2 && \
sudo tar -jxf /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 -C /opt/;

bash ./Tools/setup/ubuntu.sh

# 如有需要，执行 `sudo reboot`，然后验证 Gazebo 能否启动
cd ~/px4_dev
make px4_sitl gazebo
```

### 运行

```bash
# 复制无人机模型
cp -r <path-to-nanobot-ws>/src/nanobot_sim/quad_uav/quad_uav_gazebo/models/* ~/px4_dev/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models

cd <path-to-nanobot-ws>/src/nanobot_sim/quad_uav
chmod +x ./quad_uav_gazebo/scripts/*.sh
chmod +x ./quad_uav_gazebo/scripts/*.py

# 启动 PX4-SITL
cd <path-to-nanobot-ws>
source devel/setup.bash
cd <path-to-nanobot-ws>/src/nanobot_sim/quad_uav
./quad_uav_gazebo/scripts/rspx4.sh

# 将点云从雷达坐标系转换到 world 坐标系
python3 ./quad_uav_gazebo/scripts/pointcloud_to_body.py

# 结束后清理环境
./quad_uav_gazebo/scripts/clean_env.sh
```

## Diff-Planner

### 构建

```bash
sudo apt install -y libompl-dev libfmt-dev libeigen3-dev ros-noetic-rosfmt

mkdir -p <path-to-nanobot-ws>/src/nanobot_sim/quad_uav/quad_uav_planner
cd <path-to-nanobot-ws>/src/nanobot_sim/quad_uav/quad_uav_planner
git clone -b dev_nanobot https://github.com/zhan994/Diff-Planner.git
cd <path-to-nanobot-ws> && catkin_make
```

### 运行

- 终端 1：PX4-SITL

```bash
cd <path-to-nanobot-ws> && source devel/setup.bash
roscd quad_uav_gazebo/
chmod +x ./scripts/rspx4.sh
./scripts/rspx4.sh
```

> `rspx4.sh` 会同时发布从 `world` 到 `map` 的恒等变换。

- 终端 2：px4ctrl

```bash
cd <path-to-nanobot-ws> && source devel/setup.bash
roslaunch px4ctrl run_ctrl_sim.launch
```

- 终端 3：RC 仿真与目标发布工具

```bash
cd <path-to-nanobot-ws> && source devel/setup.bash
rosrun quad_uav_gazebo rc_sim.py
```

- 终端 4：点云转换

```bash
cd <path-to-nanobot-ws> && source devel/setup.bash
rosrun quad_uav_gazebo pointcloud_to_body.py
```

- 终端 5：规划器

```bash
cd <path-to-nanobot-ws> && source devel/setup.bash
roslaunch diff_planner gz_single_drone.launch enable_rviz:=true
```

> 无人机起飞后，等待其进入悬停状态，然后在 RViz 中使用 **2D Nav Goal** 设置目标。也可在 `rc_sim.py` 中输入 `gl <x> <y> <z>` 发布三维目标。可在 Gazebo 中添加物体，构建避障测试场景。
