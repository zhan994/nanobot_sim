# PX4 Sim

## Step 1: PX4-Autopilot 编译

### 0. 先决条件
- ubuntu 20.04
- ros1 noetic

> 实验环境：WSL2 CPU=Intel 14900HX; GPU=Nvidia RTX4090Laptop; RAM=32GB
> 如有安装Anaconda，启动前，请先退出 conda env : 多次执行 `conda deactivate`


### 1. 依赖安装

1. 系统依赖
```
sudo apt update && sudo apt upgrade -y

sudo apt install -y git cmake build-essential libssl-dev libusb-1.0-0-dev \
libprotobuf-dev protobuf-compiler libeigen3-dev libxml2-utils \
python3-pip python3-setuptools python3-wheel python3-numpy \
python3-matplotlib python3-pytest python3-pytest-cov \
gawk wget zip unzip tar bzip2 flex bison libgstreamer1.0-dev \
libgstreamer-plugins-base1.0-dev libsdl2-dev libsdl2-image-dev \
libopenjp2-7 libtiff5 libjpeg-dev

pip3 install kconfiglib jsonschema jinja2 future lxml pyros-genmsg empy==3.3.4 pyyaml
```

2. mavros

```
sudo apt install -y  ros-noetic-mavros ros-noetic-mavros-extras
cd /opt/ros/noetic/lib/mavros
sudo chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

### 2. PX4编译

1. 代码下载
```
cd ~
git clone -b dev_nanobot https://github.com/zhan994/PX4-Autopilot.git px4_dev
```
> 注意分支: 'Your branch is up to date with 'origin/dev_nanobot'.'

2. 安装环境依赖

**注意**：下面脚本卡在Saving to: ‘/tmp/gcc-arm-none-eabi-9-2020-q2-update-linux.tar.bz2’的话，可以根据3.修改 ubuntu.sh 的wget变为清华源

```
cd ~/px4_dev
sudo chmod +x ./Tools/setup/ubuntu.sh
sudo bash ./Tools/setup/ubuntu.sh
```

3. 换源(可选)

- 打开 ./Tools/setup/ubuntu.sh;
- 搜索 `gcc-arm`, 应在176行左右;
- 将`wget`开头一行和`sudo`一行替换为下面两行;

```
wget -O /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 https://mirrors.tuna.tsinghua.edu.cn/armbian-releases/_toolchains/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-${INSTALL_ARCH}-linux.tar.bz2 && \
sudo tar -jxf /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 -C /opt/;
```

4. 编译

```
cd ~/px4_dev
make px4_sitl gazebo
```

> 一切正常的话此处会开启一个gazebo页面, 关闭即可;


## Step 2: PX4 模型安装


1. 下载模型文件

```
mkdir ~/nanobot_ws/src
cd ~/nanobot_ws/src
git clone -b feature_uav https://github.com/zhan994/nanobot_sim.git
```

2. 场景与模型路径

```
cd ~/nanobot_ws
catkin_make
source devel/setup.bash
```

启动脚本会通过 `rospack` 直接读取 `gazebo_worlds` 中的场景与场景模型，
无需复制到 PX4 源码目录。无人机机体模型仍由 `quad_uav_gazebo` 提供。

3. 安装依赖

```
sudo apt install ros-noetic-velodyne-gazebo-plugins
```



## Step 3: PX4 SITL 环境启动

1. 赋予执行权限
```
cd ~/nanobot_ws/src/nanobot_sim/quad_uav
chmod +x ./quad_uav_gazebo/scripts/*.sh
chmod +x ./quad_uav_gazebo/scripts/*.py
```

2. 启动 px4-sitl

```
cd ~/nanobot_ws/src/nanobot_sim/quad_uav
./quad_uav_gazebo/scripts/rspx4.sh
```

3. 启动点云转换

> 使用脚本将雷达系点云旋转至Body系

```
cd ~/nanobot_ws/src/nanobot_sim/quad_uav
python3 ./quad_uav_gazebo/scripts/pointcloud_to_body.py
```

4. **结束后**清理环境

```
cd ~/nanobot_ws/src/nanobot_sim/quad_uav
./quad_uav_gazebo/scripts/clean_env.sh
```


## Step 4：Diff-Planner

0. 安装依赖 

```
sudo apt install -y libompl-dev libfmt-dev libeigen3-dev ros-noetic-rosfmt
```

1. 编译代码

```
mkdir -p ~/nanobot_ws/src/nanobot_sim/quad_uav/quad_uav_planner
cd ~/nanobot_ws/src/nanobot_sim/quad_uav/quad_uav_planner
git clone -b dev_nanobot https://github.com/zhan994/Diff-Planner.git
cd ~/nanobot_ws
catkin_make
```

2. 启动

下面需要多窗口:

- Terminal 1: 启动px4sitl
```
cd ~/nanobot_ws && source devel/setup.bash
roscd quad_uav_gazebo/
./scripts/rspx4.sh
```
> 也可自定义参数，例如：
> 使用120m场景或Airylike雷达时计算量较大，可以开启无GUI模式 `GUI_ENABLE=false`
```
SDF_NAME=iris_lidar_light/iris_lidar_light.sdf \
GUI_ENABLE=false \
./scripts/rspx4.sh
```

复杂 CPU-ray 场景可选择独立的 mini 雷达：

```bash
SDF_NAME=iris_lidar_mini/iris_lidar_mini.sdf \
GUI_ENABLE=false \
./scripts/rspx4.sh
```

mini 配置为 `90 × 48 @ 5 Hz`、量程 `30 m`，输出话题仍为
`/velodyne_points`。

> 第一次使用 请 chmod +x 给权限
`chmod +x ./scripts/rspx4.sh`

- Terminal 2: 启动px4ctrl

```
cd ~/nanobot_ws && source devel/setup.bash
roslaunch px4ctrl run_ctrl_sim.launch
```

- Terminal 3: 启动 rc sim

```
cd ~/nanobot_ws && source devel/setup.bash
rosrun quad_uav_gazebo uav_cli.py
```
> 输入 '?' 查看 help
> 也可使用原版：rc_sim.py

- Terminal 4: 启动点云转换

```
cd ~/nanobot_ws && source devel/setup.bash
rosrun quad_uav_gazebo pointcloud_to_body.py
```

- Terminal 5: 启动 planner

```
cd ~/nanobot_ws && source devel/setup.bash
roslaunch diff_planner gz_single_drone.launch enable_rviz:=true
```
```
roslaunch diff_planner gz_single_drone.launch enable_rviz:=true 2>&1 \
  | tee ~/intern/raycast_log/120_env/13_airylike_252505.log
```

> 先起飞，进入悬停后在 rviz 使用 2D Nav Goal 进行指点飞行，也可使用 uav_cli.py `gl x y z` 发布三维坐标
可以使用Gazebo中的物体制造障碍飞行环境
> 由于无 world->map 的 tf 坐标变换，而 Diffplanner 硬编码了原点为 world 坐标系，所以发布 world->map 的 tf 坐标变换
```
cd ~/nanobot_ws && source devel/setup.bash && roslaunch quad_uav_gazebo map_world_tf.launch
```
