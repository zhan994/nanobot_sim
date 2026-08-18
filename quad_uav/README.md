# quad_uav

- **quad_uav_gazebo**: PX4-SITL Gazebo simulation.
- **quad_uav_planner**: simple usages for Diff-Planner.

## PX4-SITL

### Build

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

# mavros
sudo apt install -y  ros-noetic-mavros ros-noetic-mavros-extras
cd /opt/ros/noetic/lib/mavros
sudo chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# PX4 `origin/dev_nanobot`
cd ~
git clone -b dev_nanobot https://github.com/zhan994/PX4-Autopilot.git --recursive px4_dev

cd px4_dev
sudo chmod +x ./Tools/setup/ubuntu.sh
bash ./Tools/setup/ubuntu.sh

# `sudo reboot`，验证是否开启gazebo页面
cd ~/px4_dev
make px4_sitl gazebo
```

### Run

```bash
# uav models
cp -r <path-to-nanobot-ws>/src/nanobot_sim/quad_uav/quad_uav_gazebo/models/* ~/px4_dev/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models

cd <path-to-nanobot-ws>/src/nanobot_sim/quad_uav
chmod +x ./quad_uav_gazebo/scripts/*.sh
chmod +x ./quad_uav_gazebo/scripts/*.py

# 启动 px4-sitl
cd <path-to-nanobot-ws>
source devel/setup.bash
cd <path-to-nanobot-ws>/src/nanobot_sim/quad_uav
./quad_uav_gazebo/scripts/rspx4.sh

# 使用脚本将雷达系点云旋转至Body系
python3 ./quad_uav_gazebo/scripts/pointcloud_to_body.py

# 结束后清理环境
./quad_uav_gazebo/scripts/clean_env.sh
```

## Diff-Planner

### Build

```bash
sudo apt install -y libompl-dev libfmt-dev libeigen3-dev ros-noetic-rosfmt

mkdir -p <path-to-nanobot-ws>/src/nanobot_sim/quad_uav/quad_uav_planner
cd <path-to-nanobot-ws>/src/nanobot_sim/quad_uav/quad_uav_planner
git clone -b dev_nanobot https://github.com/zhan994/Diff-Planner.git
cd <path-to-nanobot-ws> && catkin_make
```

### Run

- Terminal 1: px4-sitl

```bash
cd <path-to-nanobot-ws> && source devel/setup.bash
roscd quad_uav_gazebo/
./scripts/rspx4.sh
```

- Terminal 2: px4ctrl

```bash
cd <path-to-nanobot-ws> && source devel/setup.bash
roslaunch px4ctrl run_ctrl_sim.launch
```

- Terminal 3: rc sim

```bash
cd <path-to-nanobot-ws> && source devel/setup.bash
rosrun quad_uav_gazebo rc_sim.py

# 输入 '1' 起飞
```

- Terminal 4: 点云转换

```bash
cd <path-to-nanobot-ws> && source devel/setup.bash
rosrun quad_uav_gazebo pointcloud_to_body.py
```

- Terminal 5: planner

```
cd <path-to-nanobot-ws> && source devel/setup.bash
roslaunch diff_planner gz_single_drone.launch

# 先起飞，进入悬停后在 rviz 使用 2D Nav Goal 进行指点飞行, 可以使用Gazebo中的物体制造障碍飞行环境
```