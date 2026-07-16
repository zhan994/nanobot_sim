# PX4 Sim

## Step 1: PX4-Autopilot 编译

### 0. 先决条件
- ubuntu 20.04
- ros1 noetic

> 实验环境：CPU=Intel 14900HX; GPU=Nvidia RTX4090Laptop; RAM=32GB


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

pip3 install kconfiglib jsonschema jinja2 future lxml pyros-genmsg
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
cd
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
cd
git clone -b feature_uav https://github.com/zhan994/nanobot_sim.git
```

2. 移动模型

```
cp -r ~/nanobot_sim/quad_uav/quad_uav_gazebo/models/* ~/px4_dev/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
cp -r ~/nanobot_sim/quad_uav/quad_uav_gazebo/worlds/* ~/px4_dev/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds
```

3. 安装依赖

```
sudo apt install ros-noetic-velodyne-gazebo-plugins
```



## Step 3: PX4 SITL 环境启动

1. 给可执行权限
```
cd nanobot_sim/quad_uav
sudo chmod +x  ./quad_uav_base/sh/rspx4sitl.sh
```

2. 启动脚本

```
cd nanobot_sim/quad_uav
./quad_uav_base/sh/rspx4sitl.sh
```