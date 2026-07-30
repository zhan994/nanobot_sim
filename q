[33mcommit c44a3f92f1d87a917b24fbad1fa931576467b1b2[m[33m ([m[1;31morigin/feature_uav[m[33m)[m
Author: nixe-kzh <kc.zh@foxmail.com>
Date:   Wed Jul 22 11:17:43 2026 +0800

    fix readme

[1mdiff --git a/quad_uav/README.md b/quad_uav/README.md[m
[1mindex a09bdf0..93e4ed7 100644[m
[1m--- a/quad_uav/README.md[m
[1m+++ b/quad_uav/README.md[m
[36m@@ -6,7 +6,7 @@[m
 - ubuntu 20.04[m
 - ros1 noetic[m
 [m
[31m-> 实验环境：CPU=Intel 14900HX; GPU=Nvidia RTX4090Laptop; RAM=32GB[m
[32m+[m[32m> 实验环境：WSL2 CPU=Intel 14900HX; GPU=Nvidia RTX4090Laptop; RAM=32GB[m
 [m
 [m
 ### 1. 依赖安装[m
[36m@@ -23,7 +23,7 @@[m [mgawk wget zip unzip tar bzip2 flex bison libgstreamer1.0-dev \[m
 libgstreamer-plugins-base1.0-dev libsdl2-dev libsdl2-image-dev \[m
 libopenjp2-7 libtiff5 libjpeg-dev[m
 [m
[31m-pip3 install kconfiglib jsonschema jinja2 future lxml pyros-genmsg[m
[32m+[m[32mpip3 install kconfiglib jsonschema jinja2 future lxml pyros-genmsg empy==3.3.4 pyyaml[m
 ```[m
 [m
 2. mavros[m
[36m@@ -39,7 +39,7 @@[m [msudo ./install_geographiclib_datasets.sh[m
 [m
 1. 代码下载[m
 ```[m
[31m-cd[m
[32m+[m[32mcd ~[m
 git clone -b dev_nanobot https://github.com/zhan994/PX4-Autopilot.git px4_dev[m
 ```[m
 > 注意分支: 'Your branch is up to date with 'origin/dev_nanobot'.'[m
[36m@@ -81,7 +81,8 @@[m [mmake px4_sitl gazebo[m
 1. 下载模型文件[m
 [m
 ```[m
[31m-cd[m
[32m+[m[32mmkdir ~/nanobot_ws/src[m
[32m+[m[32mcd ~/nanobot_ws/src[m
 git clone -b feature_uav https://github.com/zhan994/nanobot_sim.git[m
 ```[m
 [m
[36m@@ -104,15 +105,16 @@[m [msudo apt install ros-noetic-velodyne-gazebo-plugins[m
 [m
 1. 赋予执行权限[m
 ```[m
[31m-cd nanobot_sim/quad_uav[m
[31m-sudo chmod +x  ./quad_uav_base/*[m
[32m+[m[32mcd ~/nanobot_ws/src/nanobot_sim/quad_uav[m
[32m+[m[32mchmod +x ./quad_uav_gazebo/scripts/*.sh[m
[32m+[m[32mchmod +x ./quad_uav_gazebo/scripts/*.py[m
 ```[m
 [m
 2. 启动 px4-sitl[m
 [m
 ```[m
[31m-cd nanobot_sim/quad_uav[m
[31m-./quad_uav_base/rspx4sitl_beta.sh[m
[32m+[m[32mcd ~/nanobot_ws/src/nanobot_sim/quad_uav[m
[32m+[m[32m./quad_uav_gazebo/scripts/rspx4.sh[m
 ```[m
 [m
 3. 启动点云转换[m
[36m@@ -120,15 +122,15 @@[m [mcd nanobot_sim/quad_uav[m
 > 使用脚本将雷达系点云旋转至Body系[m
 [m
 ```[m
[31m-cd nanobot_sim/quad_uav[m
[31m-python3 ./quad_uav_base/pointcloud_to_body.py[m
[32m+[m[32mcd ~/nanobot_ws/src/nanobot_sim/quad_uav[m
[32m+[m[32mpython3 ./quad_uav_gazebo/scripts/pointcloud_to_body.py[m
 ```[m
 [m
 4. **结束后**清理环境[m
 [m
 ```[m
[31m-cd nanobot_sim/quad_uav[m
[31m-./quad_uav_base/clean_env.sh[m
[32m+[m[32mcd ~/nanobot_ws/src/nanobot_sim/quad_uav[m
[32m+[m[32m./quad_uav_gazebo/scripts/clean_env.sh[m
 ```[m
 [m
 [m
[36m@@ -161,27 +163,28 @@[m [mroscd quad_uav_gazebo/[m
 ./scripts/rspx4.sh[m
 ```[m
 > 记得 chmod +x 给权限[m
[32m+[m[32m`chmod +x ./scripts/rspx4.sh`[m
 [m
 - Terminal 2: 启动px4ctrl[m
 [m
 ```[m
 cd ~/nanobot_ws && source devel/setup.bash[m
 roslaunch px4ctrl run_ctrl_sim.launch[m
[31m-````[m
[32m+[m[32m```[m
 [m
 - Terminal 3: 启动 rc sim[m
 [m
 ```[m
 cd ~/nanobot_ws && source devel/setup.bash[m
 rosrun quad_uav_gazebo rc_sim.py[m
[31m-````[m
[32m+[m[32m```[m
 > 输入 '1' 起飞[m
 [m
 - Terminal 4: 启动点云转换[m
 [m
 ```[m
 cd ~/nanobot_ws && source devel/setup.bash[m
[31m-rosrun  quad_uav_gazebo pointcloud_to_body.py[m
[32m+[m[32mrosrun quad_uav_gazebo pointcloud_to_body.py[m
 ```[m
 [m
 - Terminal 5: 启动 planner[m
[36m@@ -191,4 +194,4 @@[m [mcd ~/nanobot_ws && source devel/setup.bash[m
 roslaunch diff_planner gz_single_drone.launch[m
 ```[m
 [m
[31m-> 先起飞，进入悬停后 rviz 使用 2D Goal 进行指点飞行[m
\ No newline at end of file[m
[32m+[m[32m> 先起飞，进入悬停后在 rviz 使用 2D Nav Goal 进行指点飞行, 可以使用Gazebo中的物体制造障碍飞行环境[m
\ No newline at end of file[m
