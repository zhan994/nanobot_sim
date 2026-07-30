# 120 × 120 m 无人机复杂雷达测试场

`uav_complex_120m.world` 是面向前向 50 m 探测与局部规划测试的 Gazebo
Classic 场景。默认起飞点为 `(0, 0, 0.1)`，机头按 PX4 默认方向朝向 `+X`。

## 一键启动

```bash
cd ~/nanobot_ws/src/nanobot_sim/quad_uav
./quad_uav_gazebo/scripts/rspx4_complex_120m.sh
```

点云计算压力较大时可关闭 Gazebo GUI：

```bash
GUI_ENABLE=false ./quad_uav_gazebo/scripts/rspx4_complex_120m.sh
```

另开终端启动世界坐标点云转换和电线随机漏检模型：

```bash
cd ~/nanobot_ws
source devel/setup.bash
roslaunch quad_uav_gazebo complex_120m_pointcloud.launch
```

过滤器只作用于三根输电线附近的点。它按距离和观察角度降低检测概率，
并让相邻 `0.5 m` 线段在连续帧间保持相关状态，因此点云中的电线会局部、
断续地出现，而不会整根同时闪烁。电杆附近的点不参与随机丢弃，场景碰撞体
也保持不变。

默认随机种子固定为 `120`，同样的输入可重复实验；需要另一组随机序列时：

```bash
roslaunch quad_uav_gazebo complex_120m_pointcloud.launch random_seed:=2026
```

可在 `launch/complex_120m_pointcloud.launch` 中调整近距离检测率、距离衰减、
时间连续性和分段长度。其他 world 继续直接运行 `pointcloud_to_body.py`，
默认不会启用电线过滤。

也可以继续使用通用启动脚本：

```bash
WORLD_FILE="$PWD/quad_uav_gazebo/worlds/uav_complex_120m.world" \
./quad_uav_gazebo/scripts/rspx4.sh
```

## 场景分区

- 中心：6 m 直径安全起飞区、十字沥青道路和车道标线。
- `+X` 雷达走廊：12 m 门框；20–42 m 高低错落柱体；48 m 高门架；
  54 m 窄缝墙。它既覆盖 50 m 探测范围，也提供一个略超量程参照物。
- 西北：仓库、附属建筑、筒仓、烟囱和三层管廊。
- 东北：14 个可堆叠集装箱、装卸车辆和不规则狭窄通道。
- 西南：六栋 6–13 m 高的砖墙建筑、街巷、车辆和路牌。
- 东南：16 棵 6–9 m 高的树、不同尺寸的岩石和低空林间缝隙。
- 南部：不同高度的混凝土迷宫、矮墙和近地杂物。
- 全场：120 × 120 m 围栏、跨场输电杆/细导线、12 m 施工吊架及悬吊物。

## 纹理与碰撞

场景使用沥青、草地、旧砖墙和锈蚀波纹钢四种项目内纹理。纹理位于：

```text
quad_uav_gazebo/models/uav_complex_scene/materials/
```

装饰性道路标线和起飞坪没有碰撞体；门框、导线、树冠、建筑、车辆、
管道、集装箱及主要杂物均有碰撞体，可被 Gazebo ray/lidar 传感器检测。

## 重新生成

世界文件由确定性脚本生成。调整布局后执行：

```bash
python3 quad_uav_gazebo/worlds/generate_uav_complex_120m.py
```

生成结果固定为 `quad_uav_gazebo/worlds/uav_complex_120m.world`。
