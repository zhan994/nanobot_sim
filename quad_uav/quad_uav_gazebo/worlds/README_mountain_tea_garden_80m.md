# 80 × 80 m 山地茶园路径规划场景

`mountain_tea_garden_80m.world` 面向 Gazebo Classic / ROS1 的山地农业路径
规划与避障测试。坐标原点位于场景中心，边界为 `x,y ∈ [-40, 40] m`。

## 场景内容

- 257 × 257 可碰撞高度图，地面高度为 0.50–17.50 m；
- 中位坡度约 12°，95 分位坡度约 25.5°，最大坡度约 32°；
- 16 条随等高线轻微弯曲的茶垄，垄间距约 3.35 m，并有 5 组横向缺口；
- 5 根电线杆和 3 根分段下垂导线；
- 树木、岩石、水箱、工具房、木箱、倒木、灌溉管和排水沟；
- 80 × 80 m 周界围栏；
- 蓝色起点 `(-34, -33)` 与红色终点 `(33, 33)`。

茶垄、电杆、导线、树木和常见障碍都有 collision，可被 ray/lidar、深度相机
和 Gazebo 接触检测感知。起终点标识只用于显示，不参与碰撞。

## 启动

从 `quad_uav` 目录启动 PX4 SITL：

```bash
cd ~/nanobot_ws/src/nanobot_sim/quad_uav
./quad_uav_gazebo/scripts/rspx4_tea_garden_80m.sh
```

只启动 Gazebo（不启动 PX4）：

```bash
export GAZEBO_MODEL_PATH="$PWD/quad_uav_gazebo/models:${GAZEBO_MODEL_PATH:-}"
gazebo quad_uav_gazebo/worlds/mountain_tea_garden_80m.world
```

## 坐标与起点

| 项目 | 坐标 |
|---|---|
| 场景范围 | `x,y = [-40, 40] m` |
| 蓝色起点地面 | `(-34, -33, 4.03 m)` 左右 |
| 红色终点地面 | `(33, 33, 13.83 m)` 左右 |
| PX4 默认出生点 | `(-34, -33, 4.8 m)` |

高度图会产生局部坡度；地面机器人应根据其离地间隙在出生时额外增加
`0.15–0.40 m` 的 Z 高度。当前场景包含超过 25° 的局部陡坡，地面机器人
规划时应同时设置最大允许坡度或倾覆代价。

## 重新生成与调参

```bash
python3 quad_uav_gazebo/worlds/generate_mountain_tea_garden_80m.py
```

默认随机种子为 `20260730`。若要生成另一组茶株尺寸和岩石尺寸：

```bash
python3 quad_uav_gazebo/worlds/generate_mountain_tea_garden_80m.py --seed 42
```

主要参数位于生成脚本顶部：

- `terrain_height()`：山坡形状与高度；
- `add_tea_rows()`：茶垄间距、缺口和尺寸；
- `add_power_line()`：电杆、线高及导线下垂；
- `add_rocks()` / `add_infrastructure()`：离散障碍布局。

改变解析度时，Gazebo heightmap 的边长应保持 `2^n + 1`，例如 129、257 或
513。
