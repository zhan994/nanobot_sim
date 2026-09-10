# diff_tracked_planning

使用 Fields2Cover 为矩形地块生成全覆盖路径，并以锁存的
`nav_msgs/Path` 发布到 `/coverage_path`。

## 运行

```bash
source /opt/ros/noetic/setup.bash
source ~/diff_drive/devel/setup.bash
roslaunch diff_tracked_planning coverage_planner.launch
```

查看路径：

```bash
rostopic echo -n 1 /coverage_path
```

地块尺寸和机器人参数位于 `config/rectangle_field.yaml`。
