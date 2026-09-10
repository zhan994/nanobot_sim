# diff_tracked

- **diff_tracked_base**: URDF descriptions.
- **diff_tracked_gazebo**: Gazebo simulation.
- **diff_tracked_control**: Straight-line RPP and general path RPP controllers.
- **diff_tracked_planning**: Fields2Cover coverage path planner.
- **diff_tracked_bringup**: Gazebo, Rviz and controller bringup.
- **diff_tracked_localization**: robot-localization configuration

## Run

Gazebo simulation with RPP straight-line control:

```bash
roslaunch diff_tracked_bringup tracked_rpp_control.launch
```

Saved-map localization keeps the controller stopped until the corrected map
origin is confirmed. After Gazebo, EKF, lidar and slam_toolbox are ready, run:

```bash
rosservice call /confirm_map_origin
```

The controller then waits for the `map -> odom -> base_link` pose to remain
stable before recording its start pose and publishing motion commands.

Gazebo simulation with Fields2Cover coverage control:

```bash
roslaunch diff_tracked_bringup tracked_coverage_control.launch
```

The coverage launch uses `rectangle_field.world` by default. Its open area is
the same `x = 0..30 m`, `y = 0..20 m` rectangle configured in
`rectangle_field.yaml`, and the robot starts at the first coverage-path pose.

To open only this field and the robot:

```bash
roslaunch diff_tracked_gazebo rectangle_field_gazebo.launch
```

The planner publishes `/coverage_path`. The independent
`path_rpp_controller_node` receives that path and publishes `/cmd_vel`.
The original `line_rpp_controller_node` is unchanged and can still be used
through `tracked_rpp_control.launch`.

The saved-map mode also starts stopped. Confirm the map origin after Gazebo,
lidar and slam_toolbox are ready:

```bash
rosservice call /confirm_map_origin
```

To change the field, edit
`diff_tracked_planning/config/rectangle_field.yaml`.
