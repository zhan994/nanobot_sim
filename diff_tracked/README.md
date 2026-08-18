# diff_tracked

- **diff_tracked_base**: URDF descriptions.
- **diff_tracked_gazebo**: Gazebo simulation.
- **diff_tracked_control**: RPP straight-line controller.
- **diff_tracked_bringup**: Gazebo, Rviz and controller bringup.

## Run

Gazebo simulation with keyboard control:

```bash
roslaunch diff_tracked_gazebo bunker_gazebo.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.3 _turn:=1.0 _repeat_rate:=10.0 _key_timeout:=0.5 cmd_vel:=/cmd_vel
```


Gazebo simulation with RPP straight-line control:

```bash
roslaunch diff_tracked_bringup tracked_rpp_control.launch
```
