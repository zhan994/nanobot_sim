# diff_car

- **diff_car_base**: URDF decriptions.
- **diff_car_gazebo**: Gazebo Simulation.

## Run

```bash
roslaunch diff_car_gazebo diff_car_gazebo.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.3 _turn:=1.0 _repeat_rate:=10.0 _key_timeout:=0.5 cmd_vel:=/cmd_vel
```