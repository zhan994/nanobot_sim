# diff_tracked

- **diff_tracked_base**: URDF descriptions.
- **diff_tracked_gazebo**: Gazebo simulation.
- **diff_tracked_control**: RPP straight-line controller.
- **diff_tracked_bringup**: Gazebo, FAST-LIO, and controller bringup.

## Run

Gazebo simulation with FAST-LIO and RPP straight-line control:

```bash
roslaunch diff_tracked_bringup tracked_fastlio_control.launch
```
