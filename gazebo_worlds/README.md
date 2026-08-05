# gazebo_worlds

***Note!!! Copy models in folder `models/` to `/home/<usrname>/.gazebo/models` to load those in worlds.***

## Run

Source the workspace and launch the default world `weston_robot_empty.world`.  

```bash
roslaunch gazebo_worlds world.launch
```

Select another world with the `world_name` argument.  

```bash
roslaunch gazebo_worlds world.launch \
  world_name:=$(rospack find gazebo_worlds)/worlds/test_city.world
```
