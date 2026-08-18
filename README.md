# NanoBot_Sim

***A repo. which maintains simulation tools in Gazebo for Robotics.***

![](nanobot_sim.png)

**Supported Platforms**

- ROS-Noetic on Ubuntu20.04
<!-- - ROS-One on Ubuntu22.04 -->

**Vehicles**

- **diff_car**: a simple differential robot equipped with 2-wheel drive.
- **diff_tracked**: a compact tracked mobile robot specifically designed to tackle demanding off-road terrains, as well as confined and challenging work areas.
- **quad_uav**: a simple quadrotor drone based on PX4 firmware.

**Shared simulation resources**

- **gazebo_worlds**: vehicle-independent Gazebo worlds and the common world launcher.

## Third-party

- Gazebo 11
- other related tools
  
  ```bash
  # robot control middleware and kinematics plugins of common models
  sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers
  
  # communication interface between Gazebo and ROS 
  sudo apt install ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control
  
  # visualize the joint control
  sudo apt install ros-noetic-joint-state-publisher-gui
  
  # a ROS tool closely related to robot motion control
  sudo apt-get install ros-noetic-rqt-robot-steering 
 
  # keyboard or joy control
  sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-joy ros-noetic-teleop-twist-joy

  # sensors
  sudo apt install ros-noetic-velodyne*
  ```

## Run 

```bash
mkdir -p ws_nanobot_sim/src
cd ws_nanobot_sim/src
git clone git@github.com:zhan994/nanobot_sim.git
cd ..
catkin_make
```

Launch a world without spawning a vehicle:

```bash
roslaunch gazebo_worlds world.launch \
  world_name:=$(rospack find gazebo_worlds)/worlds/iscas_museum.world
```

Vehicle launch files accept the same `world_name` argument, so any shared world can be selected without editing the package:

```bash
roslaunch diff_car_gazebo diff_car_gazebo.launch \
  world_name:=$(rospack find gazebo_worlds)/worlds/clearpath_playpen.world
```

## Related Work

- [ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim): AgileX Product Gazebo Simulate.
- [gazebo_models](https://github.com/osrf/gazebo_models): Gazebo database of SDF models.