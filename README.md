# ws_mobile_v3

Senior project software for autonomous indoor mobile robot navigation.

This workspace contains the ROS 2 Humble package for the Yahboom 4WD robot, Gazebo Classic simulation assets, RPLIDAR C1 sensor setup, AMCL-based localization support, SAC navigation nodes, trained RL model files, maps, launch files, and validation utilities.

## Contents

- `src/mobile_robot`: ROS 2 package source.
- `rl_models`: trained SAC model and project diagram.
- `src/mobile_robot/model`: Gazebo worlds and robot/obstacle models.
- `src/mobile_robot/maps`: saved map files.
- `src/mobile_robot/config`: localization and SAC navigation parameters.
- `src/mobile_robot/launch`: launch files for simulation, localization, RViz, and SAC navigation.
- `src/mobile_robot/scripts`: helper scripts for robot trials, logging, map saving, and endurance checks.

## Build

```bash
cd /home/abdulrahman/ws_mobile_v3
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Gazebo Check

```bash
cd /home/abdulrahman/ws_mobile_v3
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot sac_gazebo.launch.py gui:=true
```

## SAC Navigation Check

```bash
cd /home/abdulrahman/ws_mobile_v3
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot sac_center_navigation.launch.py \
  use_gazebo:=true \
  gui:=true \
  use_sim_time:=true \
  run_motion:=true \
  run_logger:=true \
  map:=/home/abdulrahman/ws_mobile_v3/src/mobile_robot/maps/arena_map.yaml \
  model:=/home/abdulrahman/ws_mobile_v3/rl_models/sac_goal_policy.zip
```

## Notes

Generated ROS build folders are intentionally not tracked:

- `build/`
- `install/`
- `log/`

Replay buffers are also ignored because they are large and not needed for normal sharing or inference.
