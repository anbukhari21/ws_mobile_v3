# Localization Modes

This adds selectable goal-source modes without changing the trained SAC input
shape. All modes publish:

- `/goal_distance` (`std_msgs/Float32`)
- `/goal_angle` (`std_msgs/Float32`)
- `/goal_valid` (`std_msgs/Bool`)
- `/goal_reached` (`std_msgs/Bool`)

The SAC controller must stop when `/goal_valid` is missing, stale, or false.

## 1. Saved Map AMCL

Use the existing saved-map localization behavior.

```bash
cd /home/abdulrahman/ws_mobile
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch mobile_robot localization_modes.launch.py \
  localization_mode:=saved_map_amcl \
  use_sim_time:=false \
  map:=/path/to/map.yaml \
  base_frame:=base_footprint
```

## 2. Online SLAM Return To Start

No saved map. SLAM Toolbox publishes `map -> odom`; the start pose is treated
as `(0, 0)` in the SLAM map frame.

```bash
cd /home/abdulrahman/ws_mobile
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch mobile_robot localization_modes.launch.py \
  localization_mode:=online_slam_return_start \
  use_sim_time:=false \
  base_frame:=base_footprint
```

## 3. Arena Center From LiDAR

No saved map. The node fits the circular wall from `/scan` and publishes the
estimated arena center as the SAC goal.

```bash
cd /home/abdulrahman/ws_mobile
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch mobile_robot localization_modes.launch.py \
  localization_mode:=arena_center_lidar \
  use_sim_time:=false \
  scan_topic:=/scan
```

## Topic Checks

```bash
ros2 topic echo /goal_distance
ros2 topic echo /goal_angle
ros2 topic echo /goal_valid
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo map base_footprint
```

Use `base_link` instead of `base_footprint` only if that is the TF frame your
robot actually publishes.
