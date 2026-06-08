# Yahboom 4WD + RPLIDAR C1 Gazebo Model

## Files

- Optional robot Xacro: `src/mobile_robot/model/robot_yahboom_rplidar_c1.xacro`
- Optional drive plugin: `src/mobile_robot/model/robot_yahboom_rplidar_c1.gazebo`
- Optional LiDAR Xacro: `src/mobile_robot/model/lidar_rplidar_c1.xacro`
- Optional LiDAR Gazebo sensor: `src/mobile_robot/model/lidar_rplidar_c1.gazebo`
- Launch file: `src/mobile_robot/launch/gazebo_model.launch.py`

The Yahboom/RPLIDAR model is kept as an optional copy. The active launch file still uses `model/robot.xacro`, which has been restored to the previous working robot model.

## Chassis Approximation

- Real target: Yahboom MSPM0 Smart Robot Car 4WD aluminum alloy chassis
- Chassis collision/visual box: `0.260 x 0.170 x 0.055 m`
- Total robot mass placeholder: `2.0 kg`
- Body mass after wheels: `1.4 kg`
- Link tree:
  - `base_footprint`
  - `base_link`
  - `body_link`
  - `wheel1_link`
  - `wheel2_link`
  - `wheel3_link`
  - `wheel4_link`
  - `lidar_link`

## Wheels

- Wheel diameter: `0.065 m`
- Wheel radius: `0.0325 m`
- Wheel width: `0.028 m`
- Wheelbase: `0.175 m`
- Track width: `0.170 m`
- Wheel mass placeholder: `0.15 kg` each
- Wheel joints preserved:
  - `wheel1_joint`
  - `wheel2_joint`
  - `wheel3_joint`
  - `wheel4_joint`

## Drive Plugin

- Plugin: `libgazebo_ros_diff_drive.so`
- Plugin name: `skid_steer_drive`
- Wheel pairs: `2`
- Command topic: `/cmd_vel`
- Odometry topic: `/odom`
- Odometry frame: `odom`
- Robot base frame: `base_footprint`
- Wheel diameter in plugin: `0.065 m`
- Wheel separation in plugin: `0.170 m`

## RPLIDAR C1 Approximation

- Link: `lidar_link`
- Frame used by `/scan`: `lidar_link`
- Visual/collision shape: cylinder
- Diameter: `0.0556 m`
- Height: `0.0413 m`
- Pose relative to `body_link`: `x=0.0, y=0.0, z=0.075 m`
- Approximate LiDAR center height from ground with current spawn pose: `0.155 m`

Gazebo ray sensor settings:

- Topic: `/scan`
- Output type: `sensor_msgs/LaserScan`
- Field of view: 360 degrees
- Min angle: `-3.14159`
- Max angle: `3.14159`
- Min range: `0.05 m`
- Max range: `12.0 m`
- Update rate: `10 Hz`
- Samples: `360`
- Range resolution: `0.01 m`

The RL environment still downsamples `/scan` to its existing 36 LiDAR beams, so SAC observation dimensions are unchanged.

## RL Compatibility

- `/cmd_vel` preserved.
- `/odom` preserved.
- `/scan` preserved.
- Observation dimensions unchanged.
- Action dimensions unchanged.
- Stable-Baselines3 SAC checkpoints remain compatible at the software interface level.

Because the robot footprint changed, watch collision behavior in Gazebo. Good starting values remain:

- `--collision-distance 0.18`
- `--near-obstacle-distance 0.30`
- `--goal-threshold 0.22`

## Validation Commands

```bash
source /opt/ros/humble/setup.bash
source ~/ws_mobile/install/setup.bash
cd ~/ws_mobile
colcon build --packages-select mobile_robot
source install/setup.bash
ros2 launch mobile_robot gazebo_model.launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_mobile/install/setup.bash

ros2 topic list | grep -E "scan|odom|cmd_vel|robot_description"
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

Stop command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1
```

TF check, if available:

```bash
ros2 run tf2_tools view_frames
```

## Real Robot Measurements To Confirm

- Chassis length
- Chassis width
- Chassis height
- Wheelbase
- Track width
- LiDAR height from ground
- LiDAR x/y offset from robot center
- Total robot mass
- Wheel width
- Distance from LiDAR to front bumper
- Real maximum safe linear velocity
- Real maximum safe angular velocity
