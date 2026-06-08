# SAC Workflow

This workflow runs mapping and the real robot SAC test with control software running on the robot. The laptop is used for RViz, copying maps/logs, and checking results.

## Terminal 1: Robot, Start Drivers

Run on laptop:

```bash
ssh abdulrhman@JTS.local
```
Or:
```bash
ssh abdulrhman@JTS.local
```

Run on robot:

```bash
sudo systemctl start fastdds-discovery.service
sudo systemctl start robot-laser-tf.service
sudo systemctl start rplidar.service
sudo systemctl start yahboom-base.service

cd /home/abdulrhman/ws_mobile_v3
source src/mobile_robot/scripts/source_robot_local_v3.sh

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /odom --once
```

## Terminal 2: Laptop RViz

Run on laptop:

```bash
cd /home/abdulrahman/ws_mobile_v3
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile_v3/install/setup.bash

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_SUPER_CLIENT=TRUE
export ROS_DISCOVERY_SERVER=JTS.local:11811

ros2 launch mobile_robot sac_rviz.launch.py
```

In RViz:

- Fixed Frame: `map`
- Add `Map`
- Add `LaserScan`, topic `/scan`
- Add `Odometry`, topic `/odom`
- Add `RobotModel`
- Add `ParticleCloud`, topic `/particle_cloud`, reliability `Best Effort`
- Add `TF`
- `Center 22 cm Circle` is loaded automatically on `/center_success_radius_marker`

Do not use `2D Pose Estimate`.

## Terminal 3: Robot, Start Mapping

Run on robot:

```bash
cd /home/abdulrhman/ws_mobile_v3
source src/mobile_robot/scripts/source_robot_local_v3.sh

ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/abdulrhman/ws_mobile_v3/src/mobile_robot/config/mapper_params_online_async.yaml \
  use_sim_time:=false
```

Leave this running while mapping.

## Terminal 4: Robot, Teleop While Mapping

Run on robot:

```bash
cd /home/abdulrhman/ws_mobile_v3
source src/mobile_robot/scripts/source_robot_local_v3.sh

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel
```

Use:

```text
i = forward
, = backward
j = turn left
l = turn right
k = stop
q/z = increase/decrease speed
w/x = increase/decrease linear speed only
e/c = increase/decrease angular speed only
Ctrl+C = quit
```

Drive slowly while mapping.

## Terminal 5: Robot, Save Map

After mapping is complete, run on robot:

```bash
cd /home/abdulrhman/ws_mobile_v3
source src/mobile_robot/scripts/source_robot_local_v3.sh

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

mkdir -p /home/abdulrhman/maps

MAP_NAME=/home/abdulrhman/maps/real_robot_sac_map_$(date +%Y%m%d_%H%M%S)

src/mobile_robot/scripts/save_sac_map.sh "$MAP_NAME"

echo "${MAP_NAME}.yaml" > /home/abdulrhman/ws_mobile_v3/latest_map_yaml.txt
cat /home/abdulrhman/ws_mobile_v3/latest_map_yaml.txt
```

Stop SLAM in Terminal 3 with `Ctrl+C`.

## Terminal 6: Robot, Run Trial

Run on robot:

The terminal output is compact. During motion it prints one line for each robot action with goal distance, goal angle, AMCL pose, and `/cmd_vel`.

```bash
cd /home/abdulrhman/ws_mobile_v3
source src/mobile_robot/scripts/source_robot_local_v3.sh

MAP_YAML=$(cat latest_map_yaml.txt)
MODEL=/home/abdulrhman/ws_mobile_v3/rl_models/sac_goal_policy.zip

SUCCESS_RADIUS=0.22 \
TRIAL_PREFIX=robot_local_sac_goal_test \
LOCALIZATION_SERVICE_WAIT_TIMEOUT=30 \
POST_ROTATE_SETTLE_SEC=2 \
AMCL_STARTUP_EXTRA_SEC=8 \
AMCL_POSE_REQUIRED=true \
AMCL_READY_TIMEOUT=25 \
AMCL_GLOBAL_RETRIES=1 \
EVIDENCE_DELAY_SEC=8 \
AMCL_POST_LAUNCH_NOMOTION_SEC=10 \
src/mobile_robot/scripts/run_real_amcl_sac_trial_with_logging.sh robot_local_01 \
  --model "$MODEL" \
  --map "$MAP_YAML" \
  --goal-x 0.0 \
  --goal-y 0.0 \
  --goal-yaw 0.0 \
  --trial-duration 120 \
  --rviz false \
  --require-amcl-ready true \
  --auto-global-localization true \
  --auto-rotate true \
  --rotate-speed 0.20 \
  --rotate-duration 35
```

## Optional: Robot, Live Trial Monitor

Run on robot in another terminal while Terminal 6 is running:

```bash
cd /home/abdulrhman/ws_mobile_v3
source src/mobile_robot/scripts/source_robot_local_v3.sh

python3 src/mobile_robot/scripts/live_trial_monitor.py
```

This is passive. It does not publish `/cmd_vel`.

It prints live values once per second:

- `goal=(distance, angle)`
- `cmd=(linear, angular)`
- `odom=(x, y, yaw)`
- `min_lidar`
- `success`

## Optional: Laptop, Live Graph Monitor

Run on laptop while Terminal 6 is running:

```bash
cd /home/abdulrahman/ws_mobile_v3
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile_v3/install/setup.bash

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_SUPER_CLIENT=TRUE
export ROS_DISCOVERY_SERVER=JTS.local:11811

python3 src/mobile_robot/scripts/live_trial_plotter.py --success-radius 0.22 --window-sec 240
```

This is passive. It does not publish `/cmd_vel`.

It draws live:

- goal distance and goal angle
- commanded and odometry linear velocity
- commanded and odometry angular velocity

## Optional: Battery Endurance Monitor

Use this to validate runtime. Do not bypass the battery protection circuit. If
you intentionally run until the robot powers off, expect SSH and robot-side logs
to stop abruptly.

First check whether the robot publishes battery data:

```bash
cd /home/abdulrahman/ws_mobile_v3
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile_v3/install/setup.bash

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_SUPER_CLIENT=TRUE
export ROS_DISCOVERY_SERVER=JTS.local:11811

ros2 topic list | grep -iE "battery|batt|volt|power|charge"
```

Current robot note: the Yahboom base driver publishes `/odom` from motor-speed
feedback, but it does not expose battery voltage. If this command prints
nothing, use the timer-only view unless a real voltage sensor/topic is added.

Run the battery graph with a large timer:

```bash
cd /home/abdulrahman/ws_mobile_v3
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile_v3/install/setup.bash

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_SUPER_CLIENT=TRUE
export ROS_DISCOVERY_SERVER=JTS.local:11811

LOG_DIR=/home/abdulrahman/ws_mobile_v3/experiment_logs/endurance_battery_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG_DIR"

python3 src/mobile_robot/scripts/battery_endurance_monitor.py \
  --log-path "$LOG_DIR/battery_endurance.csv"
```

Timer-only view:

```bash
python3 src/mobile_robot/scripts/battery_endurance_monitor.py \
  --timer-only \
  --show-velocity \
  --ssh-robot-host abdulrhman@JTS.local \
  --log-path "$LOG_DIR/battery_endurance.csv"
```

If the battery topic is not auto-detected, pass it directly:

```bash
python3 src/mobile_robot/scripts/battery_endurance_monitor.py \
  --battery-topic /battery_state \
  --topic-type battery_state \
  --log-path "$LOG_DIR/battery_endurance.csv"
```

For the all-drivers spin load, run on the robot. This starts Fast DDS
discovery, the laser TF, RPLIDAR, and the Yahboom base driver before motion:

```bash
cd /home/abdulrhman/ws_mobile_v3
source src/mobile_robot/scripts/source_robot_local_v3.sh

SPIN_MODE=alternate \
SPIN_DIRECTION=left \
SPIN_INTERVAL_SEC=180 \
SPIN_SPEED=0.20 \
DURATION_SEC=0 \
STATUS_PERIOD_SEC=60 \
src/mobile_robot/scripts/run_endurance_spin_test.sh
```

`DURATION_SEC=0` means keep spinning until you stop it or the robot powers off.
For a fixed 65 minute test, use `DURATION_SEC=3900`.
`SPIN_MODE=alternate` with `SPIN_INTERVAL_SEC=180` means 3 minutes left,
3 minutes right, then repeat. It starts with `SPIN_DIRECTION=left`.

Live RViz view for the endurance test:

```bash
ROBOT_HOST=JTS.local
ROBOT_IP=$(ssh abdulrhman@$ROBOT_HOST 'echo $SSH_CONNECTION' | awk '{print $3}')
echo "Robot IP: $ROBOT_IP"

cd /home/abdulrahman/ws_mobile_v3
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile_v3/install/setup.bash

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_SUPER_CLIENT=TRUE
export ROS_DISCOVERY_SERVER=$ROBOT_IP:11811

ros2 launch mobile_robot sac_rviz.launch.py \
  rviz_config:=/home/abdulrahman/ws_mobile_v3/src/mobile_robot/rviz/sac_real_robot_endurance.rviz \
  model_xacro:=/home/abdulrahman/ws_mobile_v3/src/mobile_robot/model/robot.xacro \
  show_center_success_circle:=false
```

When stopping manually, press `Ctrl+C`. The script sends zero velocity. If you
want to send another stop command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## Terminal 7: Robot, Save ROS Graph During Trial

Run while the trial is running:

```bash
cd /home/abdulrhman/ws_mobile_v3
source src/mobile_robot/scripts/source_robot_local_v3.sh

TRIAL=$(ls -dt /home/abdulrhman/ws_mobile_v3/experiment_logs/robot_local_sac_goal_test_* | head -n 1)

{
  date
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
  echo "--- nodes ---"
  ros2 node list
  echo "--- topics ---"
  ros2 topic list
  echo "--- cmd_vel info ---"
  ros2 topic info /cmd_vel --verbose
  echo "--- scan info ---"
  ros2 topic info /scan --verbose
  echo "--- odom info ---"
  ros2 topic info /odom --verbose
  echo "--- plan info ---"
  ros2 topic info /plan --verbose
  echo "--- amcl pose info ---"
  ros2 topic info /amcl_pose --verbose
  echo "--- services ---"
  ros2 service list
  echo "--- tf map to base_footprint ---"
  timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
} > "$TRIAL/ros_graph_snapshot.txt" 2>&1
```

## Terminal 8: Laptop, Copy Map And Logs

Run on laptop:

```bash
mkdir -p /home/abdulrahman/maps
mkdir -p /home/abdulrahman/ws_mobile_v3/experiment_logs

MAP_YAML=$(ssh abdulrhman@JTS.local 'cat /home/abdulrhman/ws_mobile_v3/latest_map_yaml.txt')
MAP_BASE=$(basename "$MAP_YAML" .yaml)

rsync -az --progress \
  "abdulrhman@JTS.local:/home/abdulrhman/maps/${MAP_BASE}.yaml" \
  "abdulrhman@JTS.local:/home/abdulrhman/maps/${MAP_BASE}.pgm" \
  /home/abdulrahman/maps/

rsync -az --progress \
  abdulrhman@JTS.local:/home/abdulrhman/ws_mobile_v3/experiment_logs/robot_local_sac_goal_test_* \
  /home/abdulrahman/ws_mobile_v3/experiment_logs/
```

## Terminal 9: Laptop, Check Result

Run on laptop:

```bash
TRIAL=$(ls -dt /home/abdulrahman/ws_mobile_v3/experiment_logs/robot_local_sac_goal_test_* | head -n 1)

cat "$TRIAL/experiment_validation_summary.txt"
cat "$TRIAL/ros_graph_snapshot.txt"
ls -l "$TRIAL/plots"
```

Look for:

```text
Result using success radius: SUCCESS
Number of collision flags: 0
control_source is sac_autonomy: yes
```

## Terminal 10: Robot, Stop Drivers

Run on robot:

```bash
cd /home/abdulrhman/ws_mobile_v3
source src/mobile_robot/scripts/source_robot_local_v3.sh

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

sudo systemctl stop rplidar.service
sudo systemctl stop robot-laser-tf.service
sudo systemctl stop yahboom-base.service
sudo systemctl stop fastdds-discovery.service
```
