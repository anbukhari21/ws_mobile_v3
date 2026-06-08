# Experiment Logging

This package includes a passive experiment logger for simulation and real robot
tests. It records advisor-ready step logs, CSV data, summaries, and plots. The
logger subscribes only; it never publishes `/cmd_vel` and never controls the
robot.

## Output Folder

Each run creates a folder under:

```bash
~/robot_test_logs/simulation/YYYY-MM-DD_HH-MM-SS_trial_name
~/robot_test_logs/real_robot/YYYY-MM-DD_HH-MM-SS_trial_name
```

Each trial folder contains:

- `advisor_step_log.txt`: text file with one line per step containing odometry,
  linear velocity, angular velocity, and command velocity.
- `step_log.csv`: full machine-readable step log.
- `step_log.txt`: key-value text log with all CSV fields.
- `metadata.yaml`: trial parameters, model path, world name, and topics.
- `summary.json`: trial metrics.
- `summary.txt`: text version of the summary.
- `trajectory.png`: odometry trajectory with the 2 m circular arena.
- `velocity_plot.png`: odometry and command linear/angular velocities.
- `goal_distance_plot.png`: generated when `/goal_distance` is available.
- `lidar_min_range_plot.png`: generated when `/scan` is available.
- `trial_report.md`: report-ready summary.

The advisor log line format is:

```text
step=000001 time=0.100 odom_x=0.000000 odom_y=0.000000 yaw=0.000000 odom_vx=0.000000 odom_wz=0.000000 cmd_vx=0.050000 cmd_wz=0.100000
```

## Simulation Logging

Terminal 1: start Gazebo and the robot.

```bash
cd /home/abdulrahman/ws_mobile
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch mobile_robot gazebo_model.launch.py \
  world:=stage8_circular_final_realistic_one_human.world \
  gui:=true
```

Terminal 2: start SAC test or controller as usual.

Terminal 3: start the logger.

```bash
cd /home/abdulrahman/ws_mobile
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch mobile_robot experiment_logger.launch.py \
  test_type:=simulation \
  trial_name:=sim_test_01 \
  use_sim_time:=true \
  localization_mode:=sac_only \
  model_path:=/home/abdulrahman/ws_mobile/src/mobile_robot/rl_models/best_eval/my_best_one_dynamic_guardfix_current.zip \
  world_name:=stage8_circular_final_realistic_one_human.world \
  target_x:=0.0 \
  target_y:=0.0
```

Stop the logger with `Ctrl+C`; it will save summaries and plots.

## Real Robot Logging

Terminal 1 on robot: start the base node.

Terminal 2 on robot: start LiDAR.

Terminal 3 on laptop: start AMCL/SAC or the controller stack as usual.

Terminal 4 on laptop: start the logger.

```bash
cd /home/abdulrahman/ws_mobile
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch mobile_robot experiment_logger.launch.py \
  test_type:=real_robot \
  trial_name:=real_test_01 \
  use_sim_time:=false \
  localization_mode:=saved_map_amcl \
  model_path:=/home/abdulrahman/ws_mobile/src/mobile_robot/rl_models/best_eval/my_best_one_dynamic_guardfix_current.zip \
  world_name:=real_robot \
  target_x:=0.0 \
  target_y:=0.0
```

The logger is safe to run during real robot testing because it records only.

## Analyze One Trial

```bash
cd /home/abdulrahman/ws_mobile
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile/install/setup.bash

python3 src/mobile_robot/scripts/analyze_experiment_log.py \
  --log-dir /home/abdulrahman/robot_test_logs/simulation/YYYY-MM-DD_HH-MM-SS_trial_name
```

## Compare All Trials

Simulation:

```bash
python3 src/mobile_robot/scripts/compare_trials.py \
  --root /home/abdulrahman/robot_test_logs/simulation
```

Real robot:

```bash
python3 src/mobile_robot/scripts/compare_trials.py \
  --root /home/abdulrahman/robot_test_logs/real_robot
```

The comparison script creates:

- `all_trials_summary.csv`
- `success_rate_summary.json`
- `success_rate_plot.png`
- `trial_duration_plot.png`
- `final_distance_plot.png`

Use `success_rate_summary.json` to report whether the robot reached at least
80% success over randomized trials.

## Live Monitor

Optional terminal monitor:

```bash
cd /home/abdulrahman/ws_mobile
source /opt/ros/humble/setup.bash
source /home/abdulrahman/ws_mobile/install/setup.bash
export ROS_DOMAIN_ID=10

python3 src/mobile_robot/scripts/live_trial_monitor.py
```

It prints elapsed time, odometry, command velocity, goal distance, goal angle,
minimum LiDAR range, and success status. It is also passive.

## Project Requirement Support

The logger provides:

- per-step odometry, linear velocity, and angular velocity for advisor review;
- simulation and real robot folders for direct comparison;
- trajectory and velocity plots for reports and presentations;
- success-rate aggregation for the 80% randomized-trial requirement;
- LiDAR minimum range plots to support obstacle-tolerance discussion.
