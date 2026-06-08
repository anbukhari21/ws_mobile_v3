#!/usr/bin/env bash
set +e

WORKSPACE="/home/abdulrahman/ws_mobile_v2"
OUT_DIR="$WORKSPACE/experiment_logs"
OUT_FILE="$OUT_DIR/ros_graph_snapshot.txt"

mkdir -p "$OUT_DIR"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f "$WORKSPACE/install/setup.bash" ]; then
  source "$WORKSPACE/install/setup.bash"
fi

{
  echo "ROS graph snapshot"
  echo "Captured at: $(date --iso-8601=seconds)"
  echo "Workspace: $WORKSPACE"
  echo

  echo "===== ros2 node list ====="
  ros2 node list
  echo

  echo "===== ros2 topic list ====="
  ros2 topic list
  echo

  echo "===== ros2 topic info /cmd_vel -v ====="
  ros2 topic info /cmd_vel -v
  echo

  echo "===== ros2 topic info /odom -v ====="
  ros2 topic info /odom -v
  echo

  echo "===== ros2 topic info /scan -v ====="
  ros2 topic info /scan -v
  echo

  echo "===== ros2 topic info /goal_distance -v ====="
  ros2 topic info /goal_distance -v
  echo

  echo "===== ros2 topic info /plan -v ====="
  ros2 topic info /plan -v
  echo
} > "$OUT_FILE" 2>&1

echo "Saved ROS graph snapshot to: $OUT_FILE"
