#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/abdulrhman/ws_mobile_v3}"
SPIN_SPEED="${SPIN_SPEED:-0.20}"
SPIN_DIRECTION="${SPIN_DIRECTION:-left}"
SPIN_MODE="${SPIN_MODE:-single}"
SPIN_INTERVAL_SEC="${SPIN_INTERVAL_SEC:-180}"
SPIN_PAUSE_SEC="${SPIN_PAUSE_SEC:-0.5}"
DURATION_SEC="${DURATION_SEC:-0}"
STATUS_PERIOD_SEC="${STATUS_PERIOD_SEC:-60}"
TRIAL_PREFIX="${TRIAL_PREFIX:-endurance_spin}"
LOG_ROOT="${LOG_ROOT:-$WORKSPACE/experiment_logs}"

cd "$WORKSPACE"

set +u
source src/mobile_robot/scripts/source_robot_local_v3.sh
set -u

SPIN_DIRECTION_NORMALIZED="${SPIN_DIRECTION,,}"
SPIN_MODE_NORMALIZED="${SPIN_MODE,,}"

direction_to_angular_z() {
  local direction="${1,,}"
  case "$direction" in
    left|ccw|counterclockwise)
      echo "${SPIN_SPEED#-}"
      ;;
    right|cw|clockwise)
      echo "-${SPIN_SPEED#-}"
      ;;
    signed|custom)
      echo "$SPIN_SPEED"
      ;;
    *)
      return 1
      ;;
  esac
}

flip_direction() {
  local direction="${1,,}"
  case "$direction" in
    left|ccw|counterclockwise)
      echo "right"
      ;;
    right|cw|clockwise)
      echo "left"
      ;;
    *)
      echo "ERROR: alternating spin supports left or right start direction only. Got: $direction" >&2
      return 1
      ;;
  esac
}

if ! ANGULAR_Z="$(direction_to_angular_z "$SPIN_DIRECTION_NORMALIZED")"; then
  echo "ERROR: SPIN_DIRECTION must be left, right, or signed. Got: $SPIN_DIRECTION" >&2
  exit 1
fi

case "$SPIN_MODE_NORMALIZED" in
  single|alternate)
    ;;
  *)
    echo "ERROR: SPIN_MODE must be single or alternate. Got: $SPIN_MODE" >&2
    exit 1
    ;;
esac

if [[ "$SPIN_MODE_NORMALIZED" == "alternate" ]]; then
  if ! [[ "$SPIN_INTERVAL_SEC" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: SPIN_INTERVAL_SEC must be a positive integer number of seconds. Got: $SPIN_INTERVAL_SEC" >&2
    exit 1
  fi
  if ! [[ "$DURATION_SEC" =~ ^[0-9]+$ ]]; then
    echo "ERROR: DURATION_SEC must be an integer number of seconds when SPIN_MODE=alternate. Got: $DURATION_SEC" >&2
    exit 1
  fi
  if ! flip_direction "$SPIN_DIRECTION_NORMALIZED" >/dev/null; then
    exit 1
  fi
fi

LOG_DIR="$LOG_ROOT/${TRIAL_PREFIX}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

STATUS_PID=""
CMD_PID=""

stop_robot() {
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
}

cleanup() {
  stop_robot
  if [[ -n "$STATUS_PID" ]]; then
    kill "$STATUS_PID" >/dev/null 2>&1 || true
    wait "$STATUS_PID" >/dev/null 2>&1 || true
  fi
  if [[ -n "$CMD_PID" ]]; then
    pkill -TERM -P "$CMD_PID" >/dev/null 2>&1 || true
    kill "$CMD_PID" >/dev/null 2>&1 || true
    wait "$CMD_PID" >/dev/null 2>&1 || true
  fi
  {
    echo
    echo "=== END $(date -Is) ==="
    echo "Stopped robot with zero /cmd_vel."
  } >>"$LOG_DIR/endurance_status.log"
  echo "Endurance log directory: $LOG_DIR"
}
trap cleanup EXIT INT TERM

echo "Starting hardware drivers..."
sudo systemctl start fastdds-discovery.service
sudo systemctl start robot-laser-tf.service
sudo systemctl start rplidar.service
sudo systemctl start yahboom-base.service

{
  echo "=== START $(date -Is) ==="
  echo "WORKSPACE=$WORKSPACE"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
  echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-}"
  echo "SPIN_SPEED=$SPIN_SPEED"
  echo "SPIN_MODE=$SPIN_MODE_NORMALIZED"
  echo "SPIN_DIRECTION=$SPIN_DIRECTION_NORMALIZED"
  echo "SPIN_INTERVAL_SEC=$SPIN_INTERVAL_SEC"
  echo "SPIN_PAUSE_SEC=$SPIN_PAUSE_SEC"
  echo "ANGULAR_Z=$ANGULAR_Z"
  echo "DURATION_SEC=$DURATION_SEC"
  echo "STATUS_PERIOD_SEC=$STATUS_PERIOD_SEC"
  echo
  echo "--- services ---"
  systemctl --no-pager -l status \
    fastdds-discovery.service \
    robot-laser-tf.service \
    rplidar.service \
    yahboom-base.service || true
  echo
  echo "--- topics ---"
  ros2 topic list | sort || true
} | tee "$LOG_DIR/endurance_startup.log"

echo "Checking /scan and /odom before motion..."
timeout 10 ros2 topic echo /scan --once --qos-reliability best_effort \
  >"$LOG_DIR/pre_spin_scan_once.txt" 2>&1 || {
  echo "ERROR: no /scan sample. See $LOG_DIR/pre_spin_scan_once.txt" >&2
  exit 1
}
timeout 10 ros2 topic echo /odom --once \
  >"$LOG_DIR/pre_spin_odom_once.txt" 2>&1 || {
  echo "ERROR: no /odom sample. See $LOG_DIR/pre_spin_odom_once.txt" >&2
  exit 1
}

(
  while true; do
    {
      echo
      echo "=== STATUS $(date -Is) ==="
      echo "--- services active ---"
      for service in \
        fastdds-discovery.service \
        robot-laser-tf.service \
        rplidar.service \
        yahboom-base.service; do
        printf "%s " "$service"
        systemctl is-active "$service" || true
      done
      echo "--- nodes ---"
      ros2 node list || true
      echo "--- topics ---"
      ros2 topic list | sort || true
      echo "--- /cmd_vel info ---"
      ros2 topic info /cmd_vel --verbose || true
      echo "--- /scan info ---"
      ros2 topic info /scan --verbose || true
      echo "--- /odom info ---"
      ros2 topic info /odom --verbose || true
    } >>"$LOG_DIR/endurance_status.log" 2>&1
    sleep "$STATUS_PERIOD_SEC"
  done
) &
STATUS_PID="$!"

echo "Publishing teleop-style ${SPIN_MODE_NORMALIZED} spin command. Press Ctrl+C to stop."
echo "Log directory: $LOG_DIR"

publish_spin_segment() {
  local direction="$1"
  local angular_z="$2"
  local duration_sec="$3"
  {
    echo
    echo "=== COMMAND $(date -Is) direction=$direction angular_z=$angular_z duration=${duration_sec}s ==="
  } >>"$LOG_DIR/cmd_vel_pub.log"
  timeout "$duration_sec" ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: ${angular_z}}}" >>"$LOG_DIR/cmd_vel_pub.log" 2>&1 || true
}

run_single_spin() {
  local cmd=(ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: ${ANGULAR_Z}}}")
  if [[ "$DURATION_SEC" == "0" ]]; then
    "${cmd[@]}" >"$LOG_DIR/cmd_vel_pub.log" 2>&1
  else
    timeout "$DURATION_SEC" "${cmd[@]}" >"$LOG_DIR/cmd_vel_pub.log" 2>&1 || true
  fi
}

run_alternating_spin() {
  local direction="$SPIN_DIRECTION_NORMALIZED"
  local elapsed=0
  local segment_duration="$SPIN_INTERVAL_SEC"
  local segment_angular_z=""

  while [[ "$DURATION_SEC" == "0" || "$elapsed" -lt "$DURATION_SEC" ]]; do
    if [[ "$DURATION_SEC" != "0" && $((elapsed + segment_duration)) -gt "$DURATION_SEC" ]]; then
      segment_duration=$((DURATION_SEC - elapsed))
    else
      segment_duration="$SPIN_INTERVAL_SEC"
    fi

    segment_angular_z="$(direction_to_angular_z "$direction")"
    publish_spin_segment "$direction" "$segment_angular_z" "$segment_duration"
    elapsed=$((elapsed + segment_duration))

    stop_robot
    sleep "$SPIN_PAUSE_SEC"
    direction="$(flip_direction "$direction")"
  done
}

if [[ "$SPIN_MODE_NORMALIZED" == "alternate" ]]; then
  run_alternating_spin &
else
  run_single_spin &
fi
CMD_PID="$!"
wait "$CMD_PID" || true
