#!/usr/bin/env bash
# Run one real-robot V2 trial with compact terminal output:
# AMCL global localization autorotate -> SAC motion -> evidence/logs.

set -uo pipefail

WORKSPACE=${WORKSPACE:-$HOME/ws_mobile_v3}
LOG_ROOT=${LOG_ROOT:-$WORKSPACE/experiment_logs}
MAP_DIR=${MAP_DIR:-$HOME/maps}
MODEL=${MODEL:-$WORKSPACE/rl_models/sac_goal_policy.zip}

GOAL_X=${GOAL_X:-0.0}
GOAL_Y=${GOAL_Y:-0.0}
GOAL_YAW=${GOAL_YAW:-0.0}
SUCCESS_RADIUS=${SUCCESS_RADIUS:-0.20}
COLLISION_RANGE_THRESHOLD=${COLLISION_RANGE_THRESHOLD:-0.20}
CONTROL_SOURCE=${CONTROL_SOURCE:-sac_autonomy}

USE_SIM_TIME=${USE_SIM_TIME:-false}
RVIZ=${RVIZ:-false}
AUTO_GLOBAL_LOCALIZATION=${AUTO_GLOBAL_LOCALIZATION:-true}
AUTO_ROTATE=${AUTO_ROTATE:-true}
ROTATE_SPEED=${ROTATE_SPEED:-0.20}
ROTATE_DURATION=${ROTATE_DURATION:-31.5}
LOCALIZATION_START_DELAY=${LOCALIZATION_START_DELAY:-3.0}
LOCALIZATION_SERVICE_WAIT_TIMEOUT=${LOCALIZATION_SERVICE_WAIT_TIMEOUT:-120.0}
POST_ROTATE_SETTLE_SEC=${POST_ROTATE_SETTLE_SEC:-2.0}
EVIDENCE_DELAY_SEC=${EVIDENCE_DELAY_SEC:-12}
RUN_GOAL_RELATIVE=${RUN_GOAL_RELATIVE:-true}
TRIAL_DURATION_SEC=${TRIAL_DURATION_SEC:-0}
TRIAL_PREFIX=${TRIAL_PREFIX:-real_sac_trial}
REQUIRE_AMCL_READY=${REQUIRE_AMCL_READY:-true}
AMCL_READY_TIMEOUT=${AMCL_READY_TIMEOUT:-30}
AMCL_COVARIANCE_XY_GATE=${AMCL_COVARIANCE_XY_GATE:-0.60}
AMCL_COVARIANCE_YAW_GATE=${AMCL_COVARIANCE_YAW_GATE:-0.50}
AMCL_GLOBAL_RETRIES=${AMCL_GLOBAL_RETRIES:-2}
AMCL_RETRY_ROTATE_DURATION=${AMCL_RETRY_ROTATE_DURATION:-}
AMCL_RETRY_SETTLE_SEC=${AMCL_RETRY_SETTLE_SEC:-$POST_ROTATE_SETTLE_SEC}
AMCL_STARTUP_EXTRA_SEC=${AMCL_STARTUP_EXTRA_SEC:-15.0}
AMCL_POSE_REQUIRED=${AMCL_POSE_REQUIRED:-true}
AMCL_TF_STABLE_SAMPLES=${AMCL_TF_STABLE_SAMPLES:-3}
AMCL_PARAMS_FILE=${AMCL_PARAMS_FILE:-$WORKSPACE/src/mobile_robot/config/amcl_real_robot_params.yaml}
AMCL_POST_LAUNCH_NOMOTION_SEC=${AMCL_POST_LAUNCH_NOMOTION_SEC:-25.0}
AMCL_POST_LAUNCH_NOMOTION_INTERVAL_SEC=${AMCL_POST_LAUNCH_NOMOTION_INTERVAL_SEC:-1.0}

if [[ "$(hostname -s 2>/dev/null || hostname)" == "JTS" ]]; then
  export ROS_DOMAIN_ID="${ROBOT_ROS_DOMAIN_ID:-10}"
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_SUPER_CLIENT="${ROS_SUPER_CLIENT:-TRUE}"
if [[ -d "$WORKSPACE/.venv" ]]; then
  export VIRTUAL_ENV="$WORKSPACE/.venv"
  export PATH="$VIRTUAL_ENV/bin:$PATH"
  venv_site_packages=$("$VIRTUAL_ENV/bin/python" -c 'import site; print(site.getsitepackages()[0])' 2>/dev/null || true)
  if [[ -n "$venv_site_packages" ]]; then
    export PYTHONPATH="$venv_site_packages${PYTHONPATH:+:$PYTHONPATH}"
  fi
fi
if [[ -z "${ROS_DISCOVERY_SERVER:-}" ]]; then
  if [[ "$(hostname -s 2>/dev/null || hostname)" == "JTS" ]]; then
    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
  else
    robot_ip=$(getent hosts JTS.local 2>/dev/null | awk '{print $1; exit}' || true)
    if [[ -n "$robot_ip" ]]; then
      export ROS_DISCOVERY_SERVER="${robot_ip}:11811"
    fi
  fi
fi

usage() {
  cat <<EOF
Usage:
  $0 [TRIAL_NUM] [options]

Options:
  --trial NUM                  Trial number or label. Same as positional TRIAL_NUM.
  --model PATH                 SAC .zip model checkpoint.
  --map PATH                   Saved map YAML file.
  --goal-x VALUE               Goal x in map frame. Default: $GOAL_X
  --goal-y VALUE               Goal y in map frame. Default: $GOAL_Y
  --goal-yaw VALUE             Goal yaw in map frame. Default: $GOAL_YAW
  --auto-global-localization true
                               Required for measured trials. Calls AMCL global localization before autorotate.
  --auto-rotate true           Required for measured trials. Autorotate once before SAC starts.
  --no-auto-rotate             Unsupported for measured trials; kept only to fail old commands clearly.
  --rotate-speed VALUE         Autorotate angular speed. Default: $ROTATE_SPEED
  --rotate-duration VALUE      Autorotate duration. Default: $ROTATE_DURATION
  --localization-start-delay V Delay before global AMCL helper starts. Default: $LOCALIZATION_START_DELAY
  --localization-service-wait-timeout V
                               Max wait for AMCL global-localization service. Default: $LOCALIZATION_SERVICE_WAIT_TIMEOUT
  --trial-duration VALUE       Auto-end trial after this many seconds after motion starts. 0 means manual Ctrl+C.
  --rviz true|false            Start RViz with localization launch. Default: $RVIZ
  --require-amcl-ready true    Required for measured trials. Checks /amcl_pose and map->base_footprint before motion.
  --amcl-ready-timeout SEC      Timeout for AMCL readiness checks. Default: $AMCL_READY_TIMEOUT
  --amcl-covariance-xy-gate V   Max AMCL covariance x/y before motion. Default: $AMCL_COVARIANCE_XY_GATE
  --amcl-covariance-yaw-gate V  Max AMCL yaw covariance before motion. Default: $AMCL_COVARIANCE_YAW_GATE
  --amcl-global-retries N       Extra global-localization + autorotate attempts before aborting. Default: $AMCL_GLOBAL_RETRIES
  --amcl-retry-rotate-duration V
                               Autorotate duration for retry attempts. Default: main rotate duration
  --amcl-retry-settle-sec V     Settle delay after retry autorotate. Default: $AMCL_RETRY_SETTLE_SEC
  --amcl-startup-extra-sec V     Extra settle time after first autorotate. Default: $AMCL_STARTUP_EXTRA_SEC
  --amcl-pose-required true|false
                               Require /amcl_pose sample. Default: $AMCL_POSE_REQUIRED
  --amcl-params-file PATH        AMCL parameter YAML. Default: $AMCL_PARAMS_FILE
  --amcl-post-launch-nomotion-sec V
                               Call /request_nomotion_update for this many seconds after SAC launch. Default: $AMCL_POST_LAUNCH_NOMOTION_SEC
  --evidence-delay VALUE       Delay after motion launch before ROS graph snapshot. Default: $EVIDENCE_DELAY_SEC
  --trial-prefix TEXT          Prefix for output trial folders. Default: $TRIAL_PREFIX
  -h, --help                   Show this help.

Examples:
  $0 01 --model /path/model.zip --map /path/map.yaml --auto-rotate true
  $0 02 --goal-x 0.6 --goal-y -0.4 --trial-duration 120

This measured-trial wrapper does not support RViz 2D Pose Estimate or manual
/initialpose correction. AMCL must converge from global localization plus
autorotate and pass the readiness gate before SAC motion starts.
EOF
}

need_value() {
  local option=$1
  local value=${2:-}
  if [ -z "$value" ]; then
    echo "ERROR: $option requires a value."
    exit 2
  fi
}

normalize_bool_arg() {
  local option=$1
  local value=$2
  value=$(echo "$value" | tr '[:upper:]' '[:lower:]')
  case "$value" in
    true|false)
      echo "$value"
      ;;
    *)
      echo "ERROR: $option must be true or false." >&2
      exit 2
      ;;
  esac
}

positive_number() {
  awk -v value="$1" 'BEGIN { exit !(value + 0 > 0) }'
}

TRIAL_NUM=${TRIAL_NUM:-}
while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --trial|--trial-num)
      need_value "$1" "${2:-}"
      TRIAL_NUM=$2
      shift 2
      ;;
    --trial-prefix)
      need_value "$1" "${2:-}"
      TRIAL_PREFIX=$2
      shift 2
      ;;
    --model|--sac-model)
      need_value "$1" "${2:-}"
      MODEL=$2
      shift 2
      ;;
    --map|--map-yaml)
      need_value "$1" "${2:-}"
      MAP_YAML=$2
      shift 2
      ;;
    --goal-x)
      need_value "$1" "${2:-}"
      GOAL_X=$2
      shift 2
      ;;
    --goal-y)
      need_value "$1" "${2:-}"
      GOAL_Y=$2
      shift 2
      ;;
    --goal-yaw|--goal-theta)
      need_value "$1" "${2:-}"
      GOAL_YAW=$2
      shift 2
      ;;
    --auto-rotate|--autorotate)
      need_value "$1" "${2:-}"
      AUTO_ROTATE=$(normalize_bool_arg "$1" "$2") || exit 2
      shift 2
      ;;
    --no-auto-rotate|--no-autorotate)
      AUTO_ROTATE=false
      shift
      ;;
    --rotate-speed)
      need_value "$1" "${2:-}"
      ROTATE_SPEED=$2
      shift 2
      ;;
    --rotate-duration)
      need_value "$1" "${2:-}"
      ROTATE_DURATION=$2
      shift 2
      ;;
    --localization-start-delay)
      need_value "$1" "${2:-}"
      LOCALIZATION_START_DELAY=$2
      shift 2
      ;;
    --localization-service-wait-timeout)
      need_value "$1" "${2:-}"
      LOCALIZATION_SERVICE_WAIT_TIMEOUT=$2
      shift 2
      ;;
    --trial-duration)
      need_value "$1" "${2:-}"
      TRIAL_DURATION_SEC=$2
      shift 2
      ;;
    --rviz)
      need_value "$1" "${2:-}"
      RVIZ=$(normalize_bool_arg "$1" "$2") || exit 2
      shift 2
      ;;
    --require-amcl-ready)
      need_value "$1" "${2:-}"
      REQUIRE_AMCL_READY=$(normalize_bool_arg "$1" "$2") || exit 2
      shift 2
      ;;
    --manual-amcl-confirm)
      need_value "$1" "${2:-}"
      manual_value=$(normalize_bool_arg "$1" "$2") || exit 2
      if [ "$manual_value" = "true" ]; then
        echo "ERROR: --manual-amcl-confirm true is not supported for measured trials."
        echo "Use global AMCL localization with autorotate and the AMCL readiness gate."
        exit 2
      fi
      shift 2
      ;;
    --amcl-ready-timeout)
      need_value "$1" "${2:-}"
      AMCL_READY_TIMEOUT=$2
      shift 2
      ;;
    --amcl-covariance-xy-gate)
      need_value "$1" "${2:-}"
      AMCL_COVARIANCE_XY_GATE=$2
      shift 2
      ;;
    --amcl-covariance-yaw-gate)
      need_value "$1" "${2:-}"
      AMCL_COVARIANCE_YAW_GATE=$2
      shift 2
      ;;
    --amcl-global-retries)
      need_value "$1" "${2:-}"
      AMCL_GLOBAL_RETRIES=$2
      shift 2
      ;;
    --amcl-retry-rotate-duration)
      need_value "$1" "${2:-}"
      AMCL_RETRY_ROTATE_DURATION=$2
      shift 2
      ;;
    --amcl-retry-settle-sec)
      need_value "$1" "${2:-}"
      AMCL_RETRY_SETTLE_SEC=$2
      shift 2
      ;;
    --amcl-startup-extra-sec)
      need_value "$1" "${2:-}"
      AMCL_STARTUP_EXTRA_SEC=$2
      shift 2
      ;;
    --amcl-pose-required)
      need_value "$1" "${2:-}"
      AMCL_POSE_REQUIRED=$2
      shift 2
      ;;
    --amcl-params-file)
      need_value "$1" "${2:-}"
      AMCL_PARAMS_FILE=$2
      shift 2
      ;;
    --amcl-post-launch-nomotion-sec)
      need_value "$1" "${2:-}"
      AMCL_POST_LAUNCH_NOMOTION_SEC=$2
      shift 2
      ;;
    --evidence-delay)
      need_value "$1" "${2:-}"
      EVIDENCE_DELAY_SEC=$2
      shift 2
      ;;
    --auto-global-localization)
      need_value "$1" "${2:-}"
      AUTO_GLOBAL_LOCALIZATION=$(normalize_bool_arg "$1" "$2") || exit 2
      shift 2
      ;;
    --run-goal-relative)
      need_value "$1" "${2:-}"
      RUN_GOAL_RELATIVE=$(normalize_bool_arg "$1" "$2") || exit 2
      shift 2
      ;;
    --)
      shift
      break
      ;;
    -*)
      echo "ERROR: unknown option: $1"
      usage
      exit 2
      ;;
    *)
      if [ -z "$TRIAL_NUM" ]; then
        TRIAL_NUM=$1
        shift
      else
        echo "ERROR: unexpected argument: $1"
        usage
        exit 2
      fi
      ;;
  esac
done

AUTO_ROTATE=$(normalize_bool_arg "--auto-rotate" "$AUTO_ROTATE") || exit 2
AUTO_GLOBAL_LOCALIZATION=$(normalize_bool_arg "--auto-global-localization" "$AUTO_GLOBAL_LOCALIZATION") || exit 2
RVIZ=$(normalize_bool_arg "--rviz" "$RVIZ") || exit 2
RUN_GOAL_RELATIVE=$(normalize_bool_arg "--run-goal-relative" "$RUN_GOAL_RELATIVE") || exit 2
REQUIRE_AMCL_READY=$(normalize_bool_arg "--require-amcl-ready" "$REQUIRE_AMCL_READY") || exit 2
AMCL_POSE_REQUIRED=$(normalize_bool_arg "--amcl-pose-required" "$AMCL_POSE_REQUIRED") || exit 2

if [ -z "$AMCL_RETRY_ROTATE_DURATION" ]; then
  AMCL_RETRY_ROTATE_DURATION=$ROTATE_DURATION
fi

if [ "$AUTO_GLOBAL_LOCALIZATION" != "true" ] || [ "$AUTO_ROTATE" != "true" ] || [ "$REQUIRE_AMCL_READY" != "true" ]; then
  echo "ERROR: measured trials require --auto-global-localization true, --auto-rotate true, and --require-amcl-ready true."
  echo "This project uses global AMCL localization with startup autorotate, not manual 2D Pose Estimate."
  exit 2
fi

if [ -z "$TRIAL_NUM" ]; then
  TRIAL_NUM=$(date +%Y%m%d_%H%M%S)
fi

if [[ "$TRIAL_NUM" =~ ^[0-9]+$ ]]; then
  TRIAL_LABEL=$(printf "%02d" "$TRIAL_NUM")
else
  TRIAL_LABEL="$TRIAL_NUM"
fi

TRIAL_NAME=${TRIAL_NAME:-${TRIAL_PREFIX}_${TRIAL_LABEL}_$(date +%Y%m%d_%H%M%S)}
TRIAL_DIR=$LOG_ROOT/$TRIAL_NAME
LATEST_TRIAL_FILE=$LOG_ROOT/latest_trial_dir.txt
LATEST_MAP_FILE=$LOG_ROOT/latest_map_yaml.txt
WORKSPACE_LATEST_MAP_FILE=$WORKSPACE/latest_map_yaml.txt

LOGGER_PID=""
LOCALIZATION_PID=""
BRIDGE_PID=""
EVIDENCE_PID=""
TRIAL_TIMER_PID=""
GOAL_WATCH_PID=""
AMCL_NOMOTION_PID=""
STATUS_MONITOR_PID=""
FINISHED=0

source_ros() {
  local restore_nounset=0
  case "$-" in
    *u*)
      restore_nounset=1
      set +u
      ;;
  esac

  source /opt/ros/humble/setup.bash
  if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
  fi

  if [ "$restore_nounset" -eq 1 ]; then
    set -u
  fi
}

select_map() {
  if [ -n "${MAP_YAML:-}" ]; then
    echo "$MAP_YAML"
    return
  fi
  if [ -f "$WORKSPACE_LATEST_MAP_FILE" ]; then
    cat "$WORKSPACE_LATEST_MAP_FILE"
    return
  fi
  if [ -f "$LATEST_MAP_FILE" ]; then
    cat "$LATEST_MAP_FILE"
    return
  fi
  ls -t "$MAP_DIR"/real_robot_sac_map_*.yaml "$MAP_DIR"/real_robot_map_*.yaml 2>/dev/null | head -1
}

stop_process() {
  local pid=$1
  local label=$2
  local timeout_loops=${3:-40}
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    echo "Stopping $label..."
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 "$timeout_loops"); do
      if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return
      fi
      sleep 0.2
    done
    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return
      fi
      sleep 0.2
    done
    kill -KILL "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

publish_zero_velocity() {
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
}

stop_stale_trial_processes() {
  local patterns=(
    "/nav2_map_server/map_server"
    "/nav2_amcl/amcl"
    "lifecycle_manager_sac_localization"
    "lifecycle_manager_sac_route"
    "/nav2_planner/planner_server"
    "/mobile_robot/global_amcl_autorotate_node"
    "/mobile_robot/goal_relative_calculator"
    "/mobile_robot/action_logger_node"
    "/mobile_robot/center_route_requester_node"
    "/mobile_robot/local_target_node"
    "/mobile_robot/sac_safety_filter_node"
    "/mobile_robot/sac_motion_node"
    "/mobile_robot/sac_validation_logger"
  )
  local pids=()
  local pattern pid

  for pattern in "${patterns[@]}"; do
    while IFS= read -r pid; do
      [ -n "$pid" ] || continue
      [ "$pid" != "$$" ] || continue
      pids+=("$pid")
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done

  if [ "${#pids[@]}" -eq 0 ]; then
    return
  fi

  echo "Stopping stale AMCL/SAC trial processes: ${pids[*]}"
  publish_zero_velocity
  kill -INT "${pids[@]}" 2>/dev/null || true
  sleep 2
  kill -TERM "${pids[@]}" 2>/dev/null || true
  sleep 1
  publish_zero_velocity
}

start_status_monitor() {
  TRIAL_DIR="$TRIAL_DIR" python3 - <<'PY' &
import math
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, Float32


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def fmt(value, digits=3):
    if value is None or not math.isfinite(float(value)):
        return "---"
    return f"{float(value):.{digits}f}"


class CompactTrialMonitor(Node):
    def __init__(self):
        super().__init__("compact_sac_trial_monitor")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.goal_distance = None
        self.goal_angle = None
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.amcl_pose = None
        self.reached = False
        self.action_count = 0
        self.goal_flag = Path(os.environ["TRIAL_DIR"]) / "goal_reached.flag"
        self.zero_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Float32, "/goal_distance", self._goal_distance_cb, qos)
        self.create_subscription(Float32, "/goal_angle", self._goal_angle_cb, qos)
        self.create_subscription(Bool, "/goal_reached", self._goal_reached_cb, qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, qos)
        print("AMCL global localization ready. SAC motion monitor started.", flush=True)

    def _goal_distance_cb(self, msg):
        self.goal_distance = float(msg.data)

    def _goal_angle_cb(self, msg):
        self.goal_angle = float(msg.data)

    def _goal_reached_cb(self, msg):
        if bool(msg.data) and not self.reached:
            self.reached = True
            self._publish_zero()
            self.goal_flag.write_text("goal_reached\n", encoding="utf-8")
            print("Reached the goal. Published zero velocity.", flush=True)

    def _amcl_pose_cb(self, msg):
        pose = msg.pose.pose
        self.amcl_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    def _cmd_cb(self, msg):
        self.cmd_linear = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)
        if self.reached:
            return
        self.action_count += 1
        self._print_status()

    def _publish_zero(self):
        zero = Twist()
        for _ in range(3):
            self.zero_pub.publish(zero)

    def _print_status(self):
        if self.reached:
            return
        if self.amcl_pose is None:
            pose_text = "(---, ---, ---)"
        else:
            pose_text = (
                f"({fmt(self.amcl_pose[0], 2)}, "
                f"{fmt(self.amcl_pose[1], 2)}, "
                f"{fmt(self.amcl_pose[2], 2)})"
            )
        print(
            f"SAC action={self.action_count} "
            f"goal_dist={fmt(self.goal_distance)}m "
            f"goal_angle={fmt(self.goal_angle)}rad "
            f"amcl_pose={pose_text} "
            f"cmd_vel=({fmt(self.cmd_linear, 2)}, {fmt(self.cmd_angular, 2)})",
            flush=True,
        )


rclpy.init()
node = CompactTrialMonitor()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    if rclpy.ok():
        node._publish_zero()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
  STATUS_MONITOR_PID=$!
}

start_goal_watch() {
  (
    while kill -0 "$BRIDGE_PID" 2>/dev/null; do
      if [ -f "$TRIAL_DIR/goal_reached.flag" ]; then
        publish_zero_velocity
        kill -INT "$BRIDGE_PID" 2>/dev/null || true
        break
      fi
      sleep 0.2
    done
  ) &
  GOAL_WATCH_PID=$!
}

wait_for_amcl_ready() {
  local timeout_sec=${1:-$AMCL_READY_TIMEOUT}
  local label=${2:-pre_motion}
  local amcl_log="$TRIAL_DIR/${label}_amcl_pose_once.txt"
  local tf_log="$TRIAL_DIR/${label}_tf_map_to_base_footprint.txt"
  local particle_log="$TRIAL_DIR/${label}_particle_cloud_once.txt"

  echo "Checking AMCL readiness before SAC motion..."
  if ! timeout "$timeout_sec" ros2 topic echo /scan sensor_msgs/msg/LaserScan \
      --qos-reliability best_effort --qos-durability volatile --once >/dev/null 2>&1; then
    echo "ERROR: no /scan sample before motion."
    return 1
  fi
  if ! timeout "$timeout_sec" ros2 topic echo /odom nav_msgs/msg/Odometry \
      --qos-reliability best_effort --qos-durability volatile --once >/dev/null 2>&1; then
    echo "ERROR: no /odom sample before motion."
    return 1
  fi

  rm -f "$particle_log"
  particle_type="nav""2_msgs/msg/ParticleCloud"
  timeout "$timeout_sec" ros2 topic echo /particle_cloud "$particle_type" \
    --qos-reliability best_effort --qos-durability volatile --once >"$particle_log" 2>&1 || true

  rm -f "$amcl_log"
  local got_amcl_pose=0
  if timeout "$timeout_sec" bash -lc '
      output=$1
      ros2 topic echo /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped \
        --once >"$output" 2>&1 &
      echo_pid=$!
      while kill -0 "$echo_pid" 2>/dev/null; do
        timeout 1 ros2 service call /request_nomotion_update std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true
        sleep 0.5
      done
      wait "$echo_pid"
    ' _ "$amcl_log"; then
    got_amcl_pose=1
  elif [ "$AMCL_POSE_REQUIRED" = "true" ]; then
    echo "ERROR: no /amcl_pose sample before motion. Log: $amcl_log"
    return 1
  else
    echo "WARNING: no /amcl_pose sample before motion; using AMCL TF readiness. Log: $amcl_log"
  fi

  if [ "$got_amcl_pose" -eq 1 ] && ! AMCL_LOG="$amcl_log" \
      AMCL_COVARIANCE_XY_GATE="$AMCL_COVARIANCE_XY_GATE" \
      AMCL_COVARIANCE_YAW_GATE="$AMCL_COVARIANCE_YAW_GATE" \
      python3 - <<'PY'
import os
import sys
import yaml

path = os.environ["AMCL_LOG"]
xy_gate = float(os.environ["AMCL_COVARIANCE_XY_GATE"])
yaw_gate = float(os.environ["AMCL_COVARIANCE_YAW_GATE"])
with open(path, "r", encoding="utf-8") as f:
    data = next(doc for doc in yaml.safe_load_all(f) if isinstance(doc, dict))
cov = data["pose"]["covariance"]
cov_x = float(cov[0])
cov_y = float(cov[7])
cov_yaw = float(cov[35])
print(f"AMCL covariance: x={cov_x:.6f} y={cov_y:.6f} yaw={cov_yaw:.6f}")
if cov_x > xy_gate or cov_y > xy_gate or cov_yaw > yaw_gate:
    print(
        "AMCL covariance gate failed: "
        f"x/y <= {xy_gate:.3f}, yaw <= {yaw_gate:.3f} required",
        file=sys.stderr,
    )
    sys.exit(1)
PY
  then
    echo "ERROR: AMCL covariance is too high before motion. Log: $amcl_log"
    return 1
  fi

  if ! AMCL_TF_STABLE_SAMPLES="$AMCL_TF_STABLE_SAMPLES" timeout "$timeout_sec" \
      python3 - >"$tf_log" 2>&1 <<'PY'
import os
import sys
import time

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

required = int(os.environ.get("AMCL_TF_STABLE_SAMPLES", "3"))
count = 0

rclpy.init()
node = rclpy.create_node("amcl_tf_ready_check")
buffer = Buffer()
listener = TransformListener(buffer, node)

try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            transform = buffer.lookup_transform("map", "base_footprint", Time())
        except Exception as exc:
            print(f"Waiting for TF map -> base_footprint: {exc}", flush=True)
            time.sleep(0.2)
            continue

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        count += 1
        print(
            f"TF sample {count}/{required}: "
            f"x={translation.x:.3f} y={translation.y:.3f} "
            f"z={translation.z:.3f} qz={rotation.z:.3f} qw={rotation.w:.3f}",
            flush=True,
        )
        if count >= required:
            sys.exit(0)
        time.sleep(0.2)
finally:
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

sys.exit(1)
PY
  then
    echo "ERROR: no valid TF map -> base_footprint before motion. Log: $tf_log"
    return 1
  fi

  echo "AMCL readiness OK."
}

run_global_localization_autorotate_retry() {
  local attempt=$1
  local rotate_duration=$AMCL_RETRY_ROTATE_DURATION

  echo "Retrying AMCL global localization attempt ${attempt} with autorotate..."
  if ! timeout 20 ros2 service call /reinitialize_global_localization std_srvs/srv/Empty '{}' \
      >"$TRIAL_DIR/amcl_global_retry_${attempt}.txt" 2>&1; then
    echo "ERROR: /reinitialize_global_localization failed on retry ${attempt}."
    return 1
  fi

  if [ "$AUTO_ROTATE" = "true" ]; then
    echo "AMCL retry ${attempt}: autorotate speed=$ROTATE_SPEED duration=${rotate_duration}s"
    ROTATE_SPEED_ENV="$ROTATE_SPEED" timeout "$rotate_duration" bash -lc '
      while true; do
        ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
          "{linear: {x: 0.0}, angular: {z: ${ROTATE_SPEED_ENV}}}" >/dev/null 2>&1 || true
        sleep 0.1
      done
    ' || true
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  fi

  sleep "$AMCL_RETRY_SETTLE_SEC"
}

wait_for_amcl_ready_with_retries() {
  local attempt=0
  local max_retries=$AMCL_GLOBAL_RETRIES

  while true; do
    if wait_for_amcl_ready "$AMCL_READY_TIMEOUT" "pre_motion_attempt_${attempt}"; then
      return 0
    fi

    if [ "$AUTO_GLOBAL_LOCALIZATION" != "true" ] || [ "$AUTO_ROTATE" != "true" ]; then
      echo "ERROR: AMCL readiness failed and automatic global-localization autorotate retry is disabled."
      return 1
    fi

    if [ "$attempt" -ge "$max_retries" ]; then
      echo "ERROR: AMCL readiness failed after $((max_retries + 1)) global-localization attempts."
      return 1
    fi

    attempt=$((attempt + 1))
    run_global_localization_autorotate_retry "$attempt" || return 1
  done
}

run_analysis() {
  if [ ! -f "$TRIAL_DIR/robot_action_log.csv" ]; then
    echo "No robot_action_log.csv found, skipping analysis: $TRIAL_DIR"
    return
  fi

  echo "Saving plots and validation summary..."
  python3 "$WORKSPACE/src/mobile_robot/mobile_robot/analyze_experiment_log.py" \
    --log "$TRIAL_DIR/robot_action_log.csv" \
    --out "$TRIAL_DIR/plots" \
    --summary-dir "$TRIAL_DIR" \
    --goal-x "$GOAL_X" \
    --goal-y "$GOAL_Y" \
    --success-radius "$SUCCESS_RADIUS" \
    --collision-threshold "$COLLISION_RANGE_THRESHOLD" \
    --expected-control-source "$CONTROL_SOURCE" \
    --route-system-expected >"$TRIAL_DIR/analysis_output.log" 2>&1
}

finish() {
  local status=${1:-0}
  if [ "$FINISHED" -eq 1 ]; then
    exit "$status"
  fi
  FINISHED=1

  stop_process "$TRIAL_TIMER_PID" "trial timer"
  stop_process "$GOAL_WATCH_PID" "goal watcher"
  stop_process "$AMCL_NOMOTION_PID" "AMCL no-motion keepalive"
  stop_process "$STATUS_MONITOR_PID" "SAC status monitor"
  stop_process "$BRIDGE_PID" "SAC motion"
  stop_process "$LOCALIZATION_PID" "AMCL localization"
  stop_process "$LOGGER_PID" "action logger"
  stop_stale_trial_processes

  publish_zero_velocity

  if [ -n "$EVIDENCE_PID" ]; then
    wait "$EVIDENCE_PID" 2>/dev/null || true
  fi

  run_analysis

  echo
  echo "Trial complete."
  echo "Reached-goal and motion details were saved in the latest trial folder."
  echo "Latest trial pointer: $LATEST_TRIAL_FILE"
  exit "$status"
}

trap 'finish 130' INT TERM

mkdir -p "$TRIAL_DIR"
echo "$TRIAL_DIR" > "$LATEST_TRIAL_FILE"

source_ros

MAP_YAML=$(select_map)
if [ -z "$MAP_YAML" ] || [ ! -f "$MAP_YAML" ]; then
  echo "ERROR: map YAML not found. Use --map /path/to/map.yaml or save a map first."
  exit 1
fi

if [ ! -f "$MODEL" ]; then
  echo "ERROR: SAC model not found: $MODEL"
  exit 1
fi
if [ ! -f "$AMCL_PARAMS_FILE" ]; then
  echo "ERROR: AMCL params file not found: $AMCL_PARAMS_FILE"
  exit 1
fi

echo "$MAP_YAML" > "$LATEST_MAP_FILE"
echo "$MAP_YAML" > "$WORKSPACE_LATEST_MAP_FILE"

echo "AMCL global localization + SAC trial"
echo "Goal: ($GOAL_X, $GOAL_Y, yaw=$GOAL_YAW)"
echo "Autorotate: $AUTO_ROTATE speed=$ROTATE_SPEED duration=$ROTATE_DURATION"
echo "AMCL readiness: timeout=${AMCL_READY_TIMEOUT}s retries=$AMCL_GLOBAL_RETRIES"
echo "AMCL pose required: $AMCL_POSE_REQUIRED"
echo "Detailed logs will be saved in the latest trial folder."
echo

stop_stale_trial_processes

ros2 run mobile_robot action_logger_node --ros-args \
  -p log_path:="$TRIAL_DIR/robot_action_log.csv" \
  -p control_source:="$CONTROL_SOURCE" \
  -p goal_x:="$GOAL_X" \
  -p goal_y:="$GOAL_Y" \
  -p success_radius:="$SUCCESS_RADIUS" \
  -p collision_range_threshold:="$COLLISION_RANGE_THRESHOLD" \
  >"$TRIAL_DIR/action_logger_output.log" 2>&1 &
LOGGER_PID=$!

sleep 2

echo "Starting AMCL global localization..."
ros2 launch mobile_robot sac_amcl_localization.launch.py \
  map:="$MAP_YAML" \
  use_sim_time:="$USE_SIM_TIME" \
  rviz:="$RVIZ" \
  auto_global_localization:="$AUTO_GLOBAL_LOCALIZATION" \
  auto_rotate:="$AUTO_ROTATE" \
  rotate_speed:="$ROTATE_SPEED" \
  rotate_duration:="$ROTATE_DURATION" \
  localization_start_delay:="$LOCALIZATION_START_DELAY" \
  localization_service_wait_timeout:="$LOCALIZATION_SERVICE_WAIT_TIMEOUT" \
  publish_robot_model:=false \
  amcl_params_file:="$AMCL_PARAMS_FILE" \
  >"$TRIAL_DIR/amcl_localization_output.log" 2>&1 &
LOCALIZATION_PID=$!

if [ "$AUTO_ROTATE" = "true" ]; then
  AMCL_WAIT_SEC=$(awk -v a="$LOCALIZATION_START_DELAY" -v s="$LOCALIZATION_SERVICE_WAIT_TIMEOUT" -v b="$ROTATE_DURATION" -v c="$POST_ROTATE_SETTLE_SEC" -v d="$AMCL_STARTUP_EXTRA_SEC" 'BEGIN { printf "%.1f", a + s + b + c + d }')
else
  AMCL_WAIT_SEC=$(awk -v a="$LOCALIZATION_START_DELAY" -v s="$LOCALIZATION_SERVICE_WAIT_TIMEOUT" -v c="$POST_ROTATE_SETTLE_SEC" -v d="$AMCL_STARTUP_EXTRA_SEC" 'BEGIN { printf "%.1f", a + s + c + d }')
fi

echo "Waiting ${AMCL_WAIT_SEC}s for AMCL startup and autorotate..."
sleep "$AMCL_WAIT_SEC"

if ! kill -0 "$LOCALIZATION_PID" 2>/dev/null; then
  echo "ERROR: AMCL localization exited before SAC could start."
  finish 1
fi

if [ "$REQUIRE_AMCL_READY" = "true" ]; then
  wait_for_amcl_ready_with_retries || finish 1
fi

echo "Starting SAC motion..."
ros2 launch mobile_robot sac_center_navigation.launch.py \
  use_gazebo:=false \
  use_amcl:=false \
  use_sim_time:="$USE_SIM_TIME" \
  gui:=false \
  rviz:=false \
  run_motion:=true \
  run_logger:=true \
  run_goal_relative:="$RUN_GOAL_RELATIVE" \
  map:="$MAP_YAML" \
  model:="$MODEL" \
  goal_x:="$GOAL_X" \
  goal_y:="$GOAL_Y" \
  goal_yaw:="$GOAL_YAW" \
  validation_csv:="$TRIAL_DIR/sac_motion_validation.csv" \
  >"$TRIAL_DIR/sac_motion_output.log" 2>&1 &
BRIDGE_PID=$!

start_status_monitor
start_goal_watch

if positive_number "$AMCL_POST_LAUNCH_NOMOTION_SEC"; then
  (
    end_time=$(awk -v now="$(date +%s)" -v duration="$AMCL_POST_LAUNCH_NOMOTION_SEC" 'BEGIN { printf "%.3f", now + duration }')
    while awk -v now="$(date +%s)" -v end="$end_time" 'BEGIN { exit !(now < end) }'; do
      timeout 3 ros2 service call /request_nomotion_update std_srvs/srv/Empty '{}' \
        >>"$TRIAL_DIR/post_launch_nomotion_update.log" 2>&1 || true
      sleep "$AMCL_POST_LAUNCH_NOMOTION_INTERVAL_SEC"
    done
  ) &
  AMCL_NOMOTION_PID=$!
fi

(
  sleep "$EVIDENCE_DELAY_SEC"
  EVIDENCE_SCRIPT="$LOG_ROOT/capture_ros_evidence.sh"
  if [ ! -f "$EVIDENCE_SCRIPT" ]; then
    EVIDENCE_SCRIPT="$WORKSPACE/experiment_logs/capture_ros_evidence.sh"
  fi

  if [ -f "$EVIDENCE_SCRIPT" ]; then
    bash "$EVIDENCE_SCRIPT"
  else
    echo "WARNING: missing capture_ros_evidence.sh"
  fi

  if [ -f "$LOG_ROOT/ros_graph_snapshot.txt" ]; then
    cp "$LOG_ROOT/ros_graph_snapshot.txt" "$TRIAL_DIR/ros_graph_snapshot.txt"
  elif [ -f "$WORKSPACE/experiment_logs/ros_graph_snapshot.txt" ]; then
    cp "$WORKSPACE/experiment_logs/ros_graph_snapshot.txt" "$TRIAL_DIR/ros_graph_snapshot.txt"
  fi
) &
EVIDENCE_PID=$!

if positive_number "$TRIAL_DURATION_SEC"; then
  (
    sleep "$TRIAL_DURATION_SEC"
    echo "Trial duration reached (${TRIAL_DURATION_SEC}s); stopping SAC motion..."
    : > "$TRIAL_DIR/trial_timeout.flag"
    kill -INT "$BRIDGE_PID" 2>/dev/null || true
  ) &
  TRIAL_TIMER_PID=$!
fi

BRIDGE_STATUS=0
while kill -0 "$BRIDGE_PID" 2>/dev/null; do
  if [ -f "$TRIAL_DIR/goal_reached.flag" ]; then
    echo "Goal reached; stopping SAC motion..."
    stop_process "$TRIAL_TIMER_PID" "trial timer"
    stop_process "$BRIDGE_PID" "SAC motion" 50
    BRIDGE_STATUS=0
    break
  fi
  if [ -f "$TRIAL_DIR/trial_timeout.flag" ]; then
    stop_process "$BRIDGE_PID" "SAC motion" 50
    BRIDGE_STATUS=0
    break
  fi
  sleep 0.2
done

if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  wait "$BRIDGE_PID" 2>/dev/null
  BRIDGE_STATUS=$?
  if [ -f "$TRIAL_DIR/goal_reached.flag" ]; then
    BRIDGE_STATUS=0
  fi
fi
finish "$BRIDGE_STATUS"
