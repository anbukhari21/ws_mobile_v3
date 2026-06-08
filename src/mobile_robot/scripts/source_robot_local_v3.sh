#!/usr/bin/env bash
# Source this file on the robot before running ws_mobile_v3 ROS commands.
# It keeps control local to the robot and uses the robot's local Fast-DDS
# discovery server instead of the laptop-facing address.

export WORKSPACE="${WORKSPACE:-$HOME/ws_mobile_v3}"

source /opt/ros/humble/setup.bash

if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  source "$WORKSPACE/install/setup.bash"
fi

# The robot boot services are configured on domain 10. Force it here so an
# old terminal value such as ROS_DOMAIN_ID=0 cannot silently isolate the stack.
export ROS_DOMAIN_ID="${ROBOT_ROS_DOMAIN_ID:-10}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_SUPER_CLIENT="${ROS_SUPER_CLIENT:-TRUE}"
export ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-127.0.0.1:11811}"

if [[ -d "$WORKSPACE/.venv" ]]; then
  export VIRTUAL_ENV="$WORKSPACE/.venv"
  export PATH="$VIRTUAL_ENV/bin:$PATH"
  venv_site_packages=$("$VIRTUAL_ENV/bin/python" -c 'import site; print(site.getsitepackages()[0])' 2>/dev/null || true)
  if [[ -n "$venv_site_packages" ]]; then
    export PYTHONPATH="$venv_site_packages${PYTHONPATH:+:$PYTHONPATH}"
  fi
fi

echo "WORKSPACE=$WORKSPACE"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
echo "VIRTUAL_ENV=${VIRTUAL_ENV:-none}"
