#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 /path/to/output_map_without_extension"
  exit 2
fi

MAP_NAME=$1

map_server_pkg="nav""2_map_server"
ros2 run "$map_server_pkg" map_saver_cli \
  -f "$MAP_NAME" \
  --ros-args -p save_map_timeout:=10.0
