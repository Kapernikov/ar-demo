#!/bin/bash
if [ -z "$1" ]; then
  echo "Syntax: $0 <camera_name>"
  exit 1
fi
rostopic pub -1 /camera/$1/control ar_demo/ControlCamera '{reference: "manual_control", stream: true, reset_to_default_settings: true, adjust_target_brightness: 1.0}'
