#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
export PYTHONPATH="/usr/local/webots/lib/controller/python:$PYTHONPATH"
export VC_WORLD_META="/scene/webots/worlds/current.json"
HOST="${WEBOTS_HOST:-172.21.224.1}"

echo "[controllers] Connecting to Webots at $HOST:1234..."

export WEBOTS_CONTROLLER_URL="tcp://$HOST:1234/courier"
python3 -u /scene/webots/controllers/courier_driver/courier_driver.py &
DRIVER_PID=$!

export WEBOTS_CONTROLLER_URL="tcp://$HOST:1234/referee"
python3 -u /scene/webots/controllers/referee/referee.py &
REFEREE_PID=$!

wait -n "$DRIVER_PID" "$REFEREE_PID"
