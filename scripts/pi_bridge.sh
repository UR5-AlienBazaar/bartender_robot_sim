#!/bin/bash
source /opt/ros/humble/setup.bash
exec python3 /workspace/scripts/pi_topic_bridge_client.py "$@"
