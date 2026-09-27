#!/usr/bin/env bash
# Start the MAB arm stack on the rpi4: the driver node (mab_arm_node.py)
# and the HTTP bridge (arm_http_bridge.py) for the VLM-facing /arm/* API
# routes, with DDS bound to the 10.42.0.x network.
#
# Deploy: copy this directory's mab_arm_node.py, arm_http_bridge.py and
# run_arm.sh to the Pi (e.g. ~/pi_code) and run there, next to the CANdle:
#   scp scripts/mab_arm_node.py scripts/arm_http_bridge.py scripts/run_arm.sh \
#       bartender@10.42.0.200:pi_code/
#   ssh bartender@10.42.0.200 'cd pi_code && ./run_arm.sh'
#
# Usage: ./run_arm.sh [--ros-args -p can_ids:='[558, 141]']
# Stop:  Ctrl-C, or pkill -f "python3 .*[m]ab_arm_node"
set -e

HERE="$(dirname "$(realpath "$0")")"
source /opt/ros/humble/setup.bash

# Must match on the Pi and on every machine that sends commands.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Find the interface that has a 10.42.0.x address (hotspot or client)
IFACE=$(ip -o -4 addr show | awk '$4 ~ /^10\.42\.0\./ {print $2; exit}')
if [ -z "$IFACE" ]; then
    echo "No interface with a 10.42.0.x address found. Is the network up?" >&2
    exit 1
fi
ADDR=$(ip -o -4 addr show "$IFACE" | awk '{split($4, a, "/"); print a[1]}')
echo "Arm node on interface $IFACE ($ADDR)"

# UDP only on the 10.42.0.x address, shared memory for nodes on the Pi itself.
# Reuses the FastDDS profile that run.sh writes, or writes its own if absent.
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/.fastdds_arm.xml"
cat > "$FASTRTPS_DEFAULT_PROFILES_FILE" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>lan</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>$ADDR</address></interfaceWhiteList>
      <maxMessageSize>1400</maxMessageSize>
      <sendBufferSize>16777216</sendBufferSize>
    </transport_descriptor>
    <transport_descriptor>
      <transport_id>shm</transport_id>
      <type>SHM</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="pi" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>lan</transport_id><transport_id>shm</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin><initialPeersList>
        <locator><udpv4><address>239.255.0.1</address></udpv4></locator>
      </initialPeersList></builtin>
    </rtps>
  </participant>
</profiles>
EOF

python3 "$HERE/arm_http_bridge.py" >> ~/arm_bridge.log 2>&1 &
BRIDGE_PID=$!
python3 "$HERE/mab_arm_node.py" "$@"
kill $BRIDGE_PID
