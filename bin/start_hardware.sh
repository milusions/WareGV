#!/bin/bash

echo "Waiting for network interfaces to come up..."
while ! ip link show up | grep -q "lo"; do
    sleep 1
done

echo "Allowing system resources 5 seconds to settle..."
sleep 5

MAPPING_ENABLE="true"
NAVIGATION_ENABLE="true"
MAX_LINEAR_VELOCITY="0.5"
MAX_ANGULAR_VELOCITY="3.14159265359"
WHEEL_RADIUS="0.0325"
WHEEL_BASE="0.176"
MAP_NAME="small_warehouse"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mapping-enable)
            MAPPING_ENABLE="$2"
            shift 2
            ;;
        --navigation-enable)
            NAVIGATION_ENABLE="$2"
            shift 2
            ;;
        --max-linear-velocity)
            MAX_LINEAR_VELOCITY="$2"
            shift 2
            ;;
        --max-angular-velocity)
            MAX_ANGULAR_VELOCITY="$2"
            shift 2
            ;;
        --wheel-radius)
            WHEEL_RADIUS="$2"
            shift 2
            ;;
        --wheel-base)
            WHEEL_BASE="$2"
            shift 2
            ;;
        --map-name)
            MAP_NAME="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash


ros2 launch waregv_bringup hardware.launch.py \
    mapping_enable:="$MAPPING_ENABLE" \
    navigation_enable:="$NAVIGATION_ENABLE" \
    max_linear_velocity:="$MAX_LINEAR_VELOCITY" \
    max_angular_velocity:="$MAX_ANGULAR_VELOCITY" \
    wheel_radius:="$WHEEL_RADIUS" \
    wheel_base:="$WHEEL_BASE" \
    map_name:="$MAP_NAME"
