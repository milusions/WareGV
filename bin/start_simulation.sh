#!/bin/bash

echo "Waiting for network interfaces to come up..."
while ! ip link show up | grep -q "lo"; do
    sleep 1
done

export DISPLAY=:0
export XAUTHORITY=$HOME/.Xauthority

echo "Waiting for X11 Display Server ($DISPLAY)..."
for i in {1..30}; do
    if xset q &>/dev/null; then
        echo "Display server detected successfully."
        break
    fi
    if [ $i -eq 30 ]; then
        echo "Warning: Display server not found. Gazebo may fail if GUI is enabled."
    fi
    sleep 1
done

echo "Allowing system resources 5 seconds to settle..."
sleep 5


MAPPING_ENABLE="true"
NAVIGATION_ENABLE="true"
WORLD_NAME="small_warehouse"
MAX_LINEAR_VELOCITY="0.5"
MAX_ANGULAR_VELOCITY="3.14159265359"
WHEEL_RADIUS="0.0325"
WHEEL_BASE="0.176"
MODEL="waregv.urdf.xacro"
SPAWN_Z="0.5"
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
        --world-name)
            WORLD_NAME="$2"
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
        --model)
            MODEL="$2"
            shift 2
            ;;
        --spawn-z)
            SPAWN_Z="$2"
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

ros2 launch waregv_bringup simulation.launch.py \
    mapping_enable:="$MAPPING_ENABLE" \
    navigation_enable:="$NAVIGATION_ENABLE" \
    world_name:="$WORLD_NAME" \
    max_linear_velocity:="$MAX_LINEAR_VELOCITY" \
    max_angular_velocity:="$MAX_ANGULAR_VELOCITY" \
    wheel_radius:="$WHEEL_RADIUS" \
    wheel_base:="$WHEEL_BASE" \
    model:="$MODEL" \
    spawn_z:="$SPAWN_Z" \
    map_name:="$MAP_NAME"
