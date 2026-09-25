#!/bin/bash

# --- Color Definitions ---
PRIMARY="\e[38;2;0;119;255m"
BOLD_WHITE="\e[1;97m"
DIM_GRAY="\e[38;2;120;120;120m"
ACCENT_GREEN="\e[38;2;0;200;100m"
RESET="\e[0m"

LOG_DIR="$HOME/waregv/waregv_ws/logs"
LOG_FILE="$LOG_DIR/hardware.log"
mkdir -p "$LOG_DIR"

# Clean signal handling to kill background tasks on exit
trap 'kill $(jobs -p) 2>/dev/null' EXIT INT TERM

echo "Waiting for network interfaces to come up..."
while ! ip link show up | grep -q "lo"; do
    sleep 1
done

MAX_LINEAR_VELOCITY="0.11"
MAX_ANGULAR_VELOCITY="0.35"
WHEEL_RADIUS="0.036"
track_width="0.192"
MAP_NAME="small_warehouse"

while [[ $# -gt 0 ]]; do
    case "$1" in
    
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
            track_width="$2"
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

# Clear old log file content before running
> "$LOG_FILE"

cd ~/waregv/waregv_ws

echo -e "  ${PRIMARY}[BUILDING]${RESET}   ${BOLD_WHITE}Compiling ROS 2 workspace (colcon build)...${RESET}"
# Removed > /dev/null 2>&1 to allow the build process to print to the screen
colcon build

echo -e "  ${PRIMARY}[SOURCING]${RESET}   ${DIM_GRAY}Loading ROS 2 Jazzy environment setup...${RESET}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Execute launch in background
stdbuf -oL -eL ros2 launch waregv_bringup hardware.launch.py \
    max_linear_velocity:="$MAX_LINEAR_VELOCITY" \
    max_angular_velocity:="$MAX_ANGULAR_VELOCITY" \
    wheel_radius:="$WHEEL_RADIUS" \
    track_width:="$track_width" \
    map_name:="$MAP_NAME" > "$LOG_FILE" 2>&1 &

LAUNCH_PID=$!

# Print the status card once instead of looping and clearing
clear
echo -e "${PRIMARY}====================================================${RESET}"
echo -e "${PRIMARY}  M I L U S I O N S   W A R E G V${RESET} ${DIM_GRAY}(Hardware)${RESET}"
echo -e "${PRIMARY}====================================================${RESET}"
echo -e "  ${BOLD_WHITE}System Mode:${RESET}  ${PRIMARY}${MODE}${RESET}"
echo -e "  ${BOLD_WHITE}Target Map:${RESET}   ${DIM_GRAY}${MAP_NAME}${RESET}"
echo -e "  ${BOLD_WHITE}Log File:${RESET}     ${DIM_GRAY}${LOG_FILE}${RESET}"
echo -e "${PRIMARY}----------------------------------------------------${RESET}"
echo -e "  ${ACCENT_GREEN}[RUNNING]${RESET}    ${BOLD_WHITE}Live Output:${RESET}\n"

# Stream the entire log continuously
tail -f "$LOG_FILE" &
TAIL_PID=$!

# Keep the script running until the ROS launch process exits
wait $LAUNCH_PID