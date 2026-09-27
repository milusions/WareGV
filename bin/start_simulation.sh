#!/bin/bash

# --- Color Definitions ---
PRIMARY="\e[38;2;0;119;255m"
BOLD_WHITE="\e[1;97m"
DIM_GRAY="\e[38;2;120;120;120m"
ACCENT_GREEN="\e[38;2;0;200;100m"
ACCENT_RED="\e[38;2;227;0;53m"
RESET="\e[0m"

LOG_DIR="$HOME/waregv/waregv_ws/logs"
LOG_FILE="$LOG_DIR/simulation.log"
mkdir -p "$LOG_DIR"

# Clean signal handling to kill background tasks on exit
trap 'kill $(jobs -p) 2>/dev/null' EXIT INT TERM

# -----------------------------
# Defaults
# -----------------------------
MODE="slam_only"                # slam_only | slam_with_nav2 | nav2_with_amcl
WORLD_NAME="small_warehouse"
MAX_LINEAR_VELOCITY="0.3"
MAX_ANGULAR_VELOCITY="0.35"
WHEEL_RADIUS="0.035"
TRACK_WIDTH="0.168"
MODEL="waregv.urdf.xacro"
SPAWN_Z="0.5"
MAP_NAME=""                     # only used in nav2_with_amcl
SKIP_BUILD=0

# -----------------------------
# Help
# -----------------------------
print_help() {
    echo -e "${PRIMARY}Usage:${RESET} $0 [OPTIONS]"
    echo ""
    echo -e "${BOLD_WHITE}Options:${RESET}"
    echo "  --mode <slam_only|slam_with_nav2|nav2_with_amcl>   Simulation mode (default: slam_only)"
    echo "  --map-name <name>                                   Loaded map for nav2_with_amcl mode"
    echo "  --world-name <name>                                 Gazebo world (default: small_warehouse)"
    echo "  --max-linear-velocity <m/s>                         Default: 0.3"
    echo "  --max-angular-velocity <rad/s>                      Default: 0.35"
    echo "  --wheel-radius <m>                                  Default: 0.035"
    echo "  --wheel-base <m>                                    Track width. Default: 0.168"
    echo "  --model <filename>                                  Default: waregv.urdf.xacro"
    echo "  --spawn-z <m>                                       Robot spawn height. Default: 0.5"
    echo "  --skip-build                                        Skip colcon build (use existing install/)"
    echo "  --help                                              Show this message"
    exit 0
}

# -----------------------------
# Argument parsing
# -----------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)                   MODE="$2";                shift 2 ;;
        --map-name)               MAP_NAME="$2";            shift 2 ;;
        --world-name)             WORLD_NAME="$2";          shift 2 ;;
        --max-linear-velocity)    MAX_LINEAR_VELOCITY="$2"; shift 2 ;;
        --max-angular-velocity)   MAX_ANGULAR_VELOCITY="$2"; shift 2 ;;
        --wheel-radius)           WHEEL_RADIUS="$2";        shift 2 ;;
        --wheel-base)             TRACK_WIDTH="$2";         shift 2 ;;
        --model)                  MODEL="$2";               shift 2 ;;
        --spawn-z)                SPAWN_Z="$2";             shift 2 ;;
        --skip-build)             SKIP_BUILD=1;             shift   ;;
        --help|-h)                print_help ;;
        *) echo -e "${ACCENT_RED}Unknown argument:${RESET} $1"; echo "Run with --help to see options."; exit 1 ;;
    esac
done

# -----------------------------
# Validate mode
# -----------------------------
case "$MODE" in
    slam_only|slam_with_nav2|nav2_with_amcl) ;;
    *)
        echo -e "${ACCENT_RED}Invalid mode:${RESET} $MODE"
        echo "Valid modes: slam_only | slam_with_nav2 | nav2_with_amcl"
        exit 1
        ;;
esac

# For nav2_with_amcl, a map name is required
if [[ "$MODE" == "nav2_with_amcl" && -z "$MAP_NAME" ]]; then
    echo -e "${ACCENT_RED}Error:${RESET} --map-name is required when --mode nav2_with_amcl"
    echo "Example: $0 --mode nav2_with_amcl --map-name small_warehouse"
    exit 1
fi

# -----------------------------
# Wait for loopback (sanity check)
# -----------------------------
echo -e "${DIM_GRAY}Waiting for network interfaces to come up...${RESET}"
while ! ip link show up | grep -q "lo"; do
    sleep 1
done

# -----------------------------
# Build + source
# -----------------------------
cd ~/waregv/waregv_ws || { echo -e "${ACCENT_RED}cannot cd to workspace${RESET}"; exit 1; }

if [[ "$SKIP_BUILD" -eq 0 ]]; then
    echo -e "  ${PRIMARY}[BUILDING]${RESET}   ${BOLD_WHITE}Compiling ROS 2 workspace (colcon build)...${RESET}"
    colcon build
else
    echo -e "  ${PRIMARY}[SKIP]${RESET}       ${DIM_GRAY}colcon build skipped${RESET}"
fi

echo -e "  ${PRIMARY}[SOURCING]${RESET}   ${DIM_GRAY}Loading ROS 2 Jazzy environment setup...${RESET}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# -----------------------------
# Clear old log
# -----------------------------
> "$LOG_FILE"

# -----------------------------
# Build launch arguments
# -----------------------------
LAUNCH_ARGS=(
    "world_name:=$WORLD_NAME"
    "max_linear_velocity:=$MAX_LINEAR_VELOCITY"
    "max_angular_velocity:=$MAX_ANGULAR_VELOCITY"
    "wheel_radius:=$WHEEL_RADIUS"
    "track_width:=$TRACK_WIDTH"
    "model:=$MODEL"
    "spawn_z:=$SPAWN_Z"
    "mode:=$MODE"
)

# Only pass map_name if the user gave one; the launch file defaults it.
if [[ -n "$MAP_NAME" ]]; then
    LAUNCH_ARGS+=("map_name:=$MAP_NAME")
fi

# -----------------------------
# Launch
# -----------------------------
stdbuf -oL -eL ros2 launch waregv_bringup simulation.launch.py \
    "${LAUNCH_ARGS[@]}" > "$LOG_FILE" 2>&1 &

LAUNCH_PID=$!

# -----------------------------
# Status banner
# -----------------------------
clear
echo -e "${PRIMARY}====================================================${RESET}"
echo -e "${PRIMARY}  M I L U S I O N S   W A R E G V${RESET} ${DIM_GRAY}(Simulation)${RESET}"
echo -e "${PRIMARY}====================================================${RESET}"
echo -e "  ${BOLD_WHITE}System Mode:${RESET}  ${PRIMARY}${MODE}${RESET}"
if [[ -n "$MAP_NAME" ]]; then
    echo -e "  ${BOLD_WHITE}Map Name:${RESET}     ${DIM_GRAY}${MAP_NAME}${RESET}"
fi
echo -e "  ${BOLD_WHITE}World Name:${RESET}   ${DIM_GRAY}${WORLD_NAME}${RESET}"
echo -e "  ${BOLD_WHITE}Robot Model:${RESET}  ${DIM_GRAY}${MODEL}${RESET}"
echo -e "  ${BOLD_WHITE}Log File:${RESET}     ${DIM_GRAY}${LOG_FILE}${RESET}"
echo -e "  ${BOLD_WHITE}Launch PID:${RESET}   ${DIM_GRAY}${LAUNCH_PID}${RESET}"
echo -e "${PRIMARY}----------------------------------------------------${RESET}"
echo -e "  ${ACCENT_GREEN}[RUNNING]${RESET}    ${BOLD_WHITE}Live Output:${RESET}"
echo -e "${PRIMARY}----------------------------------------------------${RESET}"

# Stream the entire log continuously
tail -f "$LOG_FILE" &

# Keep the script running until the ROS launch process exits
wait $LAUNCH_PID