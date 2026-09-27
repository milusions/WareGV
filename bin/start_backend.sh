#!/bin/bash

# =========================================================
#  Milusions WareGV — REST backend launcher
#  Starts the FastAPI/ROS bridge on port 8000 in its own
#  process so it survives mode switches and sim restarts.
#
#  --sim  -> use_sim_time=true,  WAREGV_LAUNCH_FILE=simulation.launch.py
#  --real -> use_sim_time=false, WAREGV_LAUNCH_FILE=hardware.launch.py
# =========================================================

PRIMARY="\e[38;2;0;119;255m"
BOLD_WHITE="\e[1;97m"
DIM_GRAY="\e[38;2;120;120;120m"
ACCENT_GREEN="\e[38;2;0;200;100m"
ACCENT_RED="\e[38;2;227;0;53m"
RESET="\e[0m"

USE_SIM_TIME="false"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORKSPACE="$HOME/waregv/waregv_ws"
LOG_DIR="$WORKSPACE/logs"
LOG_FILE="$LOG_DIR/backend.log"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim)
            USE_SIM_TIME="true"
            export WAREGV_LAUNCH_FILE=simulation.launch.py
            shift ;;
        --real)
            USE_SIM_TIME="false"
            export WAREGV_LAUNCH_FILE=hardware.launch.py
            shift ;;
        --workspace)
            WORKSPACE="$2"; shift 2 ;;
        --ros-setup)
            ROS_SETUP="$2"; shift 2 ;;
        --help|-h)
            echo "Usage: $0 [--sim | --real] [--workspace <path>] [--ros-setup <path>]"
            echo ""
            echo "  --sim           use_sim_time=true, launch target: simulation.launch.py"
            echo "  --real          use_sim_time=false, launch target: hardware.launch.py (default)"
            echo "  --workspace     ROS 2 workspace path (default: ~/waregv/waregv_ws)"
            echo "  --ros-setup     ROS 2 setup.bash  (default: /opt/ros/jazzy/setup.bash)"
            exit 0 ;;
        *)
            echo -e "${ACCENT_RED}Unknown argument:${RESET} $1"
            echo "Run with --help for usage."
            exit 1 ;;
    esac
done

if [[ ! -f "$ROS_SETUP" ]]; then
    echo -e "${ACCENT_RED}ROS setup not found:${RESET} $ROS_SETUP"
    exit 1
fi

if [[ ! -d "$WORKSPACE" ]]; then
    echo -e "${ACCENT_RED}Workspace not found:${RESET} $WORKSPACE"
    exit 1
fi

mkdir -p "$LOG_DIR"

# Kill any previous backend.
if pgrep -f "waregv_suite_backend" > /dev/null; then
    echo -e "${DIM_GRAY}Stopping previous backend...${RESET}"
    pkill -f "waregv_suite_backend"
    sleep 1
fi

# Free port 8000 if anything else grabbed it.
if command -v fuser > /dev/null 2>&1; then
    if fuser 8000/tcp > /dev/null 2>&1; then
        echo -e "${DIM_GRAY}Port 8000 busy — killing holder...${RESET}"
        fuser -k 8000/tcp > /dev/null 2>&1 || true
        sleep 1
    fi
fi

echo -e "  ${PRIMARY}[SOURCING]${RESET}  ${DIM_GRAY}$ROS_SETUP${RESET}"
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
    echo -e "  ${PRIMARY}[SOURCING]${RESET}  ${DIM_GRAY}$WORKSPACE/install/setup.bash${RESET}"
    # shellcheck disable=SC1090
    source "$WORKSPACE/install/setup.bash"
else
    echo -e "${ACCENT_RED}No install/setup.bash found — build with colcon first.${RESET}"
    exit 1
fi

clear
echo -e "${PRIMARY}====================================================${RESET}"
echo -e "${PRIMARY}  M I L U S I O N S   W A R E G V${RESET} ${DIM_GRAY}(Backend)${RESET}"
echo -e "${PRIMARY}====================================================${RESET}"
echo -e "  ${BOLD_WHITE}use_sim_time:${RESET}     ${PRIMARY}${USE_SIM_TIME}${RESET}"
echo -e "  ${BOLD_WHITE}Launch target:${RESET}    ${PRIMARY}${WAREGV_LAUNCH_FILE:-auto}${RESET}"
echo -e "  ${BOLD_WHITE}Port:${RESET}             ${DIM_GRAY}8000${RESET}"
echo -e "  ${BOLD_WHITE}Log File:${RESET}         ${DIM_GRAY}${LOG_FILE}${RESET}"
echo -e "${PRIMARY}----------------------------------------------------${RESET}"
echo -e "  ${ACCENT_GREEN}[RUNNING]${RESET}     ${BOLD_WHITE}REST API:${RESET} http://0.0.0.0:8000/"
echo -e "  ${DIM_GRAY}                  Press Ctrl+C to stop.${RESET}"
echo -e "${PRIMARY}====================================================${RESET}"
echo ""

stdbuf -oL -eL ros2 run waregv_suite waregv_suite_backend \
    --ros-args -p use_sim_time:="$USE_SIM_TIME" \
    2>&1 | tee "$LOG_FILE"