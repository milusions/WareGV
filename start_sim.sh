colcon build
source install/setup.sh
export MAP_NAME=${2:-"small_warehouse"}
ros2 launch waregv_description gazebo.launch.py mapping:=${1:-"true"}