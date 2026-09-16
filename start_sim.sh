export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build
source install/setup.sh
export MAP_NAME=${3:-"small_warehouse"}
export WORLD_NAME=${4:-"small_warehouse"}
ros2 launch waregv_description gazebo.launch.py mapping:=${1:-"true"} navigation:=${2:-"true"}