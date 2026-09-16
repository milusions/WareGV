
colcon build
source install/setup.sh
ros2 launch waregv_description waregv.launch.py mapping:=${1:-"true"} navigation:=${2:-"true"}