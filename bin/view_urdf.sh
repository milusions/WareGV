cd ~/waregv/waregv_ws

colcon build
source install/setup.sh
ros2 launch waregv_description description.launch.py visualize:=true