export ROS_DOMAIN_ID=0
colcon build
source install/setup.sh
ros2 launch waregv_description urdf.launch.py