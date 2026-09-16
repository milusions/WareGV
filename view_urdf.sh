export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build
source install/setup.sh
ros2 launch waregv_description urdf.launch.py