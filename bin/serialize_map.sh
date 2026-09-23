#!/bin/sh
cd ~/waregv/waregv_ws

mkdir "./src/waregv_mapping/maps/${1:-"recent_map"}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "filename: '~/waregv/waregv_ws/src/waregv_mapping/maps/${1:-"recent_map"}/${1:-"recent_map"}'}" 
