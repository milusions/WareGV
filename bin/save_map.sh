#!/bin/sh
cd ~/waregv/waregv_ws

mkdir "./src/waregv_mapping/maps/${1:-"recent_map"}"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: '~/waregv/waregv_ws/src/waregv_mapping/maps/${1:-"recent_map"}/${1:-"recent_map"}'}" 