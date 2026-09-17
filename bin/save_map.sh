#!/bin/sh
mkdir "./src/waregv_mapping/maps/${1:-"recent_map"}"
ros2 run nav2_map_server map_saver_cli -f "./src/waregv_description/maps/${1:-"recent_map"}/${1:-"recent_map"}"