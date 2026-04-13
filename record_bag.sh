#!/bin/bash
# rosbag kayit betigi
# Navigasyon sirasinda onemli topic'leri kaydeder.
#
# Kullanim:
#   chmod +x record_bag.sh
#   ./record_bag.sh [bag_ismi]

BAG_NAME=${1:-"navigation_experiment_$(date +%Y%m%d_%H%M%S)"}
OUTPUT_DIR=~/catkin_ws/src/ros_navigation/results/bags
mkdir -p $OUTPUT_DIR

echo "rosbag kaydi baslatiliyor: ${BAG_NAME}"
echo "Cikis dizini: ${OUTPUT_DIR}"
echo "Durdurmak icin Ctrl+C"

rosbag record -O "${OUTPUT_DIR}/${BAG_NAME}" \
    /odom \
    /scan \
    /cmd_vel \
    /move_base/NavfnROS/plan \
    /move_base/GlobalPlanner/plan \
    /move_base/DWAPlannerROS/local_plan \
    /move_base/DWAPlannerROS/global_plan \
    /move_base/status \
    /move_base/result \
    /move_base/goal \
    /amcl_pose \
    /tf \
    /tf_static \
    /map
