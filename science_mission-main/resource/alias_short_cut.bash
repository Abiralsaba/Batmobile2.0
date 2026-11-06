echo "Science Commands:"
echo "bifrost_node, gps_node, sensor_fusion_node, arm_node, camera_node"

alias bifrost_node='ros2 run science_mission bifrost_node'
alias gps_node='ros2 run science_mission gps_imu_node'
alias sensor_fusion_node='ros2 run science_mission sensor_fusion_node'
alias arm_node='ros2 run science_mission arm_node'
alias camera_node='ros2 run science_mission camera_feed_node'