#!/bin/bash
cmds=("ros2 launch depthai_examples stereo_inertial_node.launch.py"
	"ros2 launch imu_complementary_filter complementary_filter.launch.py"
	"ros2 run vins vins_node /home/rangeronmars/ros2_ws/src/TUP2023-Sentry-Framework/TUP2023-Sentry-Nav/VINS-Fusion-ROS2/config/oak-d-pro-400p/oak-rectfied.yaml"
	"ros2 launch grid_map_demos pointcloud2_to_gridmap_demo_launch.py"
	"ros2 launch nav2_bringup sentry_launch.py"
	"ros2 run ros2_serial_demo ros2_serial_demo" )
# OAK
for cmd in "${cmds[@]}";
do
echo Current CMD : "$cmd"
gnome-terminal -- bash -c "  source /opt/ros/galactic/setup.bash;cd $(pwd);source install/setup.bash;$cmd;exec bash;"
sleep 0.2
done
