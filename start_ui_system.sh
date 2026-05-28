#!/usr/bin/env bash

sleep 5

JETSON_IP=$(hostname -I | awk '{print $1}')

echo "======================================"
echo "Web otvoris na telefone cez:"
echo "http://$JETSON_IP:8000"
echo "======================================"

gnome-terminal --title="Web Interface" -- bash -lc "
cd /home/jetson/ROS2-Visualization/src/web_interface/web
python3 -m http.server 8000 --bind 0.0.0.0
exec bash
"

sleep 2

gnome-terminal --title="ROSBridge" -- bash -lc "
source /opt/ros/humble/setup.bash
source /home/jetson/ROS2-Visualization/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=0.0.0.0
exec bash
"

sleep 2

gnome-terminal --title="System Controller" -- bash -lc "
source /opt/ros/humble/setup.bash
source /home/jetson/ROS2-Visualization/install/setup.bash
ros2 run pointcloud_publisher system_controller_node
exec bash
"

sleep 3

xdg-open http://localhost:8000
