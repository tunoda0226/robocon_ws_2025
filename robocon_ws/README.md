# robocon_ws
# robocon_ws_2025
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
[11:55]
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # または rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0
source ~/.bashrc
[11:58]
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
