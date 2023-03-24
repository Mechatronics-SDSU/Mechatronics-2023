# @Zix
# Just to not have to memorize the ROS command

# ros2 service call /zed2i/zed_node/reset_pos_tracking std_srvs/srv/Trigger

ros2 topic pub -1 desired_state_data scion_types/msg/DesiredState "{desired_state: [$1, $2, $3, $4, $5, $6]}"
