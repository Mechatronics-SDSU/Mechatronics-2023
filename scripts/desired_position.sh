# @Zix
# Just to not have to memorize the ROS command

ros2 topic pub -1 desired_position scion_types/msg/DesiredState "{desired_state: [$1, $2, $3, $4, $5, $6]}"
