echo "Publishing ros message for orientation..."

ros2 topic pub -1 ahrs_orientation_data scion_types/msg/Orientation "{orientation: [$1, $2, $3]}"
