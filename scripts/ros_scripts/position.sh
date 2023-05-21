echo "Publishing ros message of position..."

ros2 topic pub -1 zed_position_data scion_types/msg/Position "{position: [$1, $2, $3]}"
