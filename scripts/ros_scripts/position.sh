echo "Publishing ros message of position..."

ros2 topic pub -1 a50_state_data scion_types/msg/State "{state: [$1, $2, $3]}"
