
origin_directory=$(pwd)
cd ..
cd ..
cd ros2_ws/

source /opt/ros/foxy/setup.bash

colcon build --packages-select scion_types
source install/setup.bash

packages_to_ignore=(
    pid_node
    unified_can_driver
)

# Perform colcon build, ignoring the specified packages
colcon build --packages-ignore "${packages_to_ignore[@]}"

cd $origin_directory
