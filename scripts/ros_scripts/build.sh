
origin_directory=$(pwd)
cd ..
cd ..
cd ros2_ws/

source /opt/ros/foxy/setup.bash

packages_to_ignore=(
    
)


colcon build --packages-select scion_types
source install/setup.bash

# Perform colcon build, ignoring the specified packages
colcon build --packages-ignore "${packages_to_ignore[@]}"

source install/setup.bash

cd $origin_directory

