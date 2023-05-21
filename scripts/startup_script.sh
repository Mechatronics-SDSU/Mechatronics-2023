#!/bin/bash
# @Zix - New Startup Script Baby

change_script_permissions()
{

original_dir=$(pwd)

directories=(
  $(pwd)/software_can
  $(pwd)/motor_tests
  $(pwd)/ros_scripts
  $(pwd)/tmux
)

for directory in "${directories[@]}"; do

    # Change to the directory
    cd "$directory" || exit 1

    # Iterate over files in the directory
    for file in *; do
    # Check if the file is a regular file and not a directory
    if [ -f "$file" ]; then
        # Make the file executable
        chmod +x "$file"
        echo "Enabled execution for $file"
    fi
    done

    cd $original_dir

done 

echo "All scripts in the directory are now enabled."
}

ask_enable_can()
{
    ENABLE_CAN_MESSAGE='Do you want to enable CAN (y/n): '  
    ENABLE_CAN_ARGS_MESSAGE='This script takes two command line arguments, the first one can be either 0 or 1, type 1 if you need to enable CAN, type 0 if its already enabled.
For the second argument type 3 if you want the robot to be in test mode or 4 for safety mode (stops after 1 second). Type them both on the same line with space in between, then press enter.'

    read -rp $ENABLE_CAN_MESSAGE choice
# Make a decision based on the user's input
    if [[ $choice == "y" ]]; then
        read -rp $ENABLE_CAN_ARGS_MESSAGE arg1 arg2 
        ./software_can/can_enable.sh "$arg1" "$arg2"
    elif [[ $choice == "n" ]]; then
        echo "Choosing to Skip CAN Enable'."
    else
        echo "Invalid choice."
        ask_enable_can
    fi
}

ask_device_enable()
{
    ENABLE_DEVICE_MESSAGE='Do you want to enable any CAN Devices (y/n): '
    ENABLE_DEVICE_ARGS_MESSAGE='This takes flags type -dvl if you want to enable all devices, or just -dv for example for only Depth sensor and DVL'
    
    read -rp $ENABLE_DEVICE_MESSAGE choice
# Make a decision based on the user's input
    if [[ $choice == "y" ]]; then
        read -rp $ENABLE_DEVICE_ARGS_MESSAGE arg1 
        ./software_can/device_enable.sh arg1
    elif [[ $choice == "n" ]]; then
        echo "Choosing to Skip CAN Enable'."
    else
        echo "Invalid choice."
    fi
}

ask_motor_tests()
{
    RUN_MOTORS_MESSAGE='Do you want to run motor tests (y/n): '
    read -rp $RUN_MOTORS_MESSAGE choice
    # Make a decision based on the user's input
    if [[ $choice == "y" ]]; then
        ./motor_tests.sh
    elif [[ $choice == "n" ]]; then
        echo "Choosing to Skip CAN Enable'."
    else
        echo "Invalid choice."
    fi  
}

ask_sensor_tests()
{
    
}

IFS=

change_script_permissions
ask_enable_can
ask_device_enable
ask_motor_tests