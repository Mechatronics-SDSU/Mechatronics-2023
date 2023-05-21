ask_enable_can()
{
    ENABLE_CAN_MESSAGE='Do you want to enable CAN (y/n): '  
    ENABLE_CAN_ARGS_MESSAGE='This script takes two command line arguments, the first one can be either 0 or 1, type 1 if you need to enable CAN, type 0 if its already enabled.
For the second argument type 3 if you want the robot to be in test mode or 4 for safety mode (stops after 1 second). Type them both on the same line with space in between, then press enter.'

    read -rp $ENABLE_CAN_MESSAGE choice
# Make a decision based on the user's input
    if [[ $choice == "y" ]]; then
        read -rp $ENABLE_CAN_ARGS_MESSAGE arg1 arg2 
        ./software_can/can_enable.sh 0 4
    elif [[ $choice == "n" ]]; then
        echo "Choosing to Skip CAN Enable'."
    # Perform actions for 'no' choice
    else
        echo "Invalid choice."
        ask_enable_can
    fi
}

IFS=

ask_enable_can



# ./software_can/can_enable.sh
# ./software_can/device_enable.sh
# ./motor_tests.sh
# ./
