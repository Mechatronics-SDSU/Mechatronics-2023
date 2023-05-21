#!/bin/bash

# @ Zix
# Setup CAN Script for Pool Testing or Robot Activation

# First terminal must be connected to main orin through SSH and then start tmux/start

# Takes two command line arguments - the first one is a boolean if we want to enable CAN, the second one can 
# take the value of 3 or 4, and determines the mode of the robot for CAN (Joseph's modes; 3 is testing, 4 is performance)

# ENABLE CAN ------------------------------------------------------------------------------>
if [ $1 -eq 1 ]
then
  echo "Enabling CAN....."
  sudo busybox devmem 0x0c303010 w 0xc400
  sudo busybox devmem 0x0c303018 w 0xc458
  sudo modprobe can
  sudo modprobe can_raw
  sudo modprobe mttcan
  sudo ip link set can0 type can bitrate 500000 berr-reporting on loopback off
  sudo ip link set up can0
fi

# status check
tmux send-keys -t robot:0.1 "candump can0" Enter
if [ $2 -eq 3 ]
then
  tmux send-keys -t robot:0.2 "cansend can0 022#0000000003 " Enter
  tmux send-keys -t robot:0.2 "cansend can0 020#00000000 " Enter
fi

if [ $2 -eq 4 ]
then
  tmux send-keys -t robot:0.2 "cansend can0 022#0000000004 " Enter
  tmux send-keys -t robot:0.2 "cansend can0 020#00000000 " Enter
fi

# RUN MOTOR TEST ------------------------------------------------------------------------->
# /bin/bash motor_tests/motor_test.sh

# RUN DEEP MOTOR TEST -------------------------------------------------------------------->
# /bin/bash motor_tests/deep_motor_test.sh

# RUN SENSORS TESTS ---------------------------------------------------------------------->

exit 0
