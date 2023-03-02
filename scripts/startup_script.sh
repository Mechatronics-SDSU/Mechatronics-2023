#!/bin/bash

# @ Zix
# Startup Script baby

# First terminal must be connected to main orin through SSH and then open dbus-launch gnome-terminal     <------ This is like really important

# ENABLE CAN ------------------------------------------------------------------------------>
sudo busybox devmem 0x0c303010 w 0xc400
sudo busybox devmem 0x0c303018 w 0xc458
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 500000 berr-reporting on loopback off
sudo ip link set up can0

# RUN MOTOR TEST ------------------------------------------------------------------------->
/bin/bash motor_tests/motor_test.sh

# RUN DEEP MOTOR TEST -------------------------------------------------------------------->
# /bin/bash motor_tests/deep_motor_test.sh

# RUN SENSORS TESTS ---------------------------------------------------------------------->

# *****AHRS*****
gnome-terminal --geometry 80x25+0+0 -- bash -c "cd ../ros2_ws; . install/setup.bash; timeout 4 ros2 run ahrs_node ahrs_exec; exit; exec bash"

sleep 4            # Has to match timeout from the internal shell commands
# **CAN_MAILBOX**
gnome-terminal --geometry 80x25+0+500 -- bash -c "cd ../ros2_ws; . install/setup.bash; timeout 6 ros2 run can2ros_driver can2ros_driver exec bash"

# *****DEPTH*****
gnome-terminal --geometry 80x25+1000+500 -- bash -c "cd ../ros2_ws; . install/setup.bash; timeout 6 ros2 run dres_ms5837_decode dres_ms5837_decode; exit; exec bash"

# *****DVL*****
gnome-terminal --geometry 80x25+1000+0 -- bash -c "cd ../ros2_ws; . install/setup.bash; timeout 6 ros2 run dres_dvl_decode dres_dvl_decode; exit; exec bash"

# *****ZED*****
sleep 6

# CREATE FOUR ROS SOURCED SHELLS IN CORRECT DIRECTORY (OPTION FOR SSH WITH ARG) ---------->

gnome-terminal --geometry 80x25+0+500 -- bash -c "cd ../ros2_ws; . install/setup.bash; cd ..; exec bash"

gnome-terminal --geometry 80x25+1000+500 -- bash -c "cd ../ros2_ws; . install/setup.bash; exec bash"

gnome-terminal --geometry 80x25+0+0 -- bash -c "cd ../ros2_ws; . install/setup.bash; cd src; exec bash"

gnome-terminal --geometry 80x25+1000+0 -- bash -c "exec bash"

exit 0
