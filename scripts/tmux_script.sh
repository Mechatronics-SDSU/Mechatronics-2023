#!/bin/bash
# @Zix
# Tmux script makes pool testing easier by creating session inside robot (presumably already SSHd in from client)
# Panes are organized and access three directories - scripts, ros_ws, and src (launch file)
# All windows will have ROS sourced so I can keep my sanity one day longer
# VSCode will also be opened if it is not already

tmux new-session -d -s robot        # I named it robot because i felt like it

tmux setw -g mouse on               # This is so you can click between panes

# Split the session into a small bottom pane and split top horizontally
tmux split-window -d -p 10\; split-window -h\; split-window -d\; split-window -d;

# Run commands in each pane - 0:i is used to identify pane

tmux send-keys -t robot:0.0 "cd /home/mechatronics/master/Mechatronics-2023/ros2_ws; 
. install/setup.bash; 
cd src;
clear" Enter

tmux send-keys -t robot:0.1 "cd /home/mechatronics/master/Mechatronics-2023/ros2_ws;
. install/setup.bash
clear" Enter


tmux send-keys -t robot:0.2 "cd /home/mechatronics/master/Mechatronics-2023/ros2_ws;
. install/setup.bash
clear" Enter

tmux send-keys -t robot:0.3 "cd /home/mechatronics/master/Mechatronics-2023/ros2_ws;
. install/setup.bash
clear" Enter

# Here we open VScode if not already opened and then source and open scripts
tmux send-keys -t robot:0.4 "cd /home/mechatronics/master/Mechatronics-2023;
if pgrep -x "code" > /dev/null; then
    echo "VSCode is running"
else
    code .
fi;
clear;
cd /home/mechatronics/master/Mechatronics-2023/ros2_ws;
. install/setup.bash;
cd ..;
cd scripts
clear" Enter

# Attach to the tmux session to keep the panes open
tmux attach-session -t robot

# I made another script called kill_tmux.sh which will help you get out of the session