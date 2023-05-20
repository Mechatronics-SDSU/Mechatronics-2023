#!/bin/bash

tmux new-session -d -s robot

tmux setw -g mouse on

# Split the session into four panes
tmux split-window -d -p 15\; split-window -h

# Run commands in each pane
tmux send-keys -t robot:0.0 "cd /home/mechatronics/master/Mechatronics-2023;
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

tmux send-keys -t robot:0.1 "cd /home/mechatronics/master/Mechatronics-2023/ros2_ws; 
. install/setup.bash; 
cd src;
clear" Enter

tmux send-keys -t robot:0.2 "cd /home/mechatronics/master/Mechatronics-2023/ros2_ws;
. install/setup.bash
clear" Enter

# Attach to the tmux session to keep the panes open
tmux attach-session -t robot
