#@Aeron
#The script clears the contents of all panes/windows

tmux list-panes -F "#{pane_id}" | xargs -I {} tmux send-keys -t {} "clear" Enter
