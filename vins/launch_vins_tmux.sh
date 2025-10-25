#!/bin/bash

SESSION="vins"

# Kill any existing session
tmux kill-session -t $SESSION 2>/dev/null

# Start new session
tmux new-session -d -s $SESSION

# Pane 0: Build + rosrun (left column)
##tmux send-keys -t $SESSION 'cd ~/Algs/vins && chmod +x build.sh && ./build.sh && source ~/Algs/vins/devel/setup.bash && rosrun vins vins_node ~/Algs/vins/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml' C-m
tmux send-keys -t $SESSION 'cd ~/Algs/vins && chmod +x build.sh && ./build.sh && source ~/Algs/vins/devel/setup.bash && rosrun vins vins_node ~/Algs/vins/src/VINS-Fusion/config/oivio/oivio_mono_imu_config.yaml' C-m

# Split right vertically for roscore
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION:0.1 'source ~/Algs/vins/devel/setup.bash && roscore' C-m

# Split Pane 1 horizontally into roscore (top) and roslaunch (bottom)
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.2 'sleep 2 && source ~/Algs/vins/devel/setup.bash && roslaunch vins vins_rviz.launch' C-m

# Split Pane 2 again into roslaunch (top) and rosbag (bottom)
tmux split-window -v -t $SESSION:0.2
##tmux send-keys -t $SESSION:0.3 'sleep 4 && source ~/Algs/vins/devel/setup.bash && rosbag play ~/Datasets/euroc_data/MH_01_easy.bag --clock --pause' C-m
tmux send-keys -t $SESSION:0.3 'sleep 4 && source ~/Algs/vins/devel/setup.bash && rosbag play ~/Datasets/dso_oivio/MN_015_GV_01/cam_april.bag --clock --pause' C-m

# Attach to the tmux session
tmux select-pane -t $SESSION:0.0
tmux attach-session -t $SESSION
