#!/bin/bash

LAYOUT_PATH="$HOME/.config/terminator/vins_layout"

mkdir -p "$(dirname "$LAYOUT_PATH")"

cat > "$LAYOUT_PATH" <<EOF
[global_config]
  enabled_plugins = LaunchpadCodeURLHandler, APTURLHandler

[keybindings]

[profiles]
  [[default]]
    use_system_font = False
    font = Monospace 10

[layouts]
  [[vins_layout]]
    [[[child1]]]
      parent = window0
      type = Terminal
      profile = default
      command = bash -c "cd ~/Algs/vins && chmod +x build.sh && ./build.sh && source ~/Algs/vins/devel/setup.bash && bash"

    [[[child2]]]
      parent = window0
      type = Terminal
      profile = default
      command = bash -c "source ~/Algs/vins/devel/setup.bash && roscore && bash"

    [[[child3]]]
      parent = window0
      type = Terminal
      profile = default
      command = bash -c "source ~/Algs/vins/devel/setup.bash && roslaunch vins vins_rviz.launch && bash"

    [[[child4]]]
      parent = window0
      type = Terminal
      profile = default
      command = bash -c "source ~/Algs/vins/devel/setup.bash && rosbag play ~/Datasets/euroc_data/MH_01_easy.bag --clock --pause && bash"

    [[[child5]]]
      parent = window0
      type = Terminal
      profile = default
      command = bash -c "source ~/Algs/vins/devel/setup.bash && rosrun vins vins_node ~/Algs/vins/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml && bash"

    [[[window0]]]
      parent = ""
      type = Window
EOF

terminator --layout=vins_layout

