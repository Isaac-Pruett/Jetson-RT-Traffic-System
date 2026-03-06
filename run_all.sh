#!/bin/bash 

cd "$(dirname "$0")"

#ws="/home/nvidia/Jetson-RT-Traffic-Sytem"

# Terminal 3 NAME:
BASE_FILE="$1"

# Terminal 1:  Image view    
gnome-terminal -- bash -c "source install/setup.bash; ros2 run rqt_image_view rqt_image_view; exec bash"
sleep 2

# Terminal 2: Deepstream
#gnome-terminal -- bash -c "source install/setup.bash; ros2 launch launch/deepstream_live.py; exec bash"

# Terminal 2: CSV Subcriber and Publisher
gnome-terminal -- bash -c "source install/setup.bash; ros2 run csv_process csv_process ${BASE_FILE}; exec bash"
sleep 5

# Terminal 3: Deepstream
gnome-terminal -- bash -c "source install/setup.bash; ros2 launch launch/deepstream_vid.py; exec bash"
sleep 2

# Terminal 4: Cluster Algorithm

#gnome-terminal -- bash -c "source install/setup.bash; ros2 run cluster_algorithm cluster_algorithm; exec bash"


