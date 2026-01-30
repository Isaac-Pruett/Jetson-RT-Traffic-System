#!/bin/bash 

cd "$(dirname "$0")"

#ws="/home/nvidia/Jetson-RT-Traffic-Sytem"

# Terminal 3 NAME:
NAME="$1"

# Terminal 1:  Image view    
gnome-terminal -- bash -c "source install/setup.bash; ros2 run rqt_image_view rqt_image_view; exec bash"

sleep 2

# Terminal 2: Deepstream
gnome-terminal -- bash -c "source install/setup.bash; ros2 launch launch/deepstream_vid.py; exec bash"

sleep 2

# Terminal 3: CSV Subcriber and Publisher
gnome-terminal -- bash -c "source install/setup.bash; ros2 run inputCSVSubscriber inputCSVSubscriber ${NAME}; exec bash"

sleep 2

# Terminal 4: Cluster Algorithm

#gnome-terminal -- bash -c "source install/setup.bash; ros2 run cluster_algorithm cluster_algorithm; exec bash"


